//==============================================================================
// lut_engine.v
//------------------------------------------------------------------------------
// FastBEV Part2 INT8 LUT engine.
//
// Input:
//   Part1 FP32 feature map in PLDDR, NHWC:
//     [cam][v][u][64 fp32], 256 bytes per pixel, 4 DDR beats per pixel.
//
// LUT:
//   CPU-built table in ZYX voxel order:
//     lut_idx = (z * bev_y + y) * bev_x + x
//   entry format in one 64-bit lane:
//     {pad[63:48], v[47:32], u[31:16], cam_id[15:0]}
//
// Output:
//   Final Decoder INT8 input layout (dims for this delivery):
//     N, Z, C/32, X, Y, C%32   =   1, 4, 8, 200, 200, 32
//     c = c_blk*32 + c_inner ,  c_blk = 0..7 , c_inner = 0..31
//
//   Row-major (rightmost fastest):
//     byte_addr = feat3d_wr_addr
//               + ((((z*(C/32) + c_blk)*bev_x + x)*bev_y + y)*32 + c_inner)
//     -> c_inner stride 1, y stride 32, x stride bev_y*32,
//        c_blk stride bev_x*bev_y*32, z stride (C/32)*bev_x*bev_y*32.
//     Total = 1*4*8*200*200*32 = 40,960,000 bytes.
//
//   512-bit (64-byte) write granularity:
//     c_inner (32 bytes) is half a beat, so one beat packs c_inner=0..31 of
//     two consecutive y values (y even, y+1) for the same (z, c_blk, x):
//       wr_data[127:0]   = (y  , c_inner 0..15)  = src cblkA
//       wr_data[255:128] = (y  , c_inner 16..31) = src cblkB
//       wr_data[383:256] = (y+1, c_inner 0..15)  = src cblkA
//       wr_data[511:384] = (y+1, c_inner 16..31) = src cblkB
//     c_blk even -> (cblkA,cblkB)=(0,1) src ch 0..31
//     c_blk odd  -> (cblkA,cblkB)=(2,3) src ch 32..63
//
// Replication:
//   Source has 64 channels (4 src blocks of 16). Decoder expects 256 channels.
//   Output channel c uses source channel (c % 64): the 8 c_blk values are
//   4 repeats x 2 parity of the 64 source channels.
//   NOTE: this delivery assumes bev_z=4 and even bev_y (=200) so that y forms
//   complete (even,odd) pairs within each x row.
//==============================================================================
`timescale 1ns/1ps

module lut_engine #(
    parameter integer FEAT_MAX_OUTSTANDING = 1
)(
    input                 engine_start,
    output reg            engine_done,

    input        [31:0]   lut_base_addr,
    input        [31:0]   lut_size,
    input        [31:0]   feat2d_base_addr,
    input        [31:0]   feat3d_wr_addr,
    input        [7:0]    bev_x,
    input        [7:0]    bev_y,
    input        [7:0]    bev_z,
    input        [11:0]   img_w,
    input        [11:0]   img_h,

    output       [31:0]   rd_addr,
    output                rd_req,
    input                 rd_grant,
    input        [511:0]  rd_data,
    input                 rd_data_valid,
    output                rd_data_ready,

    output reg   [31:0]   wr_addr,
    output reg   [511:0]  wr_data,
    output                wr_req,
    input                 wr_grant,

    output reg   [31:0]   perf_lut_wait,
    output reg   [31:0]   perf_feat_wait,
    output reg   [31:0]   perf_write_wait,
    output reg   [31:0]   perf_valid_voxel,
    output reg   [31:0]   perf_lut_reads,
    output reg   [31:0]   perf_feat_reads,
    output reg   [31:0]   perf_writes,
    output reg   [31:0]   perf_pipe_stall,

    input                 clk,
    input                 rst_n
);

    localparam S_IDLE          = 5'd0;
    localparam S_PREP_P        = 5'd1;
    localparam S_RD_LUT        = 5'd2;
    localparam S_WAIT_LUT      = 5'd3;
    localparam S_PARSE_LUT     = 5'd4;
    localparam S_CALC_FEAT0    = 5'd5;
    localparam S_CALC_FEAT1    = 5'd6;
    localparam S_CALC_FEAT2    = 5'd19;
    localparam S_CALC_FEAT3    = 5'd20;
    localparam S_CALC_FEAT4    = 5'd22;
    localparam S_RD_FEAT       = 5'd7;
    localparam S_WAIT_FEAT     = 5'd8;
    localparam S_NEXT_P        = 5'd13;
    localparam S_PREP_WRITE_E  = 5'd14;
    localparam S_WR_OUT        = 5'd15;
    localparam S_ADV_GROUP     = 5'd17;
    localparam S_DONE          = 5'd18;
    localparam S_CACHE_HIT     = 5'd23;

    reg [4:0] state, next_state;

    (* MAX_FANOUT = 256 *)
    reg rst_n_int;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rst_n_int <= 1'b0;
        else        rst_n_int <= 1'b1;
    end

    reg [31:0] p_group_base;
    reg [31:0] xy_idx;
    // Registered LUT-space xy index: y * bev_x + x. This removes the
    // cur_y -> multiplier -> lut_batch_addr critical path seen in timing.
    reg [31:0] lut_xy_idx;
    reg [7:0]  cur_x;
    reg [7:0]  cur_y;
    reg [1:0]  p_sub;
    reg [2:0]  feat_req_count;
    reg [2:0]  feat_store_count;
    reg [2:0]  feat_outstanding;
    // Output write loop counter. write_k = z*(C/32) + c_blk, 0..31.
    //   z     = write_k[4:3]  (0..3)
    //   c_blk = write_k[2:0]  (0..7)  ; parity write_k[0] selects src cblocks
    reg [4:0]  write_k;
    wire [4:0] write_k_next = write_k + 5'd1;

    reg [31:0] lut_batch_addr;
    reg [2:0]  lut_lane;
    reg [63:0]  lut_entry;

    // The LUT is x-fastest and every DDR beat contains eight adjacent x
    // entries. Traversal is y-fastest for Decoder output packing, so retain
    // one line for every {z,y}. x=0,8,16,... force a fill; the following seven
    // x positions reuse the cached line. 1024 entries cover z<=4,y<=256.
    (* ram_style = "block" *) reg [511:0] lut_line_cache [0:1023];
    reg [511:0] lut_cache_q;
    reg [9:0]   lut_cache_index_r;
    wire [9:0]  lut_cache_index_w = {p_sub, cur_y};
    wire        lut_cache_compatible = (bev_x == 8'd200) && (bev_z <= 8'd4);
    wire        lut_cache_hit_w = lut_cache_compatible && (cur_x[2:0] != 3'b000);

    // Timing pre-computes. These values are static during one engine run,
    // so do not rebuild them on every LUT read/write address cycle.
    reg [31:0] plane_size_r;
    reg [31:0] plane_size_x2_r;
    reg [31:0] plane_size_x3_r;
    // Output-layout stride: bev_x*bev_y*32 = per-unit stride of (z*(C/32)+c_blk).
    // Computed once at engine start (single multiply, off the per-beat path).
    reg [31:0] zc_stride_r;
    // Running output byte address for the current write; walks by zc_stride_r.
    reg [31:0] wr_walk;

    reg signed [15:0] cur_cam_id;
    reg [15:0] cur_u;
    reg [15:0] cur_v;
    reg        entry_valid;

    // cam_id is consumed as cur_cam_id[7:0], img_h is 12 bits and cur_v is
    // 16 bits, so the exact row index needs only 21 bits. Keeping it narrow
    // prevents Vivado from implementing the following multiply as a 32x32
    // two-DSP cascade.
    reg [20:0] cam_row_idx;
    reg [31:0] pixel_idx;
    reg [31:0] pixel_mul_lo;
    reg [31:0] pixel_mul_hi;
    reg [31:0] feat_base_addr_r;
    // qbufN[y_parity][z]: N = source cblock (16 ch), y_parity 0=even 1=odd,
    // z = p_sub 0..3. A full (even,odd) y-pair is held before writing.
    reg [127:0] qbuf0 [0:1][0:3];
    reg [127:0] qbuf1 [0:1][0:3];
    reg [127:0] qbuf2 [0:1][0:3];
    reg [127:0] qbuf3 [0:1][0:3];

    wire [31:0] lut_z_offset_w = (p_sub == 2'd0) ? 32'd0 :
                                    (p_sub == 2'd1) ? plane_size_r :
                                    (p_sub == 2'd2) ? plane_size_x2_r :
                                                       plane_size_x3_r;
    wire [31:0] cur_lut_idx_w  = lut_xy_idx + lut_z_offset_w;

    assign rd_data_ready = (state == S_WAIT_LUT) || (state == S_WAIT_FEAT);

    wire quant_in_valid;
    wire [15:0] quant_out_valid;
    wire signed [7:0] qdst [0:15];

    genvar qi;
    generate
        for (qi = 0; qi < 16; qi = qi + 1) begin : quant_gen
            fp32_int8_quant U_QUANT (
                .clk       (clk),
                .rst_n     (rst_n_int),
                .in_valid  (quant_in_valid),
                .src       (rd_data[qi*32 +: 32]),
                .out_valid (quant_out_valid[qi]),
                .dst       (qdst[qi])
            );
        end
    endgenerate

    function [63:0] get_lut_lane;
        input [511:0] data;
        input [2:0] lane;
        begin
            case (lane)
                3'd0: get_lut_lane = data[ 63:  0];
                3'd1: get_lut_lane = data[127: 64];
                3'd2: get_lut_lane = data[191:128];
                3'd3: get_lut_lane = data[255:192];
                3'd4: get_lut_lane = data[319:256];
                3'd5: get_lut_lane = data[383:320];
                3'd6: get_lut_lane = data[447:384];
                3'd7: get_lut_lane = data[511:448];
                default: get_lut_lane = 64'd0;
            endcase
        end
    endfunction

    function signed [15:0] get_lut_cam;
        input [511:0] data;
        input [2:0] lane;
        reg [63:0] entry;
        begin
            entry = get_lut_lane(data, lane);
            get_lut_cam = entry[15:0];
        end
    endfunction

    function [127:0] pack_quant16;
        input signed [7:0] q0;
        input signed [7:0] q1;
        input signed [7:0] q2;
        input signed [7:0] q3;
        input signed [7:0] q4;
        input signed [7:0] q5;
        input signed [7:0] q6;
        input signed [7:0] q7;
        input signed [7:0] q8;
        input signed [7:0] q9;
        input signed [7:0] q10;
        input signed [7:0] q11;
        input signed [7:0] q12;
        input signed [7:0] q13;
        input signed [7:0] q14;
        input signed [7:0] q15;
        begin
            pack_quant16 = {q15, q14, q13, q12, q11, q10, q9, q8,
                            q7,  q6,  q5,  q4,  q3,  q2,  q1, q0};
        end
    endfunction

    function [127:0] qbuf_read;
        input [1:0] blk;
        input       par;   // y parity: 0 = even y, 1 = odd y
        input [1:0] sub;   // z (p_sub)
        begin
            case (blk)
                2'd0: qbuf_read = qbuf0[par][sub];
                2'd1: qbuf_read = qbuf1[par][sub];
                2'd2: qbuf_read = qbuf2[par][sub];
                2'd3: qbuf_read = qbuf3[par][sub];
                default: qbuf_read = 128'd0;
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int)
            state <= S_IDLE;
        else
            state <= next_state;
    end

    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE:        if (engine_start) next_state = S_PREP_P;
            S_PREP_P:      next_state = lut_cache_hit_w ? S_CACHE_HIT : S_RD_LUT;
            S_CACHE_HIT:   next_state = S_PARSE_LUT;
            S_RD_LUT:      if (rd_grant) next_state = S_WAIT_LUT;
            S_WAIT_LUT:    if (rd_data_valid) next_state = S_PARSE_LUT;
            S_PARSE_LUT:   next_state = entry_valid ? S_CALC_FEAT0 : S_NEXT_P;
            S_CALC_FEAT0:  next_state = S_CALC_FEAT1;
            S_CALC_FEAT1:  next_state = S_CALC_FEAT2;
            S_CALC_FEAT2:  next_state = S_CALC_FEAT3;
            S_CALC_FEAT3:  next_state = S_CALC_FEAT4;
            S_CALC_FEAT4:  next_state = S_RD_FEAT;
            S_RD_FEAT:     next_state = S_WAIT_FEAT;
            S_WAIT_FEAT:   if (feat_store_count == 3'd4) next_state = S_NEXT_P;
            // Even y is the first of an (even,odd) pair: hold it and advance
            // without writing. Odd y completes the pair -> flush the 32 beats.
            S_NEXT_P:      next_state = (p_sub == 2'd3) ?
                                           (cur_y[0] ? S_PREP_WRITE_E : S_ADV_GROUP) :
                                           S_PREP_P;
            S_PREP_WRITE_E: next_state = S_WR_OUT;
            S_WR_OUT:      if (wr_grant && write_k == 5'd31)
                               next_state = S_ADV_GROUP;
            S_ADV_GROUP:   next_state = (p_group_base + 32'd4 >= lut_size) ? S_DONE : S_PREP_P;
            S_DONE:        next_state = S_IDLE;
            default:       next_state = S_IDLE;
        endcase
    end

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            p_group_base <= 32'd0;
            xy_idx <= 32'd0;
            lut_xy_idx <= 32'd0;
            cur_x <= 8'd0;
            cur_y <= 8'd0;
            p_sub <= 2'd0;
            feat_req_count <= 3'd0;
            feat_store_count <= 3'd0;
            feat_outstanding <= 3'd0;
            write_k <= 5'd0;
            wr_addr <= 32'd0;
            wr_data <= 512'd0;
            plane_size_r <= 32'd0;
            plane_size_x2_r <= 32'd0;
            plane_size_x3_r <= 32'd0;
            zc_stride_r <= 32'd0;
            wr_walk <= 32'd0;
        end else begin
            if (state == S_IDLE && engine_start) begin
                p_group_base <= 32'd0;
                xy_idx <= 32'd0;
                lut_xy_idx <= 32'd0;
                cur_x <= 8'd0;
                cur_y <= 8'd0;
                p_sub <= 2'd0;
                feat_req_count <= 3'd0;
                feat_store_count <= 3'd0;
                feat_outstanding <= 3'd0;
                write_k <= 5'd0;
                wr_walk <= 32'd0;
                // Static geometry/stride registers for this run.
                plane_size_r <= {24'd0, bev_x} * {24'd0, bev_y};
                plane_size_x2_r <= ({24'd0, bev_x} * {24'd0, bev_y}) << 1;
                plane_size_x3_r <= (({24'd0, bev_x} * {24'd0, bev_y}) << 1) +
                                   ({24'd0, bev_x} * {24'd0, bev_y});
                // Output-layout stride: bev_x*bev_y*32 (one (z*(C/32)+c_blk) unit).
                zc_stride_r <= ({24'd0, bev_x} * {24'd0, bev_y}) << 5;
            end else begin
                if (state == S_RD_FEAT) begin
                    feat_req_count <= 3'd0;
                    feat_store_count <= 3'd0;
                    feat_outstanding <= 3'd0;
                end else if (state == S_WAIT_FEAT) begin
                    if (rd_grant)
                        feat_req_count <= feat_req_count + 3'd1;
                    if (quant_out_valid[0])
                        feat_store_count <= feat_store_count + 3'd1;
                    case ({rd_grant, rd_data_valid})
                        2'b10: feat_outstanding <= feat_outstanding + 3'd1;
                        2'b01: feat_outstanding <= feat_outstanding - 3'd1;
                        default: feat_outstanding <= feat_outstanding;
                    endcase
                end

                if (state == S_NEXT_P && p_sub != 2'd3)
                    p_sub <= p_sub + 2'd1;
                else if (state == S_ADV_GROUP)
                    p_sub <= 2'd0;

                // Entering the write phase for a completed (even,odd) y-pair.
                // First beat address = base + (x*bev_y + y_even)*32.
                //   y_even = cur_y-1, and (x*bev_y + y_even) = xy_idx - 1.
                // (xy_idx << 5) is a wire shift, no DSP multiply.
                if (state == S_NEXT_P && p_sub == 2'd3 && cur_y[0]) begin
                    write_k <= 5'd0;
                    wr_walk <= feat3d_wr_addr + ((xy_idx - 32'd1) << 5);
                end

                if (state == S_PREP_WRITE_E) begin
                    wr_addr <= wr_walk;
                    // write_k = z*(C/32) + c_blk : z=write_k[4:3], c_blk=write_k[2:0].
                    // Beat = { y_odd cblkB, y_odd cblkA, y_even cblkB, y_even cblkA }.
                    // Even c_blk uses source cblocks (0,1). Splitting even/odd
                    // states keeps write_k[0] off the 512-bit wr_data mux select.
                    wr_data <= {qbuf_read(2'd1, 1'b1, write_k[4:3]),
                                qbuf_read(2'd0, 1'b1, write_k[4:3]),
                                qbuf_read(2'd1, 1'b0, write_k[4:3]),
                                qbuf_read(2'd0, 1'b0, write_k[4:3])};
                end

                if (state == S_WR_OUT && wr_grant && write_k != 5'd31) begin
                    // Advance and prepare the following beat immediately. With
                    // no DDR backpressure wr_req stays asserted and one beat is
                    // accepted every core cycle.
                    write_k <= write_k_next;
                    wr_walk <= wr_walk + zc_stride_r;
                    wr_addr <= wr_walk + zc_stride_r;
                    if (write_k_next[0] == 1'b0) begin
                        wr_data <= {qbuf_read(2'd1, 1'b1, write_k_next[4:3]),
                                    qbuf_read(2'd0, 1'b1, write_k_next[4:3]),
                                    qbuf_read(2'd1, 1'b0, write_k_next[4:3]),
                                    qbuf_read(2'd0, 1'b0, write_k_next[4:3])};
                    end else begin
                        wr_data <= {qbuf_read(2'd3, 1'b1, write_k_next[4:3]),
                                    qbuf_read(2'd2, 1'b1, write_k_next[4:3]),
                                    qbuf_read(2'd3, 1'b0, write_k_next[4:3]),
                                    qbuf_read(2'd2, 1'b0, write_k_next[4:3])};
                    end
                end

                if (state == S_ADV_GROUP) begin
                    // One group = the four z values of one (x,y); groups advance
                    // with y faster than x. y-fastest is required so consecutive
                    // groups form (even,odd) y-pairs for the 64-byte writes, and
                    // it keeps p_group_base (voxel counter) stepping by bev_z=4.
                    p_group_base <= p_group_base + 32'd4;
                    xy_idx <= xy_idx + 32'd1;
                    if (cur_y + 8'd1 >= bev_y) begin
                        // Next group is (x+1, y=0), so LUT xy index is x+1.
                        lut_xy_idx <= {24'd0, cur_x} + 32'd1;
                        cur_y <= 8'd0;
                        cur_x <= cur_x + 8'd1;
                    end else begin
                        // Next group is (same x, y+1), so y*bev_x+x advances by bev_x.
                        lut_xy_idx <= lut_xy_idx + {24'd0, bev_x};
                        cur_y <= cur_y + 8'd1;
                    end
                end
            end
        end
    end

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            lut_batch_addr <= 32'd0;
            lut_lane <= 3'd0;
        end else if (state == S_PREP_P) begin
            lut_batch_addr <= lut_base_addr + ((cur_lut_idx_w >> 3) << 6);
            lut_lane <= cur_lut_idx_w[2:0];
        end
    end

    // Keep both cache ports in reset-free synchronous blocks. Vivado 2018.3
    // otherwise treats the entire 512 Kbit array as asynchronously reset and
    // cannot infer BRAM from it.
    always @(posedge clk) begin
        if (state == S_PREP_P) begin
            lut_cache_index_r <= lut_cache_index_w;
            lut_cache_q <= lut_line_cache[lut_cache_index_w];
        end
    end

    always @(posedge clk) begin
        if (state == S_WAIT_LUT && rd_data_valid && lut_cache_compatible)
            lut_line_cache[lut_cache_index_r] <= rd_data;
    end

    wire feat_request_enable = (state == S_WAIT_FEAT) &&
                               (feat_req_count < 3'd4) &&
                               (feat_outstanding < FEAT_MAX_OUTSTANDING);
    assign rd_req = (state == S_RD_LUT) || feat_request_enable;
    assign rd_addr = (state == S_RD_LUT) ? lut_batch_addr :
                     (feat_base_addr_r + ({29'd0, feat_req_count} << 6));

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            lut_entry <= 64'd0;
            cur_cam_id <= -16'sd1;
            cur_u <= 16'd0;
            cur_v <= 16'd0;
            entry_valid <= 1'b0;
        end else begin
            if (state == S_WAIT_LUT && rd_data_valid) begin
                lut_entry <= get_lut_lane(rd_data, lut_lane);
                entry_valid <= (get_lut_cam(rd_data, lut_lane) >= 0);
            end else if (state == S_CACHE_HIT) begin
                lut_entry <= get_lut_lane(lut_cache_q, lut_lane);
                entry_valid <= (get_lut_cam(lut_cache_q, lut_lane) >= 0);
            end
            if (state == S_PARSE_LUT) begin
                cur_cam_id <= lut_entry[15:0];
                cur_u      <= lut_entry[31:16];
                cur_v      <= lut_entry[47:32];
            end
        end
    end

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            cam_row_idx <= 21'd0;
            pixel_idx <= 32'd0;
            pixel_mul_lo <= 32'd0;
            pixel_mul_hi <= 32'd0;
            feat_base_addr_r <= 32'd0;
        end else begin
            if (state == S_CALC_FEAT0)
                // cur_cam_id is intentionally consumed as [7:0], matching the
                // original RTL. The full legal result width is 21 bits:
                // 255*4095 + 65535 = 1,109,760 < 2^21.
                cam_row_idx <= ({13'd0, cur_cam_id[7:0]} * {9'd0, img_h}) +
                               {5'd0, cur_v};

            if (state == S_CALC_FEAT1) begin
                // Split cam_row_idx * img_w into two small multiplies instead
                // of allowing Vivado to infer a 32x32/two-DSP cascade on the
                // path reported as cam_row_idx0 -> pixel_idx_reg[*].
                pixel_mul_lo <= cam_row_idx[15:0]  * img_w;
                pixel_mul_hi <= {11'd0, cam_row_idx[20:16]} * img_w;
            end

            if (state == S_CALC_FEAT2)
                // Recombine the partial products modulo 2^32, which is exactly
                // what the original 32-bit pixel_idx assignment preserved.
                pixel_idx <= pixel_mul_lo + (pixel_mul_hi << 16);

            if (state == S_CALC_FEAT3)
                // Add horizontal pixel coordinate in its own carry-chain stage.
                pixel_idx <= pixel_idx + {16'd0, cur_u};

            if (state == S_CALC_FEAT4)
                // Convert pixel index to byte address. The shift is wiring; the
                // only arithmetic left here is one 32-bit base-address add.
                feat_base_addr_r <= feat2d_base_addr + (pixel_idx << 8);
        end
    end

    assign quant_in_valid = (state == S_WAIT_FEAT) && rd_data_valid;

    // Quantized values land in qbufN[y_parity][z]. y_parity = cur_y[0] of the
    // group being processed; both parities of a pair are kept until the write.
    wire [127:0] qpack_w = pack_quant16(qdst[0], qdst[1], qdst[2], qdst[3],
                                        qdst[4], qdst[5], qdst[6], qdst[7],
                                        qdst[8], qdst[9], qdst[10], qdst[11],
                                        qdst[12], qdst[13], qdst[14], qdst[15]);
    wire [127:0] qstore_w = entry_valid ? qpack_w : 128'd0;

    integer zi, pj;
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            for (pj = 0; pj < 2; pj = pj + 1)
                for (zi = 0; zi < 4; zi = zi + 1) begin
                    qbuf0[pj][zi] <= 128'd0;
                    qbuf1[pj][zi] <= 128'd0;
                    qbuf2[pj][zi] <= 128'd0;
                    qbuf3[pj][zi] <= 128'd0;
                end
        end else if (state == S_PARSE_LUT && !entry_valid) begin
            qbuf0[cur_y[0]][p_sub] <= 128'd0;
            qbuf1[cur_y[0]][p_sub] <= 128'd0;
            qbuf2[cur_y[0]][p_sub] <= 128'd0;
            qbuf3[cur_y[0]][p_sub] <= 128'd0;
        end else if (state == S_WAIT_FEAT && quant_out_valid[0]) begin
            case (feat_store_count)
                3'd0: qbuf0[cur_y[0]][p_sub] <= qstore_w;
                3'd1: qbuf1[cur_y[0]][p_sub] <= qstore_w;
                3'd2: qbuf2[cur_y[0]][p_sub] <= qstore_w;
                3'd3: qbuf3[cur_y[0]][p_sub] <= qstore_w;
            endcase
        end
    end

    assign wr_req = (state == S_WR_OUT);

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int)
            engine_done <= 1'b0;
        else
            engine_done <= (state == S_DONE);
    end

    // Diagnostic counters are cleared at each accepted start and remain stable
    // after engine_done so the slower register-access clock can read them safely.
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            perf_lut_wait    <= 32'd0;
            perf_feat_wait   <= 32'd0;
            perf_write_wait  <= 32'd0;
            perf_valid_voxel <= 32'd0;
            perf_lut_reads   <= 32'd0;
            perf_feat_reads  <= 32'd0;
            perf_writes      <= 32'd0;
            perf_pipe_stall  <= 32'd0;
        end else if (state == S_IDLE && engine_start) begin
            perf_lut_wait    <= 32'd0;
            perf_feat_wait   <= 32'd0;
            perf_write_wait  <= 32'd0;
            perf_valid_voxel <= 32'd0;
            perf_lut_reads   <= 32'd0;
            perf_feat_reads  <= 32'd0;
            perf_writes      <= 32'd0;
            perf_pipe_stall  <= 32'd0;
        end else begin
            if (state == S_WAIT_LUT && !rd_data_valid) begin
                perf_lut_wait   <= perf_lut_wait + 32'd1;
                perf_pipe_stall <= perf_pipe_stall + 32'd1;
            end
            if (state == S_WAIT_FEAT && !rd_data_valid) begin
                perf_feat_wait  <= perf_feat_wait + 32'd1;
                perf_pipe_stall <= perf_pipe_stall + 32'd1;
            end
            if (state == S_WR_OUT && !wr_grant) begin
                perf_write_wait <= perf_write_wait + 32'd1;
                perf_pipe_stall <= perf_pipe_stall + 32'd1;
            end
            if (state == S_PARSE_LUT && entry_valid)
                perf_valid_voxel <= perf_valid_voxel + 32'd1;
            if (state == S_RD_LUT && rd_grant)
                perf_lut_reads <= perf_lut_reads + 32'd1;
            if (state == S_WAIT_FEAT && rd_grant)
                perf_feat_reads <= perf_feat_reads + 32'd1;
            if (state == S_WR_OUT && wr_grant)
                perf_writes <= perf_writes + 32'd1;
        end
    end

endmodule
