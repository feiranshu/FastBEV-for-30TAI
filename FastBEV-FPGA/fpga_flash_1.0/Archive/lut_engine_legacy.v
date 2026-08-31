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
//   Final Decoder INT8 input layout, not an intermediate BEV buffer:
//     [C_outer=16][P=160000][c16=16]
//     P = ((x * bev_y + y) * bev_z + z)
//     byte_addr = feat3d_wr_addr + ((C_outer * lut_size + P) * 16)
//
//   One 512-bit write packs 4 consecutive P values for the same C_outer:
//     wr_data[127:0]   = P+0, 16 int8 values
//     wr_data[255:128] = P+1, 16 int8 values
//     wr_data[383:256] = P+2, 16 int8 values
//     wr_data[511:384] = P+3, 16 int8 values
//
// Replication:
//   Source has 64 channels = 4 channel blocks. Decoder expects 256 channels =
//   16 channel blocks. The engine writes each source block into four repeated
//   frame groups:
//     C_outer = repeat_id * 4 + src_cblock, repeat_id = 0..3.
//==============================================================================
`timescale 1ns/1ps

module lut_engine(
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

    output reg   [31:0]   rd_addr,
    output reg            rd_req,
    input                 rd_grant,
    input        [511:0]  rd_data,
    input                 rd_data_valid,
    output                rd_data_ready,

    output reg   [31:0]   wr_addr,
    output reg   [511:0]  wr_data,
    output reg            wr_req,
    input                 wr_grant,

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
    localparam S_RD_FEAT       = 5'd7;
    localparam S_WAIT_FEAT     = 5'd8;
    localparam S_START_QUANT   = 5'd9;
    localparam S_WAIT_QUANT    = 5'd10;
    localparam S_STORE_QUANT   = 5'd11;
    localparam S_NEXT_CBLOCK   = 5'd12;
    localparam S_NEXT_P        = 5'd13;
    localparam S_PREP_WRITE    = 5'd14;
    localparam S_WR_OUT        = 5'd15;
    localparam S_NEXT_WRITE    = 5'd16;
    localparam S_ADV_GROUP     = 5'd17;
    localparam S_DONE          = 5'd18;

    reg [4:0] state, next_state;

    (* MAX_FANOUT = 256 *)
    reg rst_n_int;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rst_n_int <= 1'b0;
        else        rst_n_int <= 1'b1;
    end

    reg [31:0] p_group_base;
    reg [31:0] xy_idx;
    reg [7:0]  cur_x;
    reg [7:0]  cur_y;
    reg [1:0]  p_sub;
    reg [1:0]  cblock;
    reg [1:0]  write_cblock;
    reg [1:0]  repeat_id;

    reg [31:0] p_abs;
    reg [31:0] lut_idx;
    reg [31:0] lut_batch_addr;
    reg [2:0]  lut_lane;
    reg [511:0] lut_batch;
    reg [63:0]  lut_entry;

    reg signed [15:0] cur_cam_id;
    reg [15:0] cur_u;
    reg [15:0] cur_v;
    reg        entry_valid;

    reg [31:0] cam_row_idx;
    reg [31:0] pixel_idx;
    reg [31:0] feat_base_addr_r;
    reg [511:0] feat_beat;

    reg [127:0] qbuf0 [0:3];
    reg [127:0] qbuf1 [0:3];
    reg [127:0] qbuf2 [0:3];
    reg [127:0] qbuf3 [0:3];

    wire [31:0] plane_size = bev_x * bev_y;
    wire [31:0] cur_z_ext  = {30'd0, p_sub};
    wire [31:0] cur_x_ext  = {24'd0, cur_x};
    wire [31:0] cur_y_ext  = {24'd0, cur_y};
    wire [31:0] xy_lut_idx = cur_y_ext * bev_x + cur_x_ext;
    wire [31:0] cur_lut_idx_w = cur_z_ext * plane_size + xy_lut_idx;
    wire [31:0] out_c_outer_idx = ({28'd0, repeat_id} << 2) + {30'd0, write_cblock};
    wire [31:0] out_beat_addr = feat3d_wr_addr + (((out_c_outer_idx * lut_size) + p_group_base) << 4);

    assign rd_data_ready = (state == S_WAIT_LUT) || (state == S_WAIT_FEAT);

    reg quant_in_valid;
    wire [15:0] quant_out_valid;
    wire signed [7:0] qdst [0:15];

    genvar qi;
    generate
        for (qi = 0; qi < 16; qi = qi + 1) begin : quant_gen
            fp32_int8_quant U_QUANT (
                .clk       (clk),
                .rst_n     (rst_n_int),
                .in_valid  (quant_in_valid),
                .src       (feat_beat[qi*32 +: 32]),
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
        input [1:0] sub;
        begin
            case (blk)
                2'd0: qbuf_read = qbuf0[sub];
                2'd1: qbuf_read = qbuf1[sub];
                2'd2: qbuf_read = qbuf2[sub];
                2'd3: qbuf_read = qbuf3[sub];
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
            S_PREP_P:      next_state = S_RD_LUT;
            S_RD_LUT:      if (rd_grant) next_state = S_WAIT_LUT;
            S_WAIT_LUT:    if (rd_data_valid) next_state = S_PARSE_LUT;
            S_PARSE_LUT:   next_state = entry_valid ? S_CALC_FEAT0 : S_STORE_QUANT;
            S_CALC_FEAT0:  next_state = S_CALC_FEAT1;
            S_CALC_FEAT1:  next_state = S_RD_FEAT;
            S_RD_FEAT:     if (rd_grant) next_state = S_WAIT_FEAT;
            S_WAIT_FEAT:   if (rd_data_valid) next_state = S_START_QUANT;
            S_START_QUANT: next_state = S_WAIT_QUANT;
            S_WAIT_QUANT:  if (quant_out_valid[0]) next_state = S_STORE_QUANT;
            S_STORE_QUANT: next_state = S_NEXT_CBLOCK;
            S_NEXT_CBLOCK: next_state = (cblock == 2'd3) ? S_NEXT_P :
                                           (entry_valid ? S_CALC_FEAT1 : S_STORE_QUANT);
            S_NEXT_P:      next_state = (p_sub == 2'd3) ? S_PREP_WRITE : S_PREP_P;
            S_PREP_WRITE:  next_state = S_WR_OUT;
            S_WR_OUT:      if (wr_grant) next_state = S_NEXT_WRITE;
            S_NEXT_WRITE:  next_state = (repeat_id == 2'd3 && write_cblock == 2'd3) ? S_ADV_GROUP : S_PREP_WRITE;
            S_ADV_GROUP:   next_state = (p_group_base + 32'd4 >= lut_size) ? S_DONE : S_PREP_P;
            S_DONE:        next_state = S_IDLE;
            default:       next_state = S_IDLE;
        endcase
    end

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            p_group_base <= 32'd0;
            xy_idx <= 32'd0;
            cur_x <= 8'd0;
            cur_y <= 8'd0;
            p_sub <= 2'd0;
            cblock <= 2'd0;
            write_cblock <= 2'd0;
            repeat_id <= 2'd0;
        end else begin
            if (state == S_IDLE && engine_start) begin
                p_group_base <= 32'd0;
                xy_idx <= 32'd0;
                cur_x <= 8'd0;
                cur_y <= 8'd0;
                p_sub <= 2'd0;
                cblock <= 2'd0;
                write_cblock <= 2'd0;
                repeat_id <= 2'd0;
            end else begin
                if (state == S_PREP_P)
                    cblock <= 2'd0;
                else if (state == S_NEXT_CBLOCK && cblock != 2'd3)
                    cblock <= cblock + 2'd1;

                if (state == S_NEXT_P && p_sub != 2'd3)
                    p_sub <= p_sub + 2'd1;
                else if (state == S_ADV_GROUP)
                    p_sub <= 2'd0;

                if (state == S_PREP_WRITE) begin
                    wr_addr <= out_beat_addr;
                    wr_data <= {qbuf_read(write_cblock, 2'd3),
                                qbuf_read(write_cblock, 2'd2),
                                qbuf_read(write_cblock, 2'd1),
                                qbuf_read(write_cblock, 2'd0)};
                end

                if (state == S_NEXT_WRITE) begin
                    if (write_cblock == 2'd3) begin
                        write_cblock <= 2'd0;
                        repeat_id <= repeat_id + 2'd1;
                    end else begin
                        write_cblock <= write_cblock + 2'd1;
                    end
                end

                if (state == S_ADV_GROUP) begin
                    // Output P order is P = ((x * bev_y + y) * bev_z + z).
                    // One group contains four z values for the same (x,y), so
                    // p_group_base advances by bev_z (=4). To keep P contiguous,
                    // y must advance faster than x.
                    p_group_base <= p_group_base + 32'd4;
                    xy_idx <= xy_idx + 32'd1;
                    repeat_id <= 2'd0;
                    write_cblock <= 2'd0;
                    if (cur_y + 8'd1 >= bev_y) begin
                        cur_y <= 8'd0;
                        cur_x <= cur_x + 8'd1;
                    end else begin
                        cur_y <= cur_y + 8'd1;
                    end
                end
            end
        end
    end

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            p_abs <= 32'd0;
            lut_idx <= 32'd0;
            lut_batch_addr <= 32'd0;
            lut_lane <= 3'd0;
        end else if (state == S_PREP_P) begin
            p_abs <= p_group_base + {30'd0, p_sub};
            lut_idx <= cur_lut_idx_w;
            lut_batch_addr <= lut_base_addr + ((cur_lut_idx_w >> 3) << 6);
            lut_lane <= cur_lut_idx_w[2:0];
        end
    end

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            rd_addr <= 32'd0;
            rd_req <= 1'b0;
        end else if (rd_grant) begin
            rd_req <= 1'b0;
        end else begin
            case (state)
                S_RD_LUT: begin
                    rd_addr <= lut_batch_addr;
                    rd_req <= 1'b1;
                end
                S_RD_FEAT: begin
                    rd_addr <= feat_base_addr_r + {cblock, 6'b0};
                    rd_req <= entry_valid;
                end
                default: rd_req <= 1'b0;
            endcase
        end
    end

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            lut_batch <= 512'd0;
            lut_entry <= 64'd0;
            cur_cam_id <= -16'sd1;
            cur_u <= 16'd0;
            cur_v <= 16'd0;
            entry_valid <= 1'b0;
        end else begin
            if (state == S_WAIT_LUT && rd_data_valid) begin
                lut_batch <= rd_data;
                lut_entry <= get_lut_lane(rd_data, lut_lane);
                entry_valid <= (get_lut_cam(rd_data, lut_lane) >= 0);
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
            cam_row_idx <= 32'd0;
            pixel_idx <= 32'd0;
            feat_base_addr_r <= 32'd0;
        end else begin
            if (state == S_CALC_FEAT0)
                cam_row_idx <= cur_cam_id[7:0] * img_h + cur_v;
            if (state == S_CALC_FEAT1) begin
                pixel_idx <= cam_row_idx * img_w + cur_u;
                feat_base_addr_r <= feat2d_base_addr + ((cam_row_idx * img_w + cur_u) << 8);
            end
        end
    end

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int)
            feat_beat <= 512'd0;
        else if (state == S_WAIT_FEAT && rd_data_valid)
            feat_beat <= rd_data;
        else if (state == S_PARSE_LUT && !entry_valid)
            feat_beat <= 512'd0;
    end

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int)
            quant_in_valid <= 1'b0;
        else
            quant_in_valid <= (state == S_START_QUANT);
    end

    integer zi;
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            for (zi = 0; zi < 4; zi = zi + 1) begin
                qbuf0[zi] <= 128'd0;
                qbuf1[zi] <= 128'd0;
                qbuf2[zi] <= 128'd0;
                qbuf3[zi] <= 128'd0;
            end
        end else if (state == S_STORE_QUANT) begin
            case (cblock)
                2'd0: qbuf0[p_sub] <= entry_valid ?
                    pack_quant16(qdst[0], qdst[1], qdst[2], qdst[3],
                                 qdst[4], qdst[5], qdst[6], qdst[7],
                                 qdst[8], qdst[9], qdst[10], qdst[11],
                                 qdst[12], qdst[13], qdst[14], qdst[15]) : 128'd0;
                2'd1: qbuf1[p_sub] <= entry_valid ?
                    pack_quant16(qdst[0], qdst[1], qdst[2], qdst[3],
                                 qdst[4], qdst[5], qdst[6], qdst[7],
                                 qdst[8], qdst[9], qdst[10], qdst[11],
                                 qdst[12], qdst[13], qdst[14], qdst[15]) : 128'd0;
                2'd2: qbuf2[p_sub] <= entry_valid ?
                    pack_quant16(qdst[0], qdst[1], qdst[2], qdst[3],
                                 qdst[4], qdst[5], qdst[6], qdst[7],
                                 qdst[8], qdst[9], qdst[10], qdst[11],
                                 qdst[12], qdst[13], qdst[14], qdst[15]) : 128'd0;
                2'd3: qbuf3[p_sub] <= entry_valid ?
                    pack_quant16(qdst[0], qdst[1], qdst[2], qdst[3],
                                 qdst[4], qdst[5], qdst[6], qdst[7],
                                 qdst[8], qdst[9], qdst[10], qdst[11],
                                 qdst[12], qdst[13], qdst[14], qdst[15]) : 128'd0;
            endcase
        end
    end

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int)
            wr_req <= 1'b0;
        else if (wr_grant)
            wr_req <= 1'b0;
        else if (state == S_WR_OUT)
            wr_req <= 1'b1;
        else
            wr_req <= 1'b0;
    end

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int)
            engine_done <= 1'b0;
        else
            engine_done <= (state == S_DONE);
    end

endmodule
