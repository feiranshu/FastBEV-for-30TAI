`timescale 1ns/1ps
//==============================================================================
// File Name     : lut_engine_fp32.v
// Project       : Fast-BEV Part2 FPGA Accelerator
// Description   : LUT gather and direct FP16 Decoder Conv-input writer.
//
// Data path:
//   Vehicle Part1 FP32 NHWC [camera][v][u][channel] = [6][120][160][64]
//     -> vehicle LUT gather into BEV voxels ordered [z][y][x][64]
//     -> FP32 -> TF32 RNE -> IEEE FP16 RNE
//     -> fold Z into channel blocks and write [1][cblk16][x][y][c16]
//
// The physical output dimension is:
//   [1][16][200][200][16] FP16.
// Global cblk16 = z*4 + source_cblk16.  One 512-bit write packs the same
// cblk16 for adjacent (y_even,y_odd) positions at one X.
//
// Vehicle production contract:
//   FEAT2D=[6,120,160,64], BEV_X=200, BEV_Y=200, BEV_Z=4,
//   channels=64.
//   Output: [1,16,200,200,16] FP16 = 20,480,000 bytes.
//==============================================================================

module lut_engine(

    // --- Control ---
    input                       engine_start     ,
    output reg                  engine_done      ,

    // --- Configuration (stable while the engine is active) ---
    input         [   31 : 0]   lut_base_addr    ,
    input         [   31 : 0]   lut_size         ,
    input         [   31 : 0]   feat2d_base_addr ,
    input         [   31 : 0]   feat3d_wr_addr   ,
    input         [   11 : 0]   img_w            ,
    input         [   11 : 0]   img_h            ,
    input         [    7 : 0]   cameras          ,
    input         [    7 : 0]   bev_x            ,
    input         [    7 : 0]   bev_y            ,
    input         [    7 : 0]   bev_z            ,

    // --- PLDDR Read Request ---
    output reg    [   31 : 0]   rd_addr          ,
    output reg                  rd_req           ,
    input                       rd_grant         ,

    // --- PLDDR Read Data, 512 bits ---
    input         [  511 : 0]   rd_data          ,
    input                       rd_data_valid    ,
    output                      rd_data_ready    ,

    // --- PLDDR Write Request, 512 bits ---
    output reg    [   31 : 0]   wr_addr          ,
    output reg    [  511 : 0]   wr_data          ,
    output reg                  wr_req           ,
    input                       wr_grant         ,

    input                       clk              ,
    input                       rst_n
);

    // ===================== FP32 -> TF32 -> FP16 =====================
    // The Decoder prefix first rounds FP32 to TF32 precision, then converts
    // to FP16. Keep this two-stage behavior explicit to avoid edge-case
    // differences around FP16 subnormal rounding boundaries.
    function [31:0] fp32_to_tf32_rne;
        input [31:0] value;
        reg          sign;
        reg  [7:0]   exponent;
        reg  [22:0]  fraction;
        reg  [10:0]  rounded_fraction;
        reg          round_up;
        begin
            sign     = value[31];
            exponent = value[30:23];
            fraction = value[22:0];

            if (exponent == 8'hFF) begin
                if (fraction == 23'd0)
                    fp32_to_tf32_rne = {sign, 8'hFF, 23'd0};
                else
                    fp32_to_tf32_rne = {sign, 8'hFF, 1'b1, 22'd0};
            end
            else begin
                round_up = fraction[12] &&
                           ((|fraction[11:0]) || fraction[13]);
                rounded_fraction = {1'b0, fraction[22:13]} + round_up;

                if (rounded_fraction[10]) begin
                    if (exponent == 8'hFE)
                        fp32_to_tf32_rne = {sign, 8'hFF, 23'd0};
                    else
                        fp32_to_tf32_rne = {
                            sign, exponent + 1'b1, 23'd0
                        };
                end
                else begin
                    fp32_to_tf32_rne = {
                        sign, exponent, rounded_fraction[9:0], 13'd0
                    };
                end
            end
        end
    endfunction

    function [15:0] tf32_to_fp16_rne;
        input [31:0] value;
        reg          sign;
        reg  [7:0]   exponent;
        reg  [9:0]   fraction;
        reg  [10:0]  significand;
        reg  [10:0]  shifted;
        reg  [10:0]  rounded;
        reg  [4:0]   half_exponent;
        reg          guard_bit;
        reg          sticky_bit;
        reg          round_up;
        integer      shift;
        integer      bit_index;
        begin
            sign     = value[31];
            exponent = value[30:23];
            fraction = value[22:13];
            significand = {1'b1, fraction};
            shifted     = 11'd0;
            rounded     = 11'd0;
            half_exponent = 5'd0;
            guard_bit   = 1'b0;
            sticky_bit  = 1'b0;
            round_up    = 1'b0;
            shift       = 0;

            if (exponent == 8'hFF) begin
                if (value[22:0] == 23'd0)
                    tf32_to_fp16_rne = {sign, 5'h1F, 10'd0};
                else
                    tf32_to_fp16_rne = {sign, 5'h1F, 10'h200};
            end
            else if (exponent > 8'd142) begin
                tf32_to_fp16_rne = {sign, 5'h1F, 10'd0};
            end
            else if (exponent >= 8'd113) begin
                // TF32 and FP16 both retain ten explicit mantissa bits.
                // In the handled range (113..142), modulo-32 subtraction
                // yields the exact binary16 exponent (1..30).
                half_exponent = exponent[4:0] - 5'd16;
                tf32_to_fp16_rne = {sign, half_exponent, fraction};
            end
            else if (exponent >= 8'd102) begin
                // FP16 subnormal. RNE is applied to the bits discarded by
                // the exponent-dependent right shift.
                shift = 113 - {24'd0, exponent};
                shifted = significand >> shift;
                guard_bit = significand[shift - 1];
                sticky_bit = 1'b0;
                for (bit_index = 0; bit_index < 11; bit_index = bit_index + 1) begin
                    if (bit_index < shift - 1)
                        sticky_bit = sticky_bit | significand[bit_index];
                end
                round_up = guard_bit && (sticky_bit || shifted[0]);
                rounded = shifted + round_up;

                if (rounded[10])
                    tf32_to_fp16_rne = {sign, 5'd1, 10'd0};
                else
                    tf32_to_fp16_rne = {sign, 5'd0, rounded[9:0]};
            end
            else begin
                tf32_to_fp16_rne = {sign, 15'd0};
            end
        end
    endfunction

    function [15:0] fp32_prefix_to_fp16;
        input [31:0] value;
        begin
            fp32_prefix_to_fp16 = tf32_to_fp16_rne(
                fp32_to_tf32_rne(value)
            );
        end
    endfunction

    // ===================== State Machine =====================
    localparam S_IDLE        = 4'd0;
    localparam S_RD_LUT      = 4'd1;
    localparam S_WAIT_LUT    = 4'd2;
    localparam S_PARSE_LUT   = 4'd3;
    localparam S_CALC_ADDR   = 4'd4;
    localparam S_CALC_ADDR2  = 4'd5;
    localparam S_RD_FEAT     = 4'd6;
    localparam S_WAIT_FEAT   = 4'd7;
    localparam S_CONVERT     = 4'd8;
    localparam S_VOXEL_DONE  = 4'd9;
    localparam S_WRITE_VOXEL = 4'd10;
    localparam S_DONE        = 4'd11;
    localparam S_STORE       = 4'd12;
    localparam S_PREP_WRITE  = 4'd13;
    localparam S_CONVERT_FP16 = 4'd14;

    reg  [3:0] state;
    reg  [3:0] next_state;

    (* MAX_FANOUT = 256 *)
    reg rst_n_int;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rst_n_int <= 1'b0;
        else        rst_n_int <= 1'b1;
    end

    // ===================== LUT and Voxel Tracking =====================
    reg  [31:0]  voxel_cnt;
    reg  [511:0] lut_batch;
    reg  [2:0]   lut_sub_idx;
    wire         need_new_batch;

    reg  signed [15:0] cur_cam_id;
    reg         [15:0] cur_u;
    reg         [15:0] cur_v;
    wire               lut_entry_valid;

    assign need_new_batch = (lut_sub_idx == 3'd0);
    assign lut_entry_valid = !cur_cam_id[15] &&
                             (cur_cam_id[14:8] == 7'd0) &&
                             (cur_cam_id[7:0] < cameras) &&
                             (cur_u < img_w) && (cur_v < img_h);

    wire [31:0] lut_rd_byte_addr;
    wire [63:0] cur_lut_raw;
    assign lut_rd_byte_addr = lut_base_addr + {voxel_cnt[28:3], 6'b0};
    assign cur_lut_raw = lut_batch[lut_sub_idx*64 +: 64];

    // ===================== Feature Address Pipeline =====================
    reg  [20:0] pipe_cam_v;
    wire [25:0] pixel_linear_idx_s2;
    reg  [25:0] pixel_idx_r;
    reg  [31:0] feat_rd_addr_reg;
    reg  [1:0]  feat_beat_cnt;
    reg  [511:0] feat_buffer;
    reg  [511:0] tf32_beat_r;
    reg  [255:0] converted_beat_r;

    assign pixel_linear_idx_s2 = pipe_cam_v * img_w + {10'd0, cur_u};

    // Sixteen lanes remain parallel to preserve four reads per pixel, but the
    // former nested FP32->TF32->FP16 cone is split by tf32_beat_r.
    wire [511:0] tf32_beat_w;
    wire [255:0] converted_beat;
    genvar lane;
    generate
        for (lane = 0; lane < 16; lane = lane + 1) begin : GEN_FP_CONVERT
            assign tf32_beat_w[lane*32 +: 32] =
                fp32_to_tf32_rne(feat_buffer[lane*32 +: 32]);
            assign converted_beat[lane*16 +: 16] =
                tf32_to_fp16_rne(tf32_beat_r[lane*32 +: 32]);
        end
    endgenerate

    // ===================== Row Pair FP16 Blocks =====================
    // LUT traversal is x-fastest.  Retain one even-Y row, then combine each
    // following odd-Y voxel with the stored value at the same X.
    // Vivado 2018.3 can crash while banking a 256-bit-wide inferred RAM.
    // Explicit 64-bit banks map directly to RAMB36E1 (256 deep x 64 wide).
    (* ram_style = "block" *) reg [63:0] row_b0_0 [0:255];
    (* ram_style = "block" *) reg [63:0] row_b0_1 [0:255];
    (* ram_style = "block" *) reg [63:0] row_b0_2 [0:255];
    (* ram_style = "block" *) reg [63:0] row_b0_3 [0:255];
    (* ram_style = "block" *) reg [63:0] row_b1_0 [0:255];
    (* ram_style = "block" *) reg [63:0] row_b1_1 [0:255];
    (* ram_style = "block" *) reg [63:0] row_b1_2 [0:255];
    (* ram_style = "block" *) reg [63:0] row_b1_3 [0:255];
    (* ram_style = "block" *) reg [63:0] row_b2_0 [0:255];
    (* ram_style = "block" *) reg [63:0] row_b2_1 [0:255];
    (* ram_style = "block" *) reg [63:0] row_b2_2 [0:255];
    (* ram_style = "block" *) reg [63:0] row_b2_3 [0:255];
    (* ram_style = "block" *) reg [63:0] row_b3_0 [0:255];
    (* ram_style = "block" *) reg [63:0] row_b3_1 [0:255];
    (* ram_style = "block" *) reg [63:0] row_b3_2 [0:255];
    (* ram_style = "block" *) reg [63:0] row_b3_3 [0:255];
    reg [255:0] voxel_block0;
    reg [255:0] voxel_block1;
    reg [255:0] voxel_block2;
    reg [255:0] voxel_block3;
    reg [255:0] pair_even_block0;
    reg [255:0] pair_even_block1;
    reg [255:0] pair_even_block2;
    reg [255:0] pair_even_block3;
    reg [511:0] selected_output_beat;

    // ===================== NCHWc16 Blocked-layout Writer =====================
    wire [15:0] spatial_cells_w;
    reg  [31:0] output_cblk_stride_r;
    reg  [31:0] spatial_offset_r;
    reg  [31:0] x_output_offset_r;
    reg  [ 7:0] voxel_x_r;
    reg  [ 7:0] voxel_y_r;
    reg  [ 7:0] z_index_r;
    reg  [31:0] z_output_base_r;
    reg  [31:0] block_write_addr_r;
    reg  [ 3:0] output_block_r;
    reg         last_spatial_r;

    assign spatial_cells_w = bev_x * bev_y;

    // ===================== Read Data Handshake =====================
    assign rd_data_ready = (state == S_WAIT_LUT) ||
                           (state == S_WAIT_FEAT);

    // ===================== State Transition =====================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) state <= S_IDLE;
        else            state <= next_state;
    end

    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE: begin
                if (engine_start) begin
                    if (lut_size == 32'd0)
                        next_state = S_DONE;
                    else
                        next_state = S_RD_LUT;
                end
            end
            S_RD_LUT: begin
                if (need_new_batch) begin
                    if (rd_grant) next_state = S_WAIT_LUT;
                end
                else begin
                    next_state = S_PARSE_LUT;
                end
            end
            S_WAIT_LUT: begin
                if (rd_data_valid) next_state = S_PARSE_LUT;
            end
            S_PARSE_LUT: begin
                if (lut_entry_valid)
                    next_state = S_CALC_ADDR;
                else
                    next_state = S_VOXEL_DONE;
            end
            S_CALC_ADDR:  next_state = S_CALC_ADDR2;
            S_CALC_ADDR2: next_state = S_RD_FEAT;
            S_RD_FEAT: begin
                if (rd_grant) next_state = S_WAIT_FEAT;
            end
            S_WAIT_FEAT: begin
                if (rd_data_valid) next_state = S_CONVERT;
            end
            S_CONVERT:      next_state = S_CONVERT_FP16;
            S_CONVERT_FP16: next_state = S_STORE;
            S_STORE: begin
                if (feat_beat_cnt == 2'd3)
                    next_state = S_VOXEL_DONE;
                else
                    next_state = S_RD_FEAT;
            end
            S_VOXEL_DONE: begin
                if (voxel_y_r[0])
                    next_state = S_PREP_WRITE;
                else if (voxel_cnt + 1'b1 >= lut_size)
                    next_state = S_DONE;
                else
                    next_state = S_RD_LUT;
            end
            S_PREP_WRITE: next_state = S_WRITE_VOXEL;
            S_WRITE_VOXEL: begin
                if (wr_grant && output_block_r == 4'd3) begin
                    if (voxel_cnt >= lut_size)
                        next_state = S_DONE;
                    else
                        next_state = S_RD_LUT;
                end
            end
            S_DONE: next_state = S_IDLE;
            default: next_state = S_IDLE;
        endcase
    end

    // ===================== Run Initialization =====================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            output_cblk_stride_r <= 32'd0;
            z_index_r          <= 8'd0;
            z_output_base_r    <= 32'd0;
        end
        else if (state == S_IDLE && engine_start) begin
            // One cblk16 plane has X*Y*16 FP16 values = X*Y*32 bytes.
            output_cblk_stride_r <= {11'd0, spatial_cells_w, 5'd0};
            z_index_r          <= 8'd0;
            z_output_base_r    <= feat3d_wr_addr;
        end else if (state == S_PREP_WRITE && last_spatial_r) begin
            if (z_index_r + 1'b1 < bev_z) begin
                z_index_r <= z_index_r + 1'b1;
                z_output_base_r <= z_output_base_r +
                                   (output_cblk_stride_r << 2);
            end else begin
                z_index_r <= bev_z;
            end
        end
    end

    // ===================== Voxel and LUT Batch Counters =====================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            voxel_cnt   <= 32'd0;
            lut_sub_idx <= 3'd0;
            voxel_x_r   <= 8'd0;
            voxel_y_r   <= 8'd0;
            x_output_offset_r <= 32'd0;
        end
        else if (state == S_IDLE && engine_start) begin
            voxel_cnt   <= 32'd0;
            lut_sub_idx <= 3'd0;
            voxel_x_r   <= 8'd0;
            voxel_y_r   <= 8'd0;
            x_output_offset_r <= 32'd0;
        end
        else if (state == S_VOXEL_DONE) begin
            voxel_cnt   <= voxel_cnt + 1'b1;
            lut_sub_idx <= lut_sub_idx + 1'b1;
            if (voxel_x_r + 1'b1 >= bev_x) begin
                voxel_x_r <= 8'd0;
                x_output_offset_r <= 32'd0;
                if (voxel_y_r + 1'b1 >= bev_y)
                    voxel_y_r <= 8'd0;
                else
                    voxel_y_r <= voxel_y_r + 1'b1;
            end
            else begin
                voxel_x_r <= voxel_x_r + 1'b1;
                x_output_offset_r <= x_output_offset_r +
                                     {19'd0, bev_y, 5'd0};
            end
        end
    end

    // ===================== LUT Batch and Entry Decode =====================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int)
            lut_batch <= 512'd0;
        else if (state == S_WAIT_LUT && rd_data_valid)
            lut_batch <= rd_data;
    end

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            cur_cam_id <= 16'hFFFF;
            cur_u      <= 16'd0;
            cur_v      <= 16'd0;
        end
        else if (state == S_RD_LUT && !need_new_batch) begin
            cur_cam_id <= cur_lut_raw[15:0];
            cur_u      <= cur_lut_raw[31:16];
            cur_v      <= cur_lut_raw[47:32];
        end
        else if (state == S_WAIT_LUT && rd_data_valid) begin
            cur_cam_id <= rd_data[15:0];
            cur_u      <= rd_data[31:16];
            cur_v      <= rd_data[47:32];
        end
    end

    // ===================== Feature Address Pipeline =====================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int)
            pipe_cam_v <= 21'd0;
        else if (state == S_PARSE_LUT && lut_entry_valid)
            pipe_cam_v <= cur_cam_id[7:0] * img_h + {5'd0, cur_v};
    end

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int)
            pixel_idx_r <= 26'd0;
        else if (state == S_CALC_ADDR)
            pixel_idx_r <= pixel_linear_idx_s2;
    end

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int)
            feat_rd_addr_reg <= 32'd0;
        else if (state == S_CALC_ADDR2)
            feat_rd_addr_reg <= feat2d_base_addr +
                                {pixel_idx_r[23:0], 8'b0};
    end

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int)
            feat_beat_cnt <= 2'd0;
        else if (state == S_CALC_ADDR2)
            feat_beat_cnt <= 2'd0;
        else if (state == S_STORE && feat_beat_cnt != 2'd3)
            feat_beat_cnt <= feat_beat_cnt + 1'b1;
    end

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int)
            feat_buffer <= 512'd0;
        else if (state == S_WAIT_FEAT && rd_data_valid)
            feat_buffer <= rd_data;
    end

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int)
            tf32_beat_r <= 512'd0;
        else if (state == S_CONVERT)
            tf32_beat_r <= tf32_beat_w;
    end

    // Register the FP16 rounding result before the beat-selection mux.  This
    // keeps the 200 MHz TF32-to-FP16 cone out of the voxel block write path.
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int)
            converted_beat_r <= 256'd0;
        else if (state == S_CONVERT_FP16)
            converted_beat_r <= converted_beat;
    end

    // ===================== Per-voxel FP16 Block Capture =====================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            voxel_block0 <= 256'd0;
            voxel_block1 <= 256'd0;
            voxel_block2 <= 256'd0;
            voxel_block3 <= 256'd0;
        end
        else if (state == S_IDLE && engine_start) begin
            voxel_block0 <= 256'd0;
            voxel_block1 <= 256'd0;
            voxel_block2 <= 256'd0;
            voxel_block3 <= 256'd0;
        end
        else if (state == S_PARSE_LUT && !lut_entry_valid) begin
            voxel_block0 <= 256'd0;
            voxel_block1 <= 256'd0;
            voxel_block2 <= 256'd0;
            voxel_block3 <= 256'd0;
        end
        else if (state == S_STORE) begin
            case (feat_beat_cnt)
                2'd0: voxel_block0 <= converted_beat_r;
                2'd1: voxel_block1 <= converted_beat_r;
                2'd2: voxel_block2 <= converted_beat_r;
                2'd3: voxel_block3 <= converted_beat_r;
            endcase
        end
    end

    // Store every even-Y row. On the following odd-Y row, synchronously read
    // the matching X entry. This block intentionally has no asynchronous reset:
    // Vivado 2018.3 otherwise dissolves the four 256x256 memories into 262144
    // flip-flops instead of inferring block RAM. Every location is overwritten
    // by its even-Y row before it can be read, including after an engine restart.
    // Separate write/read processes form a simple dual-port RAM template that
    // is recognized reliably by the Vivado 2018.3 synthesis engine.
    always @(posedge clk) begin
        if (state == S_VOXEL_DONE && !voxel_y_r[0]) begin
            row_b0_0[voxel_x_r] <= voxel_block0[ 63:  0];
            row_b0_1[voxel_x_r] <= voxel_block0[127: 64];
            row_b0_2[voxel_x_r] <= voxel_block0[191:128];
            row_b0_3[voxel_x_r] <= voxel_block0[255:192];
            row_b1_0[voxel_x_r] <= voxel_block1[ 63:  0];
            row_b1_1[voxel_x_r] <= voxel_block1[127: 64];
            row_b1_2[voxel_x_r] <= voxel_block1[191:128];
            row_b1_3[voxel_x_r] <= voxel_block1[255:192];
            row_b2_0[voxel_x_r] <= voxel_block2[ 63:  0];
            row_b2_1[voxel_x_r] <= voxel_block2[127: 64];
            row_b2_2[voxel_x_r] <= voxel_block2[191:128];
            row_b2_3[voxel_x_r] <= voxel_block2[255:192];
            row_b3_0[voxel_x_r] <= voxel_block3[ 63:  0];
            row_b3_1[voxel_x_r] <= voxel_block3[127: 64];
            row_b3_2[voxel_x_r] <= voxel_block3[191:128];
            row_b3_3[voxel_x_r] <= voxel_block3[255:192];
        end
    end

    always @(posedge clk) begin
        if (state == S_VOXEL_DONE && voxel_y_r[0]) begin
            pair_even_block0 <= {
                row_b0_3[voxel_x_r], row_b0_2[voxel_x_r],
                row_b0_1[voxel_x_r], row_b0_0[voxel_x_r]
            };
            pair_even_block1 <= {
                row_b1_3[voxel_x_r], row_b1_2[voxel_x_r],
                row_b1_1[voxel_x_r], row_b1_0[voxel_x_r]
            };
            pair_even_block2 <= {
                row_b2_3[voxel_x_r], row_b2_2[voxel_x_r],
                row_b2_1[voxel_x_r], row_b2_0[voxel_x_r]
            };
            pair_even_block3 <= {
                row_b3_3[voxel_x_r], row_b3_2[voxel_x_r],
                row_b3_1[voxel_x_r], row_b3_0[voxel_x_r]
            };
        end
    end

    // Lower 256 bits are y_even; upper 256 bits are y_odd.
    always @(*) begin
        case (output_block_r[1:0])
            2'd0: selected_output_beat = {voxel_block0, pair_even_block0};
            2'd1: selected_output_beat = {voxel_block1, pair_even_block1};
            2'd2: selected_output_beat = {voxel_block2, pair_even_block2};
            default: selected_output_beat = {voxel_block3, pair_even_block3};
        endcase
    end

    // ===================== Output Address Sequence =====================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            spatial_offset_r   <= 32'd0;
            last_spatial_r     <= 1'b0;
            output_block_r     <= 4'd0;
            block_write_addr_r <= 32'd0;
        end
        else if (state == S_IDLE && engine_start) begin
            spatial_offset_r   <= 32'd0;
            last_spatial_r     <= 1'b0;
            output_block_r     <= 4'd0;
            block_write_addr_r <= feat3d_wr_addr;
        end
        else if (state == S_VOXEL_DONE && voxel_y_r[0]) begin
            // Spatial order is [x][y]. Each position is 16 FP16 = 32 B;
            // this write starts at the preceding even Y position.
            spatial_offset_r <= x_output_offset_r +
                                (({24'd0, voxel_y_r} - 1'b1) << 5);
            last_spatial_r <= (voxel_x_r + 1'b1 >= bev_x) &&
                              (voxel_y_r + 1'b1 >= bev_y);
            output_block_r     <= 4'd0;
        end
        else if (state == S_PREP_WRITE) begin
            block_write_addr_r <= z_output_base_r + spatial_offset_r;
        end
        else if (state == S_WRITE_VOXEL && wr_grant &&
                 output_block_r != 4'd3) begin
            output_block_r     <= output_block_r + 1'b1;
            block_write_addr_r <= block_write_addr_r +
                                  output_cblk_stride_r;
        end
    end

    // ===================== Read Request =====================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            rd_addr <= 32'd0;
            rd_req  <= 1'b0;
        end
        else if (rd_grant) begin
            rd_req <= 1'b0;
        end
        else begin
            case (state)
                S_RD_LUT: begin
                    if (need_new_batch) begin
                        rd_addr <= lut_rd_byte_addr;
                        rd_req  <= 1'b1;
                    end
                    else begin
                        rd_req <= 1'b0;
                    end
                end
                S_RD_FEAT: begin
                    rd_addr <= feat_rd_addr_reg +
                               {24'd0, feat_beat_cnt, 6'b0};
                    rd_req  <= 1'b1;
                end
                default: rd_req <= 1'b0;
            endcase
        end
    end

    // ===================== Write Request =====================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            wr_addr <= 32'd0;
            wr_data <= 512'd0;
            wr_req  <= 1'b0;
        end
        else if (wr_grant) begin
            wr_req <= 1'b0;
        end
        else if (state == S_WRITE_VOXEL) begin
            wr_addr <= block_write_addr_r;
            wr_data <= selected_output_beat;
            wr_req  <= 1'b1;
        end
        else begin
            wr_req <= 1'b0;
        end
    end

    // No write-response channel exists on User_Ddr. wr_grant is therefore the
    // authoritative indication that the final address/data beat was accepted.
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int)
            engine_done <= 1'b0;
        else
            engine_done <= (state == S_DONE);
    end

endmodule
