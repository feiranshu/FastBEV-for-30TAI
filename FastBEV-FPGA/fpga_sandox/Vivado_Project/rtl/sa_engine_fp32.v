`timescale 1ns/1ps
//==============================================================================
// File Name     : sa_engine_fp32.v
// Module Name   : sa_engine  (drop-in replacement for int8 version)
// Project       : Fast-BEV Part2 FPGA Accelerator
// Description   : FP32 Spatial Alignment (SA) engine for temporal fusion
//
// TIMING FIX v5: uint33_to_fp32 pipeline split (WNS fix from -0.249ns)
//   v4 issue: DSP48 (w*_int_r) → uint33_to_fp32 → w*_fp32_r = 9 LUT levels
//     Path: w10_int_r0/P[26] → find_msb + barrel_shift → w10_fp32_r_reg[8]/D
//   v5 fix: Insert S_CALC_WEIGHT2 state, split into two stages:
//     Stage 1 (S_CALC_WEIGHT2): 33-bit priority encoder → w*_msb_r  (≤6 LUTs)
//     Stage 2 (S_CALC_ADDR):    barrel shift + assemble  → w*_fp32_r (≤4 LUTs)
//   Cost: +1 cycle per pixel (S_CALC_WEIGHT2), zero impact on throughput
//         (dominated by DDR latency in S_RD_P*/S_WAIT_P* states)
//
// TIMING FIX v4: fp32_add -> 5-stage pipeline (was 3-stage in v3).
//   v3: fp32_add 3-stg -> WNS = -0.971ns
//        Path A: interp_cnt -> add MUX -> fp32_add Stage1 -> s1_shift_amt
//        Path B: s2_mant_sum -> leading-one -> normalize -> result
//   v4 fix:
//     (1) Add Stage 0 input register to fp32_add (breaks Path A)
//     (2) Split Stage 3 into two stages (breaks Path B)
//         Stage 3: leading-one encoder -> register lead_pos
//         Stage 4: normalize barrel shift + output
//
//   fp32_mul latency = 3 cycles (unchanged)
//   fp32_add latency = 5 cycles (was 3)
//
//   Interpolation pipeline: 18 cycles (interp_cnt 0~17, was 0~13)
//
//   Interp schedule (per beat, 16 channels parallel):
//     cnt0:  feed MUL(w00, F00)
//     cnt1:  feed MUL(w01, F01)
//     cnt2:  feed MUL(w10, F10)
//     cnt3:  feed MUL(w11, F11)                     mul_out=w00*F00 -> prod0
//     cnt4:  feed ADD(prod0, mul_out=w01*F01)        mul_out=w01*F01
//     cnt5:                                          mul_out=w10*F10 -> prod2
//     cnt6:                                          mul_out=w11*F11 -> mul_w11_r
//     cnt7:  feed ADD(prod2, mul_w11_r)
//     cnt8:  (ADD pipeline)
//     cnt9:  add_out=s01 -> s01_r                    (cnt4 + 5)
//     cnt10-11: (waiting)
//     cnt12: feed ADD(s01_r, add_out=s23)            add_out=s23 (cnt7 + 5)
//     cnt13-16: (ADD pipeline)
//     cnt17: add_out=final -> interp_result          (cnt12 + 5)
//
//==============================================================================

module sa_engine(

    // --- Control ---
    input                       engine_start     ,
    output reg                  engine_done      ,

    // --- Configuration ---
    input         [   31 : 0]   sa_src_addr      ,
    input         [   31 : 0]   sa_dst_addr      ,
    input         [   31 : 0]   sa_size          ,
    input         [    7 : 0]   bev_x            ,
    input         [    7 : 0]   bev_y            ,
    input         [    7 : 0]   bev_z            ,

    // --- Index Affine Parameters (Q16.16 signed fixed-point) ---
    input         [   31 : 0]   xform_a00        ,  // A
    input         [   31 : 0]   xform_a01        ,  // B
    input         [   31 : 0]   xform_a02        ,  // C
    input         [   31 : 0]   xform_a10        ,  // D
    input         [   31 : 0]   xform_a11        ,  // E
    input         [   31 : 0]   xform_a12        ,  // F

    // --- PLDDR Read Request (to arbiter) ---
    output reg    [   31 : 0]   rd_addr          ,
    output reg                  rd_req           ,
    input                       rd_grant         ,

    // --- PLDDR Read Data (from arbiter, 512-bit) ---
    input         [  511 : 0]   rd_data          ,
    input                       rd_data_valid    ,
    output                      rd_data_ready    ,

    // --- PLDDR Write Request (to arbiter) ---
    output reg    [   31 : 0]   wr_addr          ,
    output reg    [  511 : 0]   wr_data          ,
    output reg                  wr_req           ,
    input                       wr_grant         ,

    input                       clk              ,
    input                       rst_n
);

    // ===================== Aliases for readability =====================
    wire signed [31:0] PARAM_A = xform_a00;
    wire signed [31:0] PARAM_B = xform_a01;
    wire signed [31:0] PARAM_C = xform_a02;
    wire signed [31:0] PARAM_D = xform_a10;
    wire signed [31:0] PARAM_E = xform_a11;
    wire signed [31:0] PARAM_F = xform_a12;

    // ===================== State Machine =====================
    localparam S_IDLE        = 5'd0;
    localparam S_CALC_COORD  = 5'd1;
    localparam S_CHECK_BOUND = 5'd2;
    localparam S_CALC_WEIGHT  = 5'd3;
    localparam S_CALC_WEIGHT2 = 5'd18;  // TIMING FIX v5: split uint33_to_fp32
    localparam S_CALC_ADDR   = 5'd4;
    localparam S_RD_P00      = 5'd5;
    localparam S_WAIT_P00    = 5'd6;
    localparam S_RD_P01      = 5'd7;
    localparam S_WAIT_P01    = 5'd8;
    localparam S_RD_P10      = 5'd9;
    localparam S_WAIT_P10    = 5'd10;
    localparam S_RD_P11      = 5'd11;
    localparam S_WAIT_P11    = 5'd12;
    localparam S_INTERP      = 5'd13;   // 18 cycles now (was 14 in v3)
    localparam S_WR_DST      = 5'd14;
    localparam S_WR_ZERO     = 5'd15;
    localparam S_NEXT_PIX    = 5'd16;
    localparam S_DONE        = 5'd17;

    reg [4:0] state, next_state;

    // ===================== Internal Reset =====================
    (* MAX_FANOUT = 256 *)
    reg rst_n_int;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rst_n_int <= 1'b0;
        else        rst_n_int <= 1'b1;
    end

    // ===================== Pixel Counter & Coordinates =====================
    reg  [31:0]  pix_cnt;
    reg  [15:0]  cur_x;
    reg  [15:0]  cur_y;

    // ===================== Index Affine Registers (Q16.16 signed) =====================
    reg  signed [31:0] u_row_start;
    reg  signed [31:0] v_row_start;
    reg  signed [31:0] u_cur;
    reg  signed [31:0] v_cur;

    // ===================== Parsed Coordinate Fields =====================
    reg  signed [15:0] u_int_r;
    reg  signed [15:0] v_int_r;
    reg         [15:0] alpha_r;
    reg         [15:0] beta_r;

    // ===================== Validity =====================
    wire signed [15:0] u_int_wire = u_cur[31:16];
    wire signed [15:0] v_int_wire = v_cur[31:16];
    wire signed [15:0] bound_x   = {8'd0, bev_x} - 16'd1;
    wire signed [15:0] bound_y   = {8'd0, bev_y} - 16'd1;
    wire src_in_bounds;
    assign src_in_bounds = (u_int_wire >= 16'sd0) && (u_int_wire < bound_x) &&
                           (v_int_wire >= 16'sd0) && (v_int_wire < bound_y);
    reg  voxel_valid;

    // ===================== Weight Registers =====================
    reg  [32:0] w00_int_r, w01_int_r, w10_int_r, w11_int_r;
    reg  [31:0] w00_fp32_r, w01_fp32_r, w10_fp32_r, w11_fp32_r;

    // ===================== Address Pipeline =====================
    reg  [31:0] base_offset_r;
    reg  [31:0] addr_p00_r, addr_p01_r, addr_p10_r, addr_p11_r;
    reg  [31:0] dst_addr_base_r;

    // ===================== Beat Counter =====================
    reg  [1:0]  beat_cnt;

    // ===================== Neighbor Pixel Buffers =====================
    reg  [511:0] p00_buf, p01_buf, p10_buf, p11_buf;

    // ===================== Interpolation Pipeline =====================
    reg  [4:0]  interp_cnt;           // v4: 0~17 (18 cycles, 5-bit; was 4-bit 0~13)
    reg  [511:0] prod0_r;
    reg  [511:0] prod2_r;
    reg  [511:0] mul_w11_r;           // holds w11*F11 until ADD is ready
    reg  [511:0] s01_r;
    reg  [511:0] interp_result;

    // ===================== FP32 Arithmetic Buses =====================
    reg  [31:0]  mul_op_a;
    wire [511:0] mul_op_b;
    wire [511:0] mul_out;

    reg  [511:0] add_op_a;
    reg  [511:0] add_op_b;
    wire [511:0] add_out;

    // ===================== Read Data Ready =====================
    assign rd_data_ready = (state == S_WAIT_P00) || (state == S_WAIT_P01) ||
                           (state == S_WAIT_P10) || (state == S_WAIT_P11);

    // ===================== uint33-to-fp32 TWO-STAGE PIPELINE (TIMING FIX v5) =====================
    // Original single-cycle uint33_to_fp32 had 9 LUT levels from DSP48 → FF,
    // causing WNS = -0.249ns.  Split into:
    //   Stage 1 (S_CALC_WEIGHT2): 33-bit priority encoder → register msb_pos  (~5 LUT levels)
    //   Stage 2 (S_CALC_ADDR):    barrel shift + assemble using registered msb_pos (~3 LUT levels)

    // --- Stage 1 function: find MSB position (priority encoder only) ---
    function [5:0] find_msb_pos;
        input [32:0] val;
        reg [5:0]  pos;
        reg        found;
        integer k;
        begin
            pos   = 6'd0;
            found = 1'b0;
            for (k = 32; k >= 0; k = k - 1) begin
                if (val[k] && !found) begin
                    pos   = k[5:0];
                    found = 1'b1;
                end
            end
            find_msb_pos = pos;
        end
    endfunction

    // --- Stage 2 function: barrel shift + assemble fp32 from registered msb_pos ---
    function [31:0] assemble_fp32;
        input [32:0] val;
        input [5:0]  msb_pos;
        input        is_zero;
        reg [7:0]  fp_exp;
        reg [32:0] shifted;
        begin
            if (is_zero) begin
                assemble_fp32 = 32'd0;
            end else begin
                fp_exp = {2'b00, msb_pos} + 8'd95;
                if (msb_pos > 6'd23)
                    shifted = val >> (msb_pos - 6'd23);
                else
                    shifted = val << (6'd23 - msb_pos);
                assemble_fp32 = {1'b0, fp_exp, shifted[22:0]};
            end
        end
    endfunction

    // --- Stage 1 intermediate registers (registered in S_CALC_WEIGHT2) ---
    reg [5:0] w00_msb_r, w01_msb_r, w10_msb_r, w11_msb_r;
    reg       w00_zero_r, w01_zero_r, w10_zero_r, w11_zero_r;

    // ===========================================================================
    //  STATE TRANSITION
    // ===========================================================================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) state <= S_IDLE;
        else            state <= next_state;
    end

    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE:        if (engine_start) next_state = S_CALC_COORD;
            S_CALC_COORD:  next_state = S_CHECK_BOUND;
            S_CHECK_BOUND: next_state = src_in_bounds ? S_CALC_WEIGHT : S_WR_ZERO;
            S_CALC_WEIGHT: next_state = S_CALC_WEIGHT2;
            S_CALC_WEIGHT2: next_state = S_CALC_ADDR;
            S_CALC_ADDR:   next_state = S_RD_P00;

            S_RD_P00:   if (rd_grant)      next_state = S_WAIT_P00;
            S_WAIT_P00: if (rd_data_valid) next_state = S_RD_P01;
            S_RD_P01:   if (rd_grant)      next_state = S_WAIT_P01;
            S_WAIT_P01: if (rd_data_valid) next_state = S_RD_P10;
            S_RD_P10:   if (rd_grant)      next_state = S_WAIT_P10;
            S_WAIT_P10: if (rd_data_valid) next_state = S_RD_P11;
            S_RD_P11:   if (rd_grant)      next_state = S_WAIT_P11;
            S_WAIT_P11: if (rd_data_valid) next_state = S_INTERP;

            S_INTERP: begin
                if (interp_cnt == 5'd17)   // v4: 18 cycles (was 4'd13 = 14 cycles)
                    next_state = S_WR_DST;
            end

            S_WR_DST: begin
                if (wr_grant) begin
                    if (beat_cnt == 2'd3) next_state = S_NEXT_PIX;
                    else                  next_state = S_RD_P00;
                end
            end

            S_WR_ZERO: begin
                if (wr_grant) begin
                    if (beat_cnt == 2'd3) next_state = S_NEXT_PIX;
                    else                  next_state = S_WR_ZERO;
                end
            end

            S_NEXT_PIX: begin
                if (pix_cnt >= sa_size - 1) next_state = S_DONE;
                else                        next_state = S_CALC_COORD;
            end

            S_DONE:  next_state = S_IDLE;
            default: next_state = S_IDLE;
        endcase
    end

    // ===========================================================================
    //  PIXEL COUNTER & X/Y COORDINATES
    // ===========================================================================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            pix_cnt <= 32'd0;
            cur_x   <= 16'd0;
            cur_y   <= 16'd0;
        end
        else if (state == S_IDLE && engine_start) begin
            pix_cnt <= 32'd0;
            cur_x   <= 16'd0;
            cur_y   <= 16'd0;
        end
        else if (state == S_NEXT_PIX) begin
            pix_cnt <= pix_cnt + 1'b1;
            if (cur_x == bev_x - 1) begin
                cur_x <= 16'd0;
                cur_y <= cur_y + 1'b1;
            end else begin
                cur_x <= cur_x + 1'b1;
            end
        end
    end

    // ===========================================================================
    //  COORDINATE UPDATE (incremental)
    // ===========================================================================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            u_row_start <= 32'sd0;
            v_row_start <= 32'sd0;
        end
        else if (state == S_IDLE && engine_start) begin
            u_row_start <= PARAM_C;
            v_row_start <= PARAM_F;
        end
        else if (state == S_NEXT_PIX && cur_x == bev_x - 1) begin
            u_row_start <= u_row_start + PARAM_B;
            v_row_start <= v_row_start + PARAM_E;
        end
    end

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            u_cur <= 32'sd0;
            v_cur <= 32'sd0;
        end
        else if (state == S_CALC_COORD) begin
            if (cur_x == 16'd0) begin
                u_cur <= u_row_start;
                v_cur <= v_row_start;
            end else begin
                u_cur <= u_cur + PARAM_A;
                v_cur <= v_cur + PARAM_D;
            end
        end
    end

    // ===========================================================================
    //  BOUNDARY CHECK & FRACTIONAL EXTRACTION
    // ===========================================================================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            u_int_r     <= 16'sd0;
            v_int_r     <= 16'sd0;
            alpha_r     <= 16'd0;
            beta_r      <= 16'd0;
            voxel_valid <= 1'b0;
        end
        else if (state == S_CHECK_BOUND) begin
            u_int_r     <= u_cur[31:16];
            v_int_r     <= v_cur[31:16];
            alpha_r     <= u_cur[15:0];
            beta_r      <= v_cur[15:0];
            voxel_valid <= src_in_bounds;
        end
    end

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int)
            dst_addr_base_r <= 32'd0;
        else if (state == S_CHECK_BOUND)
            dst_addr_base_r <= sa_dst_addr + {pix_cnt[23:0], 8'b0};
    end

    // ===========================================================================
    //  WEIGHT COMPUTATION (Q0.32)
    // ===========================================================================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            w00_int_r <= 33'd0;
            w01_int_r <= 33'd0;
            w10_int_r <= 33'd0;
            w11_int_r <= 33'd0;
        end
        else if (state == S_CALC_WEIGHT) begin
            w00_int_r <= (17'd65536 - {1'b0, alpha_r}) * (17'd65536 - {1'b0, beta_r});
            w01_int_r <= {1'b0, alpha_r}                * (17'd65536 - {1'b0, beta_r});
            w10_int_r <= (17'd65536 - {1'b0, alpha_r}) * {1'b0, beta_r};
            w11_int_r <= {1'b0, alpha_r}                * {1'b0, beta_r};
        end
    end

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int)
            base_offset_r <= 32'd0;
        else if (state == S_CALC_WEIGHT)
            base_offset_r <= {16'd0, v_int_r[7:0]} * {24'd0, bev_x} + {16'd0, u_int_r[7:0]};
    end

    // ===========================================================================
    //  WEIGHT→FP32 STAGE 1: Priority encoder (S_CALC_WEIGHT2, TIMING FIX v5)
    //  Critical path was: DSP48 P → find_msb + barrel_shift → w*_fp32_r (9 LUTs)
    //  Now split: DSP48 P → find_msb → w*_msb_r (≤6 LUTs, fits 5ns)
    // ===========================================================================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            w00_msb_r  <= 6'd0;  w01_msb_r  <= 6'd0;
            w10_msb_r  <= 6'd0;  w11_msb_r  <= 6'd0;
            w00_zero_r <= 1'b1;  w01_zero_r <= 1'b1;
            w10_zero_r <= 1'b1;  w11_zero_r <= 1'b1;
        end
        else if (state == S_CALC_WEIGHT2) begin
            w00_msb_r  <= find_msb_pos(w00_int_r);
            w01_msb_r  <= find_msb_pos(w01_int_r);
            w10_msb_r  <= find_msb_pos(w10_int_r);
            w11_msb_r  <= find_msb_pos(w11_int_r);
            w00_zero_r <= (w00_int_r == 33'd0);
            w01_zero_r <= (w01_int_r == 33'd0);
            w10_zero_r <= (w10_int_r == 33'd0);
            w11_zero_r <= (w11_int_r == 33'd0);
        end
    end

    // ===========================================================================
    //  WEIGHT→FP32 STAGE 2: Barrel shift + assemble (S_CALC_ADDR)
    //  Uses registered msb_pos: w*_int_r → barrel_shift → w*_fp32_r (≤4 LUTs)
    // ===========================================================================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            w00_fp32_r <= 32'd0;
            w01_fp32_r <= 32'd0;
            w10_fp32_r <= 32'd0;
            w11_fp32_r <= 32'd0;
        end
        else if (state == S_CALC_ADDR) begin
            w00_fp32_r <= assemble_fp32(w00_int_r, w00_msb_r, w00_zero_r);
            w01_fp32_r <= assemble_fp32(w01_int_r, w01_msb_r, w01_zero_r);
            w10_fp32_r <= assemble_fp32(w10_int_r, w10_msb_r, w10_zero_r);
            w11_fp32_r <= assemble_fp32(w11_int_r, w11_msb_r, w11_zero_r);
        end
    end

    wire [31:0] offset_p01_w = base_offset_r + 32'd1;
    wire [31:0] offset_p10_w = base_offset_r + {24'd0, bev_x};
    wire [31:0] offset_p11_w = base_offset_r + {24'd0, bev_x} + 32'd1;

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            addr_p00_r <= 32'd0;
            addr_p01_r <= 32'd0;
            addr_p10_r <= 32'd0;
            addr_p11_r <= 32'd0;
        end
        else if (state == S_CALC_ADDR) begin
            addr_p00_r <= sa_src_addr + {base_offset_r[23:0], 8'b0};
            addr_p01_r <= sa_src_addr + {offset_p01_w[23:0],  8'b0};
            addr_p10_r <= sa_src_addr + {offset_p10_w[23:0],  8'b0};
            addr_p11_r <= sa_src_addr + {offset_p11_w[23:0],  8'b0};
        end
    end

    // ===========================================================================
    //  BEAT COUNTER
    // ===========================================================================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int)
            beat_cnt <= 2'd0;
        else if (state == S_CALC_ADDR)
            beat_cnt <= 2'd0;
        else if (state == S_CHECK_BOUND && !src_in_bounds)
            beat_cnt <= 2'd0;
        else if ((state == S_WR_DST || state == S_WR_ZERO) && wr_grant && beat_cnt != 2'd3)
            beat_cnt <= beat_cnt + 1'b1;
    end

    // ===========================================================================
    //  NEIGHBOR PIXEL BUFFERS
    // ===========================================================================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            p00_buf <= 512'd0;
            p01_buf <= 512'd0;
            p10_buf <= 512'd0;
            p11_buf <= 512'd0;
        end else begin
            if (state == S_WAIT_P00 && rd_data_valid) p00_buf <= rd_data;
            if (state == S_WAIT_P01 && rd_data_valid) p01_buf <= rd_data;
            if (state == S_WAIT_P10 && rd_data_valid) p10_buf <= rd_data;
            if (state == S_WAIT_P11 && rd_data_valid) p11_buf <= rd_data;
        end
    end

    // ===========================================================================
    //  INTERPOLATION COUNTER  (v4: 0~17, was 0~13 in v3)
    // ===========================================================================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int)
            interp_cnt <= 5'd0;
        else if (state == S_INTERP)
            interp_cnt <= interp_cnt + 1'b1;
        else
            interp_cnt <= 5'd0;
    end

    // ===========================================================================
    //  INTERPOLATION PIPELINE -- MUX CONTROL (3-cycle MUL + 5-cycle ADD)
    //
    //  MUL inputs at cnt N -> MUL output at cnt N+3
    //  ADD inputs at cnt N -> ADD output at cnt N+5
    //
    //  Schedule (18 cycles, cnt 0~17):
    //    cnt0:  MUL <- (w00, F00)
    //    cnt1:  MUL <- (w01, F01)
    //    cnt2:  MUL <- (w10, F10)
    //    cnt3:  MUL <- (w11, F11)                      mul_out = w00*F00 -> save prod0
    //    cnt4:  ADD <- (prod0, mul_out=w01*F01)
    //    cnt5:                                          mul_out = w10*F10 -> save prod2
    //    cnt6:                                          mul_out = w11*F11 -> save mul_w11_r
    //    cnt7:  ADD <- (prod2, mul_w11_r)
    //    cnt8:  (ADD pipeline)
    //    cnt9:  add_out = s01 -> save s01_r              (cnt4 + 5)
    //    cnt10: (waiting)
    //    cnt11: (waiting)
    //    cnt12: ADD <- (s01_r, add_out=s23)              add_out = s23 (cnt7 + 5)
    //    cnt13-16: (ADD pipeline)
    //    cnt17: add_out = final -> save interp_result    (cnt12 + 5)
    // ===========================================================================

    // --- Multiplier input A: weight ---
    always @(*) begin
        case (interp_cnt)
            5'd0:    mul_op_a = w00_fp32_r;
            5'd1:    mul_op_a = w01_fp32_r;
            5'd2:    mul_op_a = w10_fp32_r;
            5'd3:    mul_op_a = w11_fp32_r;
            default: mul_op_a = 32'd0;
        endcase
    end

    // --- Multiplier input B: feature data ---
    assign mul_op_b = (interp_cnt == 5'd0) ? p00_buf :
                      (interp_cnt == 5'd1) ? p01_buf :
                      (interp_cnt == 5'd2) ? p10_buf :
                      (interp_cnt == 5'd3) ? p11_buf : 512'd0;

    // --- Adder input A ---
    // v4: third ADD at cnt12 (was cnt10 in v3)
    always @(*) begin
        case (interp_cnt)
            5'd4:    add_op_a = prod0_r;    // s01 = prod0 + w01*F01
            5'd7:    add_op_a = prod2_r;    // s23 = prod2 + w11*F11
            5'd12:   add_op_a = s01_r;      // final = s01 + s23
            default: add_op_a = 512'd0;
        endcase
    end

    // --- Adder input B ---
    // v4: third ADD at cnt12 (was cnt10 in v3)
    always @(*) begin
        case (interp_cnt)
            5'd4:    add_op_b = mul_out;    // w01*F01
            5'd7:    add_op_b = mul_w11_r;  // w11*F11 (saved from cnt6)
            5'd12:   add_op_b = add_out;    // s23
            default: add_op_b = 512'd0;
        endcase
    end

    // ===========================================================================
    //  INTERPOLATION PIPELINE -- INTERMEDIATE REGISTERS
    //    cnt3:  prod0_r       <- mul_out (= w00*F00)
    //    cnt5:  prod2_r       <- mul_out (= w10*F10)
    //    cnt6:  mul_w11_r     <- mul_out (= w11*F11)
    //    cnt9:  s01_r         <- add_out (= prod0 + w01*F01)     [v4: was cnt7]
    //    cnt17: interp_result <- add_out (= s01 + s23 = final)   [v4: was cnt13]
    // ===========================================================================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            prod0_r   <= 512'd0;
            prod2_r   <= 512'd0;
            mul_w11_r <= 512'd0;
            s01_r     <= 512'd0;
        end
        else if (state == S_INTERP) begin
            if (interp_cnt == 5'd3)
                prod0_r <= mul_out;
            if (interp_cnt == 5'd5)
                prod2_r <= mul_out;
            if (interp_cnt == 5'd6)
                mul_w11_r <= mul_out;
            if (interp_cnt == 5'd9)              // v4: was 4'd7
                s01_r <= add_out;
        end
    end

    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int)
            interp_result <= 512'd0;
        else if (state == S_INTERP && interp_cnt == 5'd17)   // v4: was 4'd13
            interp_result <= add_out;
    end

    // ===========================================================================
    //  FP32 ARITHMETIC INSTANCES (16 channels parallel)
    // ===========================================================================
    genvar gi;
    generate
        for (gi = 0; gi < 16; gi = gi + 1) begin : interp_ch
            fp32_mul u_mul (
                .clk    ( clk                        ),
                .a      ( mul_op_a                   ),
                .b      ( mul_op_b[gi*32 +: 32]      ),
                .result ( mul_out [gi*32 +: 32]      )
            );
            fp32_add u_add (
                .clk    ( clk                        ),
                .a      ( add_op_a[gi*32 +: 32]      ),
                .b      ( add_op_b[gi*32 +: 32]      ),
                .result ( add_out [gi*32 +: 32]      )
            );
        end
    endgenerate

    // ===========================================================================
    //  READ ADDRESS / REQUEST
    // ===========================================================================
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
                S_RD_P00: begin rd_addr <= addr_p00_r + {24'd0, beat_cnt, 6'b0}; rd_req <= 1'b1; end
                S_RD_P01: begin rd_addr <= addr_p01_r + {24'd0, beat_cnt, 6'b0}; rd_req <= 1'b1; end
                S_RD_P10: begin rd_addr <= addr_p10_r + {24'd0, beat_cnt, 6'b0}; rd_req <= 1'b1; end
                S_RD_P11: begin rd_addr <= addr_p11_r + {24'd0, beat_cnt, 6'b0}; rd_req <= 1'b1; end
                default:  begin rd_req <= 1'b0; end
            endcase
        end
    end

    // ===========================================================================
    //  WRITE ADDRESS / DATA / REQUEST
    // ===========================================================================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int) begin
            wr_addr <= 32'd0;
            wr_data <= 512'd0;
            wr_req  <= 1'b0;
        end
        else if (wr_grant) begin
            wr_req <= 1'b0;
        end
        else if (state == S_WR_DST) begin
            wr_addr <= dst_addr_base_r + {24'd0, beat_cnt, 6'b0};
            wr_data <= interp_result;
            wr_req  <= 1'b1;
        end
        else if (state == S_WR_ZERO) begin
            wr_addr <= dst_addr_base_r + {24'd0, beat_cnt, 6'b0};
            wr_data <= 512'd0;
            wr_req  <= 1'b1;
        end
        else begin
            wr_req <= 1'b0;
        end
    end

    // ===========================================================================
    //  DONE SIGNAL (single-cycle pulse, NOT sticky — TIMING FIX v5 carries forward)
    // ===========================================================================
    always @(posedge clk or negedge rst_n_int) begin
        if (!rst_n_int)
            engine_done <= 1'b0;
        else
            engine_done <= (state == S_DONE);
    end

endmodule


//==============================================================================
// FP32 Multiplier -- 3-stage pipelined (unchanged from v3)
//
//   Stage 0 (~3 LUT levels): field extraction + input register.
//            Vivado absorbs s0_mant_a/b into DSP48E1 AREG/BREG.
//   Stage 1 (DSP48 internal): 24x24 mantissa multiply, cascaded DSP48,
//            product stored in DSP48 PREG. Exponent/sign pipelined alongside.
//   Stage 2 (~3 LUT levels): normalize (1-bit shift + bit select), output.
//
// Total latency: 3 clock cycles.
//==============================================================================
module fp32_mul (
    input             clk    ,
    input      [31:0] a      ,
    input      [31:0] b      ,
    output reg [31:0] result
);
    // --- Combinational: field extraction ---
    wire [7:0]  exp_a = a[30:23];
    wire [7:0]  exp_b = b[30:23];

    // ---- Stage 0: Input Register (absorbed into DSP48 AREG/BREG) ----
    // Mantissa always formed with leading 1 (zero handled at output)
    reg [23:0]  s0_mant_a;
    reg [23:0]  s0_mant_b;
    reg         s0_res_sign;
    reg signed [9:0] s0_exp_sum;
    reg         s0_either_zero;

    always @(posedge clk) begin
        s0_mant_a     <= {1'b1, a[22:0]};
        s0_mant_b     <= {1'b1, b[22:0]};
        s0_res_sign   <= a[31] ^ b[31];
        s0_exp_sum    <= $signed({2'b0, exp_a})
                       + $signed({2'b0, exp_b})
                       - 10'sd127;
        s0_either_zero <= (exp_a == 8'd0) || (exp_b == 8'd0);
    end

    // ---- Stage 1: DSP48 multiply on registered inputs ----
    // With registered inputs, Vivado infers DSP48E1 with AREG=1, BREG=1.
    (* use_dsp = "yes" *)
    wire [47:0] mant_prod_w = s0_mant_a * s0_mant_b;

    reg        s1_sign;
    reg signed [9:0] s1_exp;
    reg [47:0] s1_prod;
    reg        s1_zero;

    always @(posedge clk) begin
        s1_sign <= s0_res_sign;
        s1_exp  <= s0_exp_sum;
        s1_prod <= mant_prod_w;
        s1_zero <= s0_either_zero;
    end

    // ---- Stage 2: Normalize + Output ----
    wire        s2_msb  = s1_prod[47];
    wire [22:0] s2_mant = s2_msb ? s1_prod[46:24] : s1_prod[45:23];
    wire signed [9:0] s2_exp = s1_exp + {9'd0, s2_msb};

    always @(posedge clk) begin
        if (s1_zero)
            result <= 32'd0;
        else if (s2_exp <= 10'sd0 || s2_exp >= 10'sd255)
            result <= 32'd0;
        else
            result <= {s1_sign, s2_exp[7:0], s2_mant};
    end
endmodule


//==============================================================================
// FP32 Adder -- 5-stage pipelined (TIMING FIX v4)
//
// Previous 3-stage (v3) had two critical path categories:
//   Path A (WNS=-0.971): interp_cnt MUX -> Stage1 combinational -> s1_shift_amt
//   Path B (WNS=-0.773): s2_mant_sum -> leading-one -> normalize -> result
//
// Fix:
//   (1) Add Stage 0 input register: breaks MUX -> Stage1 path
//   (2) Split old Stage 3 into Stage 3 (leading-one) + Stage 4 (normalize)
//
// Pipeline stages:
//   Stage 0 (~0 LUT levels): Input register. Breaks combinational path from
//            interp_cnt MUX and fp32_mul result to fp32_add internals.
//   Stage 1 (~8 LUT levels): field extraction, exponent compare,
//            magnitude swap, exp_diff, shift_amt, eff_sub flag.
//   Stage 2 (~7 LUT levels): barrel shift alignment (4-level LUT cascade),
//            25-bit mantissa add/sub (7 CARRY4 slices).
//   Stage 3 (~5 LUT levels): leading-one priority encoder on 25-bit mantissa.
//   Stage 4 (~5 LUT levels): normalization barrel shift, exp adjust, output.
//
// Total latency: 5 clock cycles.
//==============================================================================
module fp32_add (
    input             clk    ,
    input      [31:0] a      ,
    input      [31:0] b      ,
    output reg [31:0] result
);

    // ========== Stage 0: Input Register (NEW in v4) ==========
    // Breaks the critical path from interp_cnt MUX to Stage 1 logic.
    reg [31:0] s0_a, s0_b;
    always @(posedge clk) begin
        s0_a <= a;
        s0_b <= b;
    end

    // ========== Stage 1 Combinational: field extraction + compare + swap ==========
    wire [7:0]  exp_a = s0_a[30:23];
    wire [7:0]  exp_b = s0_b[30:23];
    wire        a_is_zero = (exp_a == 8'd0);
    wire        b_is_zero = (exp_b == 8'd0);

    wire [23:0] mant_a = a_is_zero ? 24'd0 : {1'b1, s0_a[22:0]};
    wire [23:0] mant_b = b_is_zero ? 24'd0 : {1'b1, s0_b[22:0]};

    // Magnitude comparison
    wire a_ge_b = (exp_a > exp_b) || ((exp_a == exp_b) && (mant_a >= mant_b));

    wire [7:0]  exp_l_w  = a_ge_b ? exp_a  : exp_b;
    wire [23:0] mant_l_w = a_ge_b ? mant_a : mant_b;
    wire [23:0] mant_s_w = a_ge_b ? mant_b : mant_a;
    wire        sign_l_w = a_ge_b ? s0_a[31] : s0_b[31];
    wire        sign_s_w = a_ge_b ? s0_b[31] : s0_a[31];

    // Alignment depends only on the exponent magnitude.  Do not feed the
    // 24-bit mantissa comparison (a_ge_b) into this path: when exponents are
    // equal the difference is zero regardless of which mantissa is larger.
    // Keeping the absolute exponent difference independent removes the long
    // mantissa-compare/CARRY chain from s0_* to s1_shift_amt.
    wire        exp_a_ge_b_w = (exp_a >= exp_b);
    wire [7:0]  exp_diff_w   = exp_a_ge_b_w ?
                               (exp_a - exp_b) : (exp_b - exp_a);
    wire [4:0]  shift_amt_w = (exp_diff_w > 8'd24) ? 5'd24 : exp_diff_w[4:0];
    wire        eff_sub_w   = (sign_l_w != sign_s_w);

    // ========== Stage 1 Register (swap + shift_amt + flags) ==========
    reg [23:0] s1_mant_l;
    reg [23:0] s1_mant_s;
    reg [7:0]  s1_exp_l;
    reg [4:0]  s1_shift_amt;
    reg        s1_eff_sub;
    reg        s1_sign_l;
    reg        s1_a_zero;
    reg        s1_b_zero;
    reg [31:0] s1_a_pass;
    reg [31:0] s1_b_pass;

    always @(posedge clk) begin
        s1_mant_l    <= mant_l_w;
        s1_mant_s    <= mant_s_w;
        s1_exp_l     <= exp_l_w;
        s1_shift_amt <= shift_amt_w;
        s1_eff_sub   <= eff_sub_w;
        s1_sign_l    <= sign_l_w;
        s1_a_zero    <= a_is_zero;
        s1_b_zero    <= b_is_zero;
        s1_a_pass    <= s0_a;
        s1_b_pass    <= s0_b;
    end

    // ========== Stage 2 Combinational: align + add/sub ==========
    wire [23:0] s2_mant_s_aligned = s1_mant_s >> s1_shift_amt;

    wire [24:0] s2_mant_sum_w = s1_eff_sub ?
        ({1'b0, s1_mant_l} - {1'b0, s2_mant_s_aligned}) :
        ({1'b0, s1_mant_l} + {1'b0, s2_mant_s_aligned});

    // ========== Stage 2 Register (mant_sum + passthrough) ==========
    reg [24:0] s2_mant_sum;
    reg [7:0]  s2_exp_l;
    reg        s2_sign_l;
    reg        s2_a_zero;
    reg        s2_b_zero;
    reg [31:0] s2_a_pass;
    reg [31:0] s2_b_pass;

    always @(posedge clk) begin
        s2_mant_sum <= s2_mant_sum_w;
        s2_exp_l    <= s1_exp_l;
        s2_sign_l   <= s1_sign_l;
        s2_a_zero   <= s1_a_zero;
        s2_b_zero   <= s1_b_zero;
        s2_a_pass   <= s1_a_pass;
        s2_b_pass   <= s1_b_pass;
    end

    // ========== Stage 3 Combinational: leading-one priority encoder ==========
    // (split from old Stage 3 to reduce logic depth; v4 fix for Path B)
    reg  [4:0] lead_pos;
    reg        lead_found;
    integer fi;
    always @(*) begin
        lead_pos   = 5'd0;
        lead_found = 1'b0;
        for (fi = 24; fi >= 0; fi = fi - 1) begin
            if (s2_mant_sum[fi] && !lead_found) begin
                lead_pos   = fi[4:0];
                lead_found = 1'b1;
            end
        end
    end

    // ========== Stage 3 Register (NEW in v4: breaks s2_mant_sum -> result) ==========
    reg [4:0]  s3_lead_pos;
    reg        s3_lead_found;
    reg [24:0] s3_mant_sum;
    reg [7:0]  s3_exp_l;
    reg        s3_sign_l;
    reg        s3_a_zero;
    reg        s3_b_zero;
    reg [31:0] s3_a_pass;
    reg [31:0] s3_b_pass;

    always @(posedge clk) begin
        s3_lead_pos   <= lead_pos;
        s3_lead_found <= lead_found;
        s3_mant_sum   <= s2_mant_sum;
        s3_exp_l      <= s2_exp_l;
        s3_sign_l     <= s2_sign_l;
        s3_a_zero     <= s2_a_zero;
        s3_b_zero     <= s2_b_zero;
        s3_a_pass     <= s2_a_pass;
        s3_b_pass     <= s2_b_pass;
    end

    // ========== Stage 4 Combinational: normalize + output (was part of Stage 3) ==========
    wire        carry_out  = (s3_lead_pos == 5'd24);
    wire [4:0]  left_shift = (s3_lead_pos < 5'd23) ? (5'd23 - s3_lead_pos) : 5'd0;
    wire [24:0] mant_normed = carry_out              ? (s3_mant_sum >> 1) :
                              (s3_lead_pos < 5'd23)  ? (s3_mant_sum << left_shift) :
                              s3_mant_sum;

    wire signed [9:0] exp_result = carry_out ?
        ($signed({2'b0, s3_exp_l}) + 10'sd1) :
        ($signed({2'b0, s3_exp_l}) - $signed({5'b0, left_shift}));

    wire s4_result_zero = !s3_lead_found || (s3_mant_sum == 25'd0);

    // ========== Stage 4 Output Register ==========
    always @(posedge clk) begin
        if (s3_a_zero && s3_b_zero)
            result <= 32'd0;
        else if (s3_a_zero)
            result <= s3_b_pass;
        else if (s3_b_zero)
            result <= s3_a_pass;
        else if (s4_result_zero)
            result <= 32'd0;
        else if (exp_result <= 10'sd0)
            result <= 32'd0;
        else if (exp_result >= 10'sd255)
            result <= {s3_sign_l, 8'hFE, 23'h7FFFFF};
        else
            result <= {s3_sign_l, exp_result[7:0], mant_normed[22:0]};
    end
endmodule
