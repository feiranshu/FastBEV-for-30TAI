//==============================================================================
// File Name     : sa_engine_int8.v
// Project       : Fast-BEV Part2 FPGA Accelerator
// Description   : INT8 Spatial Alignment engine for group4 temporal fusion.
//
//   Source history layout:
//     one signed-int8 BEV voxel = 64 channels = one 512-bit / 64B DDR beat
//     src_addr = sa_src_addr + (z * bev_x * bev_y + v_int * bev_x + u_int) * 64
//
//   Destination Part3 layout:
//     logical input   [N][X][Y][Z][C], C = 4 frames * 64 channels
//     physical output [N][Z][C/32][X][Y][C%32]
//     byte_addr = base + (((z * 8 + c/32) * X + x) * Y + y) * 32 + c%32.
//   Each 64-channel temporal beat is split into two 32B halves and written by
//   read-modify-write because the DDR write port has no byte enable.
//
//   Affine convention:
//     u = A * j + B * i + C
//     v = D * j + E * i + F
//     A/B/C/D/E/F are signed Q16.16. i is row/y, j is column/x.
//
//   Interpolation:
//     frac_x = u_frac[15:8], frac_y = v_frac[15:8]
//     wx0 = 256 - frac_x, wx1 = frac_x
//     wy0 = 256 - frac_y, wy1 = frac_y
//     signed INT8 bilinear result is round-to-nearest, then saturated to int8.
//==============================================================================
`timescale 1ns/1ps

module sa_engine_int8(

    input                       engine_start     ,
    output reg                  engine_done      ,

    input         [   31 : 0]   sa_src_addr      ,
    input         [   31 : 0]   concat_base_addr ,
    input         [   31 : 0]   sa_size          ,
    input         [    7 : 0]   bev_x            ,
    input         [    7 : 0]   bev_y            ,
    input         [    7 : 0]   bev_z            ,
    input         [    1 : 0]   temporal_idx     ,

    input         [   31 : 0]   xform_a00        ,
    input         [   31 : 0]   xform_a01        ,
    input         [   31 : 0]   xform_a02        ,
    input         [   31 : 0]   xform_a10        ,
    input         [   31 : 0]   xform_a11        ,
    input         [   31 : 0]   xform_a12        ,

    output reg    [   31 : 0]   rd_addr          ,
    output reg                  rd_req           ,
    input                       rd_grant         ,
    input         [  511 : 0]   rd_data          ,
    input                       rd_data_valid    ,
    output                      rd_data_ready    ,

    output reg    [   31 : 0]   wr_addr          ,
    output reg    [  511 : 0]   wr_data          ,
    output reg                  wr_req           ,
    input                       wr_grant         ,

    input                       clk              ,
    input                       rst_n
);

    localparam S_IDLE        = 5'd0;
    localparam S_CALC_COORD  = 5'd1;
    localparam S_CALC_COORD2 = 5'd2;
    localparam S_CHECK_BOUND = 5'd3;
    localparam S_CALC_WEIGHT = 5'd4;
    localparam S_CALC_ADDR   = 5'd5;
    localparam S_RD_P00      = 5'd6;
    localparam S_WAIT_P00    = 5'd7;
    localparam S_RD_P01      = 5'd8;
    localparam S_WAIT_P01    = 5'd9;
    localparam S_RD_P10      = 5'd10;
    localparam S_WAIT_P10    = 5'd11;
    localparam S_RD_P11      = 5'd12;
    localparam S_WAIT_P11    = 5'd13;
    localparam S_INTERP      = 5'd14;
    localparam S_NEXT_PIX    = 5'd15;
    localparam S_DONE        = 5'd16;
    localparam S_RMW_RD      = 5'd17;
    localparam S_RMW_WAIT    = 5'd18;
    localparam S_RMW_WR      = 5'd19;
    localparam S_CALC_COORD3 = 5'd20;
    localparam S_RMW_ADDR0   = 5'd21;
    localparam S_RMW_ADDR1   = 5'd22;
    localparam S_INTERP_MUL  = 5'd23;
    localparam S_INTERP_PAIR = 5'd24;
    localparam S_INTERP_ACC  = 5'd25;
    localparam S_INTERP_MAG  = 5'd26;
    localparam S_INTERP_ROUND= 5'd27;
    localparam S_INTERP_SIGN = 5'd28;
    localparam S_INTERP_STORE= 5'd29;
    localparam S_CALC_BASE0  = 5'd30;
    localparam S_CALC_BASE1  = 5'd31;

    reg [4:0] state;

    reg  [31:0] pix_cnt;
    reg  [15:0] cur_x;
    reg  [15:0] cur_y;
    reg  [15:0] cur_z;

    reg        [23:0] pipe_a00_lo;
    reg        [23:0] pipe_a01_lo;
    reg        [23:0] pipe_a10_lo;
    reg        [23:0] pipe_a11_lo;
    reg signed [24:0] pipe_a00_hi;
    reg signed [24:0] pipe_a01_hi;
    reg signed [24:0] pipe_a10_hi;
    reg signed [24:0] pipe_a11_hi;
    reg signed [47:0] pipe_a00x;
    reg signed [47:0] pipe_a01y;
    reg signed [47:0] pipe_a10x;
    reg signed [47:0] pipe_a11y;
    reg signed [47:0] src_u_fp;
    reg signed [47:0] src_v_fp;

    wire        [23:0] a00_lo_product_w;
    wire        [23:0] a01_lo_product_w;
    wire        [23:0] a10_lo_product_w;
    wire        [23:0] a11_lo_product_w;
    wire signed [24:0] a00_hi_product_w;
    wire signed [24:0] a01_hi_product_w;
    wire signed [24:0] a10_hi_product_w;
    wire signed [24:0] a11_hi_product_w;

    wire signed [15:0] src_u_int_w;
    wire signed [15:0] src_v_int_w;
    wire        [15:0] src_u_frac_w;
    wire        [15:0] src_v_frac_w;
    wire signed [15:0] bound_x_w;
    wire signed [15:0] bound_y_w;
    wire               src_in_bounds_w;

    assign src_u_int_w  = src_u_fp[31:16];
    assign src_v_int_w  = src_v_fp[31:16];
    assign src_u_frac_w = src_u_fp[15:0];
    assign src_v_frac_w = src_v_fp[15:0];
    assign bound_x_w    = $signed({8'd0, bev_x}) - 16'sd1;
    assign bound_y_w    = $signed({8'd0, bev_y}) - 16'sd1;
    assign src_in_bounds_w = (bev_x > 8'd1) && (bev_y > 8'd1) &&
                             (src_u_int_w >= 16'sd0) && (src_u_int_w < bound_x_w) &&
                             (src_v_int_w >= 16'sd0) && (src_v_int_w < bound_y_w);

    assign a00_lo_product_w = xform_a00[15:0] * cur_x[7:0];
    assign a01_lo_product_w = xform_a01[15:0] * cur_y[7:0];
    assign a10_lo_product_w = xform_a10[15:0] * cur_x[7:0];
    assign a11_lo_product_w = xform_a11[15:0] * cur_y[7:0];
    assign a00_hi_product_w = $signed(xform_a00[31:16]) *
                              $signed({1'b0, cur_x[7:0]});
    assign a01_hi_product_w = $signed(xform_a01[31:16]) *
                              $signed({1'b0, cur_y[7:0]});
    assign a10_hi_product_w = $signed(xform_a10[31:16]) *
                              $signed({1'b0, cur_x[7:0]});
    assign a11_hi_product_w = $signed(xform_a11[31:16]) *
                              $signed({1'b0, cur_y[7:0]});

    reg signed [15:0] u_int_r;
    reg signed [15:0] v_int_r;
    reg        [15:0] frac_x_r;
    reg        [15:0] frac_y_r;

    reg [16:0] w00_r;
    reg [16:0] w01_r;
    reg [16:0] w10_r;
    reg [16:0] w11_r;

    reg [15:0] plane_size_r;
    reg [23:0] z_base_offset_r;
    reg [15:0] source_row_offset_r;
    reg [31:0] base_zy_r;
    reg [31:0] base_offset_r;
    reg [31:0] addr_p00_r;
    reg [31:0] addr_p01_r;
    reg [31:0] addr_p10_r;
    reg [31:0] addr_p11_r;
    reg        rmw_half;
    reg [31:0] rmw_addr_r;
    reg        rmw_upper_r;
    reg [255:0] rmw_payload_r;
    reg [511:0] rmw_merge_data;
    reg [18:0] final_group_x_r;
    reg [26:0] final_xy_r;

    reg [511:0] p00_buf;
    reg [511:0] p01_buf;
    reg [511:0] p10_buf;
    reg [511:0] p11_buf;
    reg [511:0] interp_result;
    reg [1:0]   interp_group;

    // The interpolation datapath is deliberately split at every arithmetic
    // level. These registers have no asynchronous reset so Vivado can pack
    // the multiplier output registers into DSP48E1 blocks.
    reg        [127:0] interp_p00;
    reg        [127:0] interp_p01;
    reg        [127:0] interp_p10;
    reg        [127:0] interp_p11;
    reg        [ 16:0] interp_w00;
    reg        [ 16:0] interp_w01;
    reg        [ 16:0] interp_w10;
    reg        [ 16:0] interp_w11;
    reg signed [ 25:0] interp_mul00 [0:15];
    reg signed [ 25:0] interp_mul01 [0:15];
    reg signed [ 25:0] interp_mul10 [0:15];
    reg signed [ 25:0] interp_mul11 [0:15];
    reg signed [ 26:0] interp_pair0 [0:15];
    reg signed [ 26:0] interp_pair1 [0:15];
    reg signed [ 27:0] interp_acc   [0:15];
    reg                interp_negative [0:15];
    reg        [ 27:0] interp_magnitude [0:15];
    reg        [ 12:0] interp_round_mag [0:15];
    reg signed [ 12:0] interp_rounded [0:15];

    wire [31:0] offset_p01_w;
    wire [31:0] offset_p10_w;
    wire [31:0] offset_p11_w;
    wire [15:0] plane_size_product_w;
    wire [23:0] z_base_offset_product_w;
    wire [15:0] source_row_offset_product_w;
    wire [3:0]  final_cgroup_w;
    wire [10:0] final_group_w;
    wire [18:0] final_group_x_product_w;
    wire [26:0] final_xy_product_w;
    wire [31:0] final_half_addr_w;
    wire [31:0] rmw_aligned_addr_w;
    wire        rmw_upper_half_w;
    wire [255:0] rmw_payload_w;

    assign plane_size_product_w = bev_x * bev_y;
    assign z_base_offset_product_w = cur_z[7:0] * plane_size_r;
    assign source_row_offset_product_w = v_int_r[7:0] * bev_x;
    assign offset_p01_w = base_offset_r + 32'd1;
    assign offset_p10_w = base_offset_r + {24'd0, bev_x};
    assign offset_p11_w = base_offset_r + {24'd0, bev_x} + 32'd1;
    assign final_cgroup_w = {1'b0, temporal_idx, 1'b0} + {3'd0, rmw_half};
    assign final_group_w = {cur_z[7:0], 3'b0} + {7'd0, final_cgroup_w};
    assign final_group_x_product_w = final_group_w * bev_x;
    assign final_xy_product_w = final_group_x_r * bev_y;
    assign final_half_addr_w = concat_base_addr + {final_xy_r, 5'b0};
    assign rmw_aligned_addr_w = {final_half_addr_w[31:6], 6'b0};
    assign rmw_upper_half_w = final_half_addr_w[5];
    assign rmw_payload_w = (rmw_half == 1'b0) ? interp_result[255:0] :
                                                interp_result[511:256];

    assign rd_data_ready = (state == S_WAIT_P00) || (state == S_WAIT_P01) ||
                           (state == S_WAIT_P10) || (state == S_WAIT_P11) ||
                           (state == S_RMW_WAIT);

    function [7:0] saturate_i8;
        input signed [12:0] rounded;
        begin
            if (rounded > 13'sd127)
                saturate_i8 = 8'h7F;
            else if (rounded < -13'sd128)
                saturate_i8 = 8'h80;
            else
                saturate_i8 = rounded[7:0];
        end
    endfunction

    integer pipe_ch;
    integer ch;

    // Arithmetic-only pipeline. Keeping it in a reset-free clocked block
    // avoids the asynchronous-reset barrier on DSP input/output registers.
    always @(posedge clk) begin
        case (state)
            S_INTERP: begin
                case (interp_group)
                    2'd0: begin
                        interp_p00 <= p00_buf[127:0];
                        interp_p01 <= p01_buf[127:0];
                        interp_p10 <= p10_buf[127:0];
                        interp_p11 <= p11_buf[127:0];
                    end
                    2'd1: begin
                        interp_p00 <= p00_buf[255:128];
                        interp_p01 <= p01_buf[255:128];
                        interp_p10 <= p10_buf[255:128];
                        interp_p11 <= p11_buf[255:128];
                    end
                    2'd2: begin
                        interp_p00 <= p00_buf[383:256];
                        interp_p01 <= p01_buf[383:256];
                        interp_p10 <= p10_buf[383:256];
                        interp_p11 <= p11_buf[383:256];
                    end
                    default: begin
                        interp_p00 <= p00_buf[511:384];
                        interp_p01 <= p01_buf[511:384];
                        interp_p10 <= p10_buf[511:384];
                        interp_p11 <= p11_buf[511:384];
                    end
                endcase
                interp_w00 <= w00_r;
                interp_w01 <= w01_r;
                interp_w10 <= w10_r;
                interp_w11 <= w11_r;
            end

            S_INTERP_MUL: begin
                for (pipe_ch = 0; pipe_ch < 16; pipe_ch = pipe_ch + 1) begin
                    interp_mul00[pipe_ch] <= $signed(interp_p00[pipe_ch*8 +: 8]) *
                                        $signed({1'b0, interp_w00});
                    interp_mul01[pipe_ch] <= $signed(interp_p01[pipe_ch*8 +: 8]) *
                                        $signed({1'b0, interp_w01});
                    interp_mul10[pipe_ch] <= $signed(interp_p10[pipe_ch*8 +: 8]) *
                                        $signed({1'b0, interp_w10});
                    interp_mul11[pipe_ch] <= $signed(interp_p11[pipe_ch*8 +: 8]) *
                                        $signed({1'b0, interp_w11});
                end
            end

            S_INTERP_PAIR: begin
                for (pipe_ch = 0; pipe_ch < 16; pipe_ch = pipe_ch + 1) begin
                    interp_pair0[pipe_ch] <= interp_mul00[pipe_ch] + interp_mul01[pipe_ch];
                    interp_pair1[pipe_ch] <= interp_mul10[pipe_ch] + interp_mul11[pipe_ch];
                end
            end

            S_INTERP_ACC: begin
                for (pipe_ch = 0; pipe_ch < 16; pipe_ch = pipe_ch + 1)
                    interp_acc[pipe_ch] <= interp_pair0[pipe_ch] + interp_pair1[pipe_ch];
            end

            S_INTERP_MAG: begin
                for (pipe_ch = 0; pipe_ch < 16; pipe_ch = pipe_ch + 1) begin
                    interp_negative[pipe_ch] <= interp_acc[pipe_ch][27];
                    if (interp_acc[pipe_ch][27])
                        interp_magnitude[pipe_ch] <= -interp_acc[pipe_ch];
                    else
                        interp_magnitude[pipe_ch] <= interp_acc[pipe_ch];
                end
            end

            S_INTERP_ROUND: begin
                for (pipe_ch = 0; pipe_ch < 16; pipe_ch = pipe_ch + 1)
                    interp_round_mag[pipe_ch] <=
                        (interp_magnitude[pipe_ch] + 28'd32768) >> 16;
            end

            S_INTERP_SIGN: begin
                for (pipe_ch = 0; pipe_ch < 16; pipe_ch = pipe_ch + 1) begin
                    if (interp_negative[pipe_ch])
                        interp_rounded[pipe_ch] <=
                            -$signed({1'b0, interp_round_mag[pipe_ch]});
                    else
                        interp_rounded[pipe_ch] <=
                            $signed({1'b0, interp_round_mag[pipe_ch]});
                end
            end

            default: begin
            end
        endcase
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            engine_done   <= 1'b0;
            pix_cnt       <= 32'd0;
            cur_x         <= 16'd0;
            cur_y         <= 16'd0;
            cur_z         <= 16'd0;
            pipe_a00_lo   <= 24'd0;
            pipe_a01_lo   <= 24'd0;
            pipe_a10_lo   <= 24'd0;
            pipe_a11_lo   <= 24'd0;
            pipe_a00_hi   <= 25'sd0;
            pipe_a01_hi   <= 25'sd0;
            pipe_a10_hi   <= 25'sd0;
            pipe_a11_hi   <= 25'sd0;
            pipe_a00x     <= 48'sd0;
            pipe_a01y     <= 48'sd0;
            pipe_a10x     <= 48'sd0;
            pipe_a11y     <= 48'sd0;
            src_u_fp      <= 48'sd0;
            src_v_fp      <= 48'sd0;
            u_int_r       <= 16'sd0;
            v_int_r       <= 16'sd0;
            frac_x_r      <= 16'd0;
            frac_y_r      <= 16'd0;
            w00_r         <= 17'd0;
            w01_r         <= 17'd0;
            w10_r         <= 17'd0;
            w11_r         <= 17'd0;
            plane_size_r  <= 16'd0;
            z_base_offset_r <= 24'd0;
            source_row_offset_r <= 16'd0;
            base_zy_r     <= 32'd0;
            base_offset_r <= 32'd0;
            addr_p00_r    <= 32'd0;
            addr_p01_r    <= 32'd0;
            addr_p10_r    <= 32'd0;
            addr_p11_r    <= 32'd0;
            rmw_half      <= 1'b0;
            rmw_addr_r    <= 32'd0;
            rmw_upper_r   <= 1'b0;
            rmw_payload_r <= 256'd0;
            rmw_merge_data<= 512'd0;
            final_group_x_r<=19'd0;
            final_xy_r    <= 27'd0;
            p00_buf       <= 512'd0;
            p01_buf       <= 512'd0;
            p10_buf       <= 512'd0;
            p11_buf       <= 512'd0;
            interp_result <= 512'd0;
            interp_group  <= 2'd0;
            rd_addr       <= 32'd0;
            rd_req        <= 1'b0;
            wr_addr       <= 32'd0;
            wr_data       <= 512'd0;
            wr_req        <= 1'b0;
        end else begin
            engine_done <= 1'b0;

            case (state)
                S_IDLE: begin
                    rd_req  <= 1'b0;
                    wr_req  <= 1'b0;
                    pix_cnt <= 32'd0;
                    cur_x   <= 16'd0;
                    cur_y   <= 16'd0;
                    cur_z   <= 16'd0;
                    rmw_half<= 1'b0;
                    if (engine_start) begin
                        plane_size_r <= plane_size_product_w;
                        if (sa_size == 32'd0)
                            state <= S_DONE;
                        else
                            state <= S_CALC_COORD;
                    end
                end

                S_CALC_COORD: begin
                    pipe_a00_lo <= a00_lo_product_w;
                    pipe_a01_lo <= a01_lo_product_w;
                    pipe_a10_lo <= a10_lo_product_w;
                    pipe_a11_lo <= a11_lo_product_w;
                    pipe_a00_hi <= a00_hi_product_w;
                    pipe_a01_hi <= a01_hi_product_w;
                    pipe_a10_hi <= a10_hi_product_w;
                    pipe_a11_hi <= a11_hi_product_w;
                    state     <= S_CALC_COORD2;
                end

                S_CALC_COORD2: begin
                    pipe_a00x <= ($signed({{23{pipe_a00_hi[24]}}, pipe_a00_hi}) <<< 16) +
                                 $signed({24'd0, pipe_a00_lo});
                    pipe_a01y <= ($signed({{23{pipe_a01_hi[24]}}, pipe_a01_hi}) <<< 16) +
                                 $signed({24'd0, pipe_a01_lo});
                    pipe_a10x <= ($signed({{23{pipe_a10_hi[24]}}, pipe_a10_hi}) <<< 16) +
                                 $signed({24'd0, pipe_a10_lo});
                    pipe_a11y <= ($signed({{23{pipe_a11_hi[24]}}, pipe_a11_hi}) <<< 16) +
                                 $signed({24'd0, pipe_a11_lo});
                    state <= S_CALC_COORD3;
                end

                S_CALC_COORD3: begin
                    src_u_fp <= pipe_a00x + pipe_a01y +
                                $signed({{16{xform_a02[31]}}, xform_a02});
                    src_v_fp <= pipe_a10x + pipe_a11y +
                                $signed({{16{xform_a12[31]}}, xform_a12});
                    state    <= S_CHECK_BOUND;
                end

                S_CHECK_BOUND: begin
                    u_int_r    <= src_u_int_w;
                    v_int_r    <= src_v_int_w;
                    frac_x_r   <= src_u_frac_w;
                    frac_y_r   <= src_v_frac_w;

                    if (src_in_bounds_w) begin
                        state <= S_CALC_WEIGHT;
                    end else begin
                        interp_result <= 512'd0;
                        rmw_half      <= 1'b0;
                        state         <= S_RMW_ADDR0;
                    end
                end

                S_CALC_WEIGHT: begin
                    w00_r <= (9'd256 - {1'b0, frac_x_r[15:8]}) *
                             (9'd256 - {1'b0, frac_y_r[15:8]});
                    w01_r <= {1'b0, frac_x_r[15:8]} *
                             (9'd256 - {1'b0, frac_y_r[15:8]});
                    w10_r <= (9'd256 - {1'b0, frac_x_r[15:8]}) *
                             {1'b0, frac_y_r[15:8]};
                    w11_r <= {1'b0, frac_x_r[15:8]} *
                             {1'b0, frac_y_r[15:8]};

                    z_base_offset_r <= z_base_offset_product_w;
                    source_row_offset_r <= source_row_offset_product_w;
                    state <= S_CALC_BASE0;
                end

                S_CALC_BASE0: begin
                    base_zy_r <= {8'd0, z_base_offset_r} +
                                 {16'd0, source_row_offset_r};
                    state <= S_CALC_BASE1;
                end

                S_CALC_BASE1: begin
                    base_offset_r <= base_zy_r + {24'd0, u_int_r[7:0]};
                    state <= S_CALC_ADDR;
                end

                S_CALC_ADDR: begin
                    addr_p00_r <= sa_src_addr + {base_offset_r[25:0], 6'b0};
                    addr_p01_r <= sa_src_addr + {offset_p01_w[25:0], 6'b0};
                    addr_p10_r <= sa_src_addr + {offset_p10_w[25:0], 6'b0};
                    addr_p11_r <= sa_src_addr + {offset_p11_w[25:0], 6'b0};
                    state      <= S_RD_P00;
                end

                S_RD_P00: begin
                    rd_addr <= addr_p00_r;
                    rd_req  <= 1'b1;
                    if (rd_grant) begin
                        rd_req <= 1'b0;
                        state  <= S_WAIT_P00;
                    end
                end

                S_WAIT_P00: begin
                    if (rd_data_valid) begin
                        p00_buf <= rd_data;
                        state   <= S_RD_P01;
                    end
                end

                S_RD_P01: begin
                    rd_addr <= addr_p01_r;
                    rd_req  <= 1'b1;
                    if (rd_grant) begin
                        rd_req <= 1'b0;
                        state  <= S_WAIT_P01;
                    end
                end

                S_WAIT_P01: begin
                    if (rd_data_valid) begin
                        p01_buf <= rd_data;
                        state   <= S_RD_P10;
                    end
                end

                S_RD_P10: begin
                    rd_addr <= addr_p10_r;
                    rd_req  <= 1'b1;
                    if (rd_grant) begin
                        rd_req <= 1'b0;
                        state  <= S_WAIT_P10;
                    end
                end

                S_WAIT_P10: begin
                    if (rd_data_valid) begin
                        p10_buf <= rd_data;
                        state   <= S_RD_P11;
                    end
                end

                S_RD_P11: begin
                    rd_addr <= addr_p11_r;
                    rd_req  <= 1'b1;
                    if (rd_grant) begin
                        rd_req <= 1'b0;
                        state  <= S_WAIT_P11;
                    end
                end

                S_WAIT_P11: begin
                    if (rd_data_valid) begin
                        p11_buf      <= rd_data;
                        interp_group <= 2'd0;
                        state        <= S_INTERP;
                    end
                end

                S_INTERP: begin
                    state <= S_INTERP_MUL;
                end

                S_INTERP_MUL: begin
                    state <= S_INTERP_PAIR;
                end

                S_INTERP_PAIR: begin
                    state <= S_INTERP_ACC;
                end

                S_INTERP_ACC: begin
                    state <= S_INTERP_MAG;
                end

                S_INTERP_MAG: begin
                    state <= S_INTERP_ROUND;
                end

                S_INTERP_ROUND: begin
                    state <= S_INTERP_SIGN;
                end

                S_INTERP_SIGN: begin
                    state <= S_INTERP_STORE;
                end

                S_INTERP_STORE: begin
                    case (interp_group)
                        2'd0: begin
                            for (ch = 0; ch < 16; ch = ch + 1)
                                interp_result[ch*8 +: 8] <=
                                    saturate_i8(interp_rounded[ch]);
                        end
                        2'd1: begin
                            for (ch = 0; ch < 16; ch = ch + 1)
                                interp_result[128 + ch*8 +: 8] <=
                                    saturate_i8(interp_rounded[ch]);
                        end
                        2'd2: begin
                            for (ch = 0; ch < 16; ch = ch + 1)
                                interp_result[256 + ch*8 +: 8] <=
                                    saturate_i8(interp_rounded[ch]);
                        end
                        default: begin
                            for (ch = 0; ch < 16; ch = ch + 1)
                                interp_result[384 + ch*8 +: 8] <=
                                    saturate_i8(interp_rounded[ch]);
                        end
                    endcase

                    if (interp_group == 2'd3) begin
                        rmw_half <= 1'b0;
                        state    <= S_RMW_ADDR0;
                    end else begin
                        interp_group <= interp_group + 1'b1;
                        state        <= S_INTERP;
                    end
                end

                S_RMW_RD: begin
                    rd_addr       <= rmw_aligned_addr_w;
                    rmw_addr_r    <= rmw_aligned_addr_w;
                    rmw_upper_r   <= rmw_upper_half_w;
                    rmw_payload_r <= rmw_payload_w;
                    rd_req        <= 1'b1;
                    if (rd_grant) begin
                        rd_req <= 1'b0;
                        state  <= S_RMW_WAIT;
                    end
                end

                S_RMW_ADDR0: begin
                    final_group_x_r <= final_group_x_product_w +
                                       {11'd0, cur_x[7:0]};
                    state <= S_RMW_ADDR1;
                end

                S_RMW_ADDR1: begin
                    final_xy_r <= final_xy_product_w + {19'd0, cur_y[7:0]};
                    state      <= S_RMW_RD;
                end

                S_RMW_WAIT: begin
                    if (rd_data_valid) begin
                        if (rmw_upper_r)
                            rmw_merge_data <= {rmw_payload_r, rd_data[255:0]};
                        else
                            rmw_merge_data <= {rd_data[511:256], rmw_payload_r};
                        state <= S_RMW_WR;
                    end
                end

                S_RMW_WR: begin
                    wr_addr <= rmw_addr_r;
                    wr_data <= rmw_merge_data;
                    wr_req  <= 1'b1;
                    if (wr_grant) begin
                        wr_req <= 1'b0;
                        if (rmw_half == 1'b0) begin
                            rmw_half <= 1'b1;
                            state    <= S_RMW_ADDR0;
                        end else begin
                            state <= S_NEXT_PIX;
                        end
                    end
                end

                S_NEXT_PIX: begin
                    if (pix_cnt + 1'b1 >= sa_size) begin
                        state <= S_DONE;
                    end else begin
                        pix_cnt <= pix_cnt + 1'b1;
                        if (cur_x == {8'd0, bev_x} - 16'd1) begin
                            cur_x <= 16'd0;
                            if (cur_y == {8'd0, bev_y} - 16'd1) begin
                                cur_y <= 16'd0;
                                cur_z <= cur_z + 1'b1;
                            end else begin
                                cur_y <= cur_y + 1'b1;
                            end
                        end else begin
                            cur_x <= cur_x + 1'b1;
                        end
                        state <= S_CALC_COORD;
                    end
                end

                S_DONE: begin
                    engine_done <= 1'b1;
                    state       <= S_IDLE;
                end

                default: begin
                    state  <= S_IDLE;
                    rd_req <= 1'b0;
                    wr_req <= 1'b0;
                end
            endcase
        end
    end

    wire unused_bev_z;
    assign unused_bev_z = |bev_z;

endmodule
