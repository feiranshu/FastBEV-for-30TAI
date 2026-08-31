//==============================================================================
// File Name   : fp32_int8_quant.v
// Description : Pipelined IEEE-754 FP32 to signed INT8 quantizer.
//
// Quantization:
//   int8 = round(fp32 / 0.06905783), saturated to [-128, 127]
//
// The reciprocal is approximated by 927/64 = 14.484375. Multiplication by
// 927 is implemented as two registered shift/subtract partial products:
//   927 = (2^10 - 2^7) + (2^5 - 2^0)
//
// Latency is seven clocks from in_valid to out_valid. The variable right shift
// is split into 0/16/32, 0/8, and 0..7 stages to meet a 200 MHz target.
//==============================================================================
`timescale 1ns/1ps

module fp32_int8_quant #(
    // 156 = 127 FP32 bias + 23 mantissa bits + 6 reciprocal fractional bits.
    parameter integer SHIFT_BASE = 156
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire              in_valid,
    input  wire [31:0]       src,
    output reg               out_valid,
    output reg signed [7:0]  dst
);

    localparam integer ZERO_E = SHIFT_BASE - 40;
    localparam integer SAT_E  = SHIFT_BASE;
    localparam [7:0] ZERO_E_U8 = ZERO_E;
    localparam [7:0] SAT_E_U8  = SAT_E;

    wire        s0_sign;
    wire [7:0]  s0_exp;
    wire [22:0] s0_man;
    wire [23:0] s0_sig;
    wire [39:0] s0_sig_ext;
    wire [39:0] s0_partial_hi;
    wire [39:0] s0_partial_lo;
    wire        s0_is_nan;
    wire        s0_is_sat;
    wire        s0_is_zero;

    assign s0_sign = src[31];
    assign s0_exp  = src[30:23];
    assign s0_man  = src[22:0];
    assign s0_sig  = {1'b1, s0_man};
    assign s0_sig_ext = {16'd0, s0_sig};
    assign s0_partial_hi = (s0_sig_ext << 10) - (s0_sig_ext << 7);
    assign s0_partial_lo = (s0_sig_ext << 5) - s0_sig_ext;
    assign s0_is_nan  = (s0_exp == 8'hff) && (|s0_man);
    assign s0_is_sat  = (s0_exp >= SAT_E_U8) && !s0_is_nan;
    assign s0_is_zero = (s0_exp <= ZERO_E_U8) || s0_is_nan;

    // Stage 1: register the two low-Hamming-weight reciprocal products.
    reg        s1_sign, s1_is_sat, s1_is_zero, s1_v;
    reg [7:0]  s1_exp;
    reg [39:0] s1_partial_hi;
    reg [39:0] s1_partial_lo;
    always @(posedge clk) begin
        if (!rst_n) begin
            s1_sign <= 1'b0;
            s1_is_sat <= 1'b0;
            s1_is_zero <= 1'b0;
            s1_v <= 1'b0;
            s1_exp <= 8'd0;
            s1_partial_hi <= 40'd0;
            s1_partial_lo <= 40'd0;
        end else begin
            s1_sign <= s0_sign;
            s1_is_sat <= s0_is_sat;
            s1_is_zero <= s0_is_zero;
            s1_v <= in_valid;
            s1_exp <= s0_exp;
            s1_partial_hi <= s0_partial_hi;
            s1_partial_lo <= s0_partial_lo;
        end
    end

    // Stage 2: add the partial products to form sig * 927.
    reg        s2_sign, s2_is_sat, s2_is_zero, s2_v;
    reg [7:0]  s2_exp;
    reg [39:0] s2_prod;
    always @(posedge clk) begin
        if (!rst_n) begin
            s2_sign <= 1'b0;
            s2_is_sat <= 1'b0;
            s2_is_zero <= 1'b0;
            s2_v <= 1'b0;
            s2_exp <= 8'd0;
            s2_prod <= 40'd0;
        end else begin
            s2_sign <= s1_sign;
            s2_is_sat <= s1_is_sat;
            s2_is_zero <= s1_is_zero;
            s2_v <= s1_v;
            s2_exp <= s1_exp;
            s2_prod <= s1_partial_hi + s1_partial_lo;
        end
    end

    // Stage 3: coarse 0/16/32 right shift. Appending zero preserves the guard
    // bit: ({prod,1'b0} >> shamt)[0] equals prod[shamt-1].
    wire [7:0]  s2_shamt_raw;
    wire [5:0]  s2_shamt;
    wire [40:0] s2_prod_guard;
    wire [40:0] s2_coarse_shifted;
    assign s2_shamt_raw = SHIFT_BASE[7:0] - s2_exp;
    assign s2_shamt = s2_is_sat  ? 6'd1 :
                      s2_is_zero ? 6'd39 : s2_shamt_raw[5:0];
    assign s2_prod_guard = {s2_prod, 1'b0};
    assign s2_coarse_shifted =
        (s2_shamt[5:4] == 2'd0) ? s2_prod_guard :
        (s2_shamt[5:4] == 2'd1) ? (s2_prod_guard >> 16) :
                                  (s2_prod_guard >> 32);

    reg         s3_sign, s3_is_sat, s3_is_zero, s3_v;
    reg [3:0]   s3_fine_shamt;
    reg [40:0]  s3_coarse;
    always @(posedge clk) begin
        if (!rst_n) begin
            s3_sign <= 1'b0;
            s3_is_sat <= 1'b0;
            s3_is_zero <= 1'b0;
            s3_v <= 1'b0;
            s3_fine_shamt <= 4'd0;
            s3_coarse <= 41'd0;
        end else begin
            s3_sign <= s2_sign;
            s3_is_sat <= s2_is_sat;
            s3_is_zero <= s2_is_zero;
            s3_v <= s2_v;
            s3_fine_shamt <= s2_shamt[3:0];
            s3_coarse <= s2_coarse_shifted;
        end
    end

    // Stage 4: optional shift by eight.
    reg         s4_sign, s4_is_sat, s4_is_zero, s4_v;
    reg [2:0]   s4_fine_shamt;
    reg [40:0]  s4_mid;
    always @(posedge clk) begin
        if (!rst_n) begin
            s4_sign <= 1'b0;
            s4_is_sat <= 1'b0;
            s4_is_zero <= 1'b0;
            s4_v <= 1'b0;
            s4_fine_shamt <= 3'd0;
            s4_mid <= 41'd0;
        end else begin
            s4_sign <= s3_sign;
            s4_is_sat <= s3_is_sat;
            s4_is_zero <= s3_is_zero;
            s4_v <= s3_v;
            s4_fine_shamt <= s3_fine_shamt[2:0];
            s4_mid <= s3_fine_shamt[3] ? (s3_coarse >> 8) : s3_coarse;
        end
    end

    // Stage 5: final 0..7 shift.
    reg         s5_sign, s5_is_sat, s5_is_zero, s5_v;
    reg [40:0]  s5_qguard;
    always @(posedge clk) begin
        if (!rst_n) begin
            s5_sign <= 1'b0;
            s5_is_sat <= 1'b0;
            s5_is_zero <= 1'b0;
            s5_v <= 1'b0;
            s5_qguard <= 41'd0;
        end else begin
            s5_sign <= s4_sign;
            s5_is_sat <= s4_is_sat;
            s5_is_zero <= s4_is_zero;
            s5_v <= s4_v;
            s5_qguard <= s4_mid >> s4_fine_shamt;
        end
    end

    // Stage 6: round half away from zero and clamp magnitude to 128.
    wire       s5_mag_overflow;
    wire [8:0] s5_mag_rounded;
    wire [8:0] s5_mag;
    assign s5_mag_overflow = |s5_qguard[40:8];
    assign s5_mag_rounded = {1'b0, s5_qguard[7:1]} +
                            {8'd0, s5_qguard[0]};
    assign s5_mag = s5_mag_overflow ? 9'd128 : s5_mag_rounded;

    reg        s6_sign, s6_is_sat, s6_is_zero, s6_v;
    reg [8:0]  s6_mag;
    always @(posedge clk) begin
        if (!rst_n) begin
            s6_sign <= 1'b0;
            s6_is_sat <= 1'b0;
            s6_is_zero <= 1'b0;
            s6_v <= 1'b0;
            s6_mag <= 9'd0;
        end else begin
            s6_sign <= s5_sign;
            s6_is_sat <= s5_is_sat;
            s6_is_zero <= s5_is_zero;
            s6_v <= s5_v;
            s6_mag <= s5_mag;
        end
    end

    // Stage 7: apply sign and signed INT8 saturation.
    reg signed [8:0] candidate;
    always @(*) begin
        if (s6_is_zero)
            candidate = 9'sd0;
        else if (s6_is_sat)
            candidate = s6_sign ? -9'sd128 : 9'sd127;
        else if (s6_sign)
            candidate = -$signed({1'b0, s6_mag});
        else
            candidate = (s6_mag > 9'd127) ? 9'sd127 :
                        $signed({1'b0, s6_mag});
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            dst <= 8'sd0;
            out_valid <= 1'b0;
        end else begin
            dst <= candidate[7:0];
            out_valid <= s6_v;
        end
    end

endmodule
