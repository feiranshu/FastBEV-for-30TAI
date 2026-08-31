//====================================================================
// fp32_int8_quant.v
//--------------------------------------------------------------------
// IEEE-754 fp32 -> int8 symmetric quantizer.
//
// Quantization (effective formula):
//   int8 = round(fp32 / 0.06905783)
//
//   effective scale = 0.06905783
//   scale is NOT 256 * 0.06905783.
//   256 means channel count / repeated-channel scale, NOT a numeric scale
//   amplification. The previous SHIFT_BASE=164 was equivalent to dividing by
//   (256 * 0.06905783) because it added an extra >>8 (i.e. /256) on the shift
//   path. SHIFT_BASE is now 156 to remove that spurious 256x factor.
//
//   Datapath identity (constant multiply + arithmetic shift):
//     |fp32| = sig * 2^(exp-150)                        , sig = {1,mantissa}
//     s0_prod = sig * 927                               , 927 ~= (1/scale)*2^6
//     result  = round( s0_prod >> (SHIFT_BASE - exp) )
//   Solving 2^(SHIFT_BASE-150) = 927 * 0.06905783 = 64.017 -> SHIFT_BASE = 156.
//   (927/64 = 14.4844 approximates 1/0.06905783 = 14.4816, ~0.02% const error.)
//
// Saturation (post-quant clamp, NOT part of the scale):
//   positive ->  127
//   negative -> -128
// NaN and tiny values are mapped to 0.
//
// Latency:
//   in_valid -> out_valid: 3 cycles.
//====================================================================
`timescale 1ns/1ps

module fp32_int8_quant #(
    // SHIFT_BASE=156 implements int8 = round(fp32 / 0.06905783).
    // (Old value 164 wrongly implemented round(fp32 / (256 * 0.06905783)).)
    parameter integer SHIFT_BASE = 156
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    input  wire [31:0] src,
    output reg         out_valid,
    output reg  signed [7:0] dst
);

    localparam integer ZERO_E = SHIFT_BASE - 40;
    localparam integer SAT_E  = SHIFT_BASE;

    wire        s0_sign = src[31];
    wire [7:0]  s0_exp  = src[30:23];
    wire [22:0] s0_man  = src[22:0];
    wire [23:0] s0_sig  = {1'b1, s0_man};

    wire [39:0] sig_ext = {16'b0, s0_sig};
    wire [39:0] s0_prod = (sig_ext << 10) - (sig_ext << 7)
                        + (sig_ext << 5)  -  sig_ext;

    wire s0_is_nan  = (s0_exp == 8'hFF) & (|s0_man);
    wire s0_is_sat  = (s0_exp >= SAT_E[7:0]) & ~s0_is_nan;
    wire s0_is_zero = (s0_exp <= ZERO_E[7:0]) | s0_is_nan;

    reg        s1_sign, s1_is_sat, s1_is_zero, s1_v;
    reg [7:0]  s1_exp;
    reg [39:0] s1_prod;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_sign <= 1'b0; s1_is_sat <= 1'b0; s1_is_zero <= 1'b0; s1_v <= 1'b0;
            s1_exp  <= 8'd0; s1_prod   <= 40'd0;
        end else begin
            s1_sign    <= s0_sign;
            s1_exp     <= s0_exp;
            s1_prod    <= s0_prod;
            s1_is_sat  <= s0_is_sat;
            s1_is_zero <= s0_is_zero;
            s1_v       <= in_valid;
        end
    end

    wire [7:0]  s1_shamt   = SHIFT_BASE[7:0] - s1_exp;
    wire [39:0] s1_half    = (40'd1 << (s1_shamt - 8'd1));
    wire [39:0] s1_rounded = s1_prod + s1_half;
    wire [39:0] s1_shifted = s1_rounded >> s1_shamt;
    wire [8:0]  s1_mag     = (s1_shifted > 40'd128) ? 9'd128 : s1_shifted[8:0];

    reg        s2_sign, s2_is_sat, s2_is_zero, s2_v;
    reg [8:0]  s2_mag;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2_sign <= 1'b0; s2_is_sat <= 1'b0; s2_is_zero <= 1'b0; s2_v <= 1'b0;
            s2_mag  <= 9'd0;
        end else begin
            s2_sign    <= s1_sign;
            s2_is_sat  <= s1_is_sat;
            s2_is_zero <= s1_is_zero;
            s2_mag     <= s1_mag;
            s2_v       <= s1_v;
        end
    end

    reg signed [8:0] cand;
    always @(*) begin
        if (s2_is_zero)
            cand = 9'sd0;
        else if (s2_is_sat)
            cand = s2_sign ? -9'sd128 : 9'sd127;
        else if (s2_sign)
            cand = -$signed({1'b0, s2_mag});
        else
            cand = (s2_mag > 9'd127) ? 9'sd127 : $signed({1'b0, s2_mag});
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dst <= 8'sd0;
            out_valid <= 1'b0;
        end else begin
            dst <= cand[7:0];
            out_valid <= s2_v;
        end
    end

endmodule

