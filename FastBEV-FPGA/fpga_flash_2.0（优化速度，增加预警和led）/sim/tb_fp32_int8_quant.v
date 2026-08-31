`timescale 1ns/1ps

module tb_fp32_int8_quant;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg in_valid = 1'b0;
    reg [31:0] src = 32'd0;
    wire out_valid;
    wire signed [7:0] dst;

    reg [7:0] expected [0:3];
    reg       expected_valid [0:3];
    integer i;
    integer checked;

    fp32_int8_quant dut (
        .clk(clk), .rst_n(rst_n), .in_valid(in_valid), .src(src),
        .out_valid(out_valid), .dst(dst)
    );

    always #2.5 clk = ~clk;

    function [7:0] quant_ref;
        input [31:0] value;
        reg sign;
        reg [7:0] exp;
        reg [22:0] man;
        reg [23:0] sig;
        reg [39:0] sig_ext;
        reg [39:0] prod;
        reg [7:0] shamt;
        reg [39:0] half;
        reg [39:0] shifted;
        reg [8:0] mag;
        reg signed [8:0] signed_value;
        begin
            sign = value[31];
            exp = value[30:23];
            man = value[22:0];
            sig = {1'b1, man};
            sig_ext = {16'd0, sig};
            prod = (sig_ext << 10) - (sig_ext << 7) +
                   (sig_ext << 5) - sig_ext;

            if (((exp == 8'hFF) && (|man)) || exp <= 8'd116) begin
                quant_ref = 8'h00;
            end else if (exp >= 8'd156) begin
                quant_ref = sign ? 8'h80 : 8'h7F;
            end else begin
                shamt = 8'd156 - exp;
                half = 40'd1 << (shamt - 8'd1);
                shifted = (prod + half) >> shamt;
                mag = (shifted > 40'd128) ? 9'd128 : shifted[8:0];
                if (sign)
                    signed_value = -$signed({1'b0, mag});
                else
                    signed_value = (mag > 9'd127) ? 9'sd127 : $signed({1'b0, mag});
                quant_ref = signed_value[7:0];
            end
        end
    endfunction

    task fail;
        input [8*120-1:0] message;
        begin
            $display("TEST_FAIL: %0s", message);
            $finish;
        end
    endtask

    task drive;
        input [31:0] value;
        begin
            @(negedge clk);
            src = value;
            in_valid = 1'b1;
        end
    endtask

    always @(posedge clk) begin
        expected[3] = expected[2];
        expected[2] = expected[1];
        expected[1] = expected[0];
        expected[0] = quant_ref(src);
        expected_valid[3] = expected_valid[2];
        expected_valid[2] = expected_valid[1];
        expected_valid[1] = expected_valid[0];
        expected_valid[0] = in_valid && rst_n;
        #1;
        if (out_valid !== expected_valid[3])
            fail("out_valid latency mismatch");
        if (out_valid) begin
            checked = checked + 1;
            if (dst !== expected[3]) begin
                $display("value=0x%08x expected=0x%02x actual=0x%02x",
                         src, expected[3], dst);
                fail("quantized value mismatch");
            end
        end
    end

    initial begin
        for (i = 0; i < 4; i = i + 1) begin
            expected[i] = 8'd0;
            expected_valid[i] = 1'b0;
        end
        checked = 0;
        repeat (4) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;

        drive(32'h00000000); // +0
        drive(32'h80000000); // -0
        drive(32'h3D8D6B5D); // approximately one quantization step
        drive(32'h3F800000); // +1
        drive(32'hBF800000); // -1
        drive(32'h7F800000); // +Inf saturates
        drive(32'hFF800000); // -Inf saturates
        drive(32'h7FC00001); // NaN -> zero
        drive(32'h00800000); // tiny -> zero

        for (i = 0; i < 4096; i = i + 1)
            drive($random);

        @(negedge clk);
        in_valid = 1'b0;
        repeat (6) @(posedge clk);
        if (checked != 4105)
            fail("unexpected number of checked samples");
        $display("TEST_PASS: tb_fp32_int8_quant checked=%0d", checked);
        $finish;
    end
endmodule
