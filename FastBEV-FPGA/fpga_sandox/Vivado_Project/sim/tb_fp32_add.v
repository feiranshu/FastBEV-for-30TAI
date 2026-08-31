`timescale 1ns/1ps

module tb_fp32_add;
    reg         clk = 1'b0;
    reg  [31:0] a = 32'd0;
    reg  [31:0] b = 32'd0;
    wire [31:0] result;

    always #2.5 clk = ~clk;

    fp32_add dut (
        .clk(clk),
        .a(a),
        .b(b),
        .result(result)
    );

    task check_add;
        input [31:0] lhs;
        input [31:0] rhs;
        input [31:0] expected;
        begin
            @(negedge clk);
            a = lhs;
            b = rhs;
            repeat (5) @(posedge clk);
            #1;
            if (result !== expected) begin
                $display("FP32_ADD_FAIL a=%h b=%h got=%h expected=%h",
                         lhs, rhs, result, expected);
                $fatal(1);
            end
        end
    endtask

    initial begin
        // Prime all five unreset pipeline stages.
        repeat (6) @(posedge clk);

        check_add(32'h3f800000, 32'h40000000, 32'h40400000); // 1 + 2 = 3
        check_add(32'h3fc00000, 32'h40100000, 32'h40700000); // 1.5 + 2.25 = 3.75
        check_add(32'h40b00000, 32'hc0100000, 32'h40500000); // 5.5 - 2.25 = 3.25
        check_add(32'hbf800000, 32'h3f800000, 32'h00000000); // -1 + 1 = 0
        check_add(32'h00000000, 32'h40600000, 32'h40600000); // 0 + 3.5 = 3.5
        check_add(32'hc0800000, 32'hc0000000, 32'hc0c00000); // -4 - 2 = -6

        $display("TEST_PASS: tb_fp32_add");
        $finish;
    end
endmodule
