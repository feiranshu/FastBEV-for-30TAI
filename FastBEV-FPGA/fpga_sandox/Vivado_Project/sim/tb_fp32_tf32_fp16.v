`timescale 1ns/1ps

module tb_fp32_tf32_fp16;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    wire engine_done;
    wire [31:0] rd_addr, wr_addr;
    wire rd_req, rd_data_ready, wr_req;
    wire [511:0] wr_data;

    lut_engine dut (
        .engine_start(1'b0), .engine_done(engine_done),
        .lut_base_addr(32'd0), .lut_size(32'd0),
        .feat2d_base_addr(32'd0), .feat3d_wr_addr(32'd0),
        .img_w(12'd160), .img_h(12'd120), .cameras(8'd6),
        .bev_x(8'd200), .bev_y(8'd200), .bev_z(8'd4),
        .rd_addr(rd_addr), .rd_req(rd_req), .rd_grant(1'b0),
        .rd_data(512'd0), .rd_data_valid(1'b0),
        .rd_data_ready(rd_data_ready),
        .wr_addr(wr_addr), .wr_data(wr_data), .wr_req(wr_req),
        .wr_grant(1'b0), .clk(clk), .rst_n(rst_n)
    );

    always #2.5 clk = ~clk;

    task check;
        input [31:0] source;
        input [15:0] expected;
        reg [15:0] actual;
        begin
            actual = dut.fp32_prefix_to_fp16(source);
            if (actual !== expected) begin
                $display("TEST_FAIL: source=%08x expected=%04x actual=%04x",
                         source, expected, actual);
                $finish;
            end
        end
    endtask

    initial begin
        check(32'h00000000, 16'h0000); // +0
        check(32'h80000000, 16'h8000); // -0
        check(32'h3f800000, 16'h3c00); // +1
        check(32'hc0000000, 16'hc000); // -2
        check(32'h477fe000, 16'h7bff); // max finite FP16
        check(32'h47800000, 16'h7c00); // overflow
        check(32'h7f800000, 16'h7c00); // +inf
        check(32'hff800000, 16'hfc00); // -inf
        check(32'h7fc00000, 16'h7e00); // qNaN
        check(32'h38800000, 16'h0400); // minimum normal FP16
        check(32'h387fc000, 16'h03ff); // maximum subnormal FP16
        check(32'h33800000, 16'h0001); // minimum subnormal FP16
        check(32'h33000000, 16'h0000); // exact half-way, ties to even zero
        check(32'h33002000, 16'h0001); // above the subnormal half-way point
        check(32'h3f801000, 16'h3c00); // FP32->TF32 tie, retained LSB even
        check(32'h3f803000, 16'h3c02); // FP32->TF32 tie, retained LSB odd
        $display("TEST_PASS: tb_fp32_tf32_fp16");
        $finish;
    end
endmodule
