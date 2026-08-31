`timescale 1ns/1ps

module tb_bev_accel_top_cdc;
    reg clk = 1'b0;
    reg ra_clk = 1'b0;
    reg rst_n = 1'b0;
    reg ra_rst_n = 1'b0;
    reg [31:0] awaddr = 32'd0;
    reg [31:0] awdata = 32'd0;
    reg awvalid = 1'b0;
    wire awready;
    wire [31:0] araddr;
    wire arvalid;
    reg arready = 1'b0;
    reg [511:0] rdata = 512'd0;
    reg rvalid = 1'b0;
    wire rready;
    wire [31:0] wwaddr;
    wire [511:0] wwdata;
    wire wwvalid;
    reg wwready = 1'b0;
    wire [31:0] rrdata;
    wire rrvalid;
    reg rrready = 1'b1;
    wire rraddr_ready;
    reg [31:0] rraddr = 32'd0;
    reg rraddr_valid = 1'b0;
    wire reset_reg;
    integer timeout;

    bev_accel_top dut (
        .itf_ra_awaddr(awaddr), .itf_ra_awdata(awdata),
        .itf_ra_awvalid(awvalid), .itf_ra_awready(awready),
        .itf_ra_araddr(rraddr), .itf_ra_arvalid(rraddr_valid),
        .itf_ra_arready(rraddr_ready), .itf_ra_rdata(rrdata),
        .itf_ra_rvalid(rrvalid), .itf_ra_rready(rrready),
        .itf_awaddr(wwaddr), .itf_awdata(wwdata),
        .itf_awvalid(wwvalid), .itf_awready(wwready),
        .itf_araddr(araddr), .itf_arvalid(arvalid), .itf_arready(arready),
        .itf_rdata(rdata), .itf_rvalid(rvalid), .itf_rready(rready),
        .reset_reg(reset_reg), .clk(clk), .ra_clk(ra_clk),
        .rst_n(rst_n), .ra_rst_n(ra_rst_n)
    );

    always #2.5 clk = ~clk;
    always #3.5 ra_clk = ~ra_clk;

    task fail;
        input [8*160-1:0] message;
        begin $display("TEST_FAIL: %0s", message); $finish; end
    endtask

    task write_word;
        input [15:0] word_addr;
        input [31:0] value;
        begin
            @(negedge ra_clk);
            awaddr = {14'd0, word_addr, 2'b00};
            awdata = value;
            awvalid = 1'b1;
            @(negedge ra_clk);
            awvalid = 1'b0;
        end
    endtask

    initial begin
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        ra_rst_n = 1'b1;
        repeat (8) @(posedge ra_clk);
        if (dut.feat3d_wr_size !== 32'd20480000)
            fail("FP16 output-size default mismatch");
        if (dut.bev_x !== 8'd200 || dut.bev_y !== 8'd200 ||
            dut.bev_z !== 8'd4 || dut.bev_channels !== 8'd64)
            fail("BEV dimension defaults mismatch");
        if (dut.img_w !== 12'd160 || dut.img_h !== 12'd120 ||
            dut.cameras !== 8'd6)
            fail("NHWC input dimension defaults mismatch");
        write_word(16'h01, 32'h11000000);
        write_word(16'h02, 32'd4);
        write_word(16'h03, 32'h22000000);
        write_word(16'h04, 32'h33000000);
        write_word(16'h0A, 32'h060780A0); // {6,120,160}
        write_word(16'h00, 32'h00000003); // mode LUT + start

        timeout = 0;
        while (!dut.lut_start && timeout < 80) begin
            @(posedge clk); timeout = timeout + 1;
        end
        #1;
        if (timeout >= 80) fail("CDC-delayed LUT start missing");
        if (dut.active_mode_hp !== 2'b01 || dut.lut_size_hp !== 32'd4)
            fail("mode/config snapshot mismatch");
        if (dut.img_w_hp !== 12'd160 || dut.img_h_hp !== 12'd120 ||
            dut.cameras_hp !== 8'd6)
            fail("image dimension snapshot mismatch");
        if (dut.lut_base_addr_hp !== 32'h11000000 ||
            dut.feat2d_base_addr_hp !== 32'h22000000 ||
            dut.feat3d_wr_addr_hp !== 32'h33000000)
            fail("address snapshot mismatch");

        // Live GP-domain changes must not perturb the active HP run.
        write_word(16'h0A, 32'h01001001);
        repeat (5) @(posedge clk);
        if (dut.img_w_hp !== 12'd160 || dut.img_h_hp !== 12'd120 ||
            dut.cameras_hp !== 8'd6)
            fail("active configuration changed across clock domains");
        $display("TEST_PASS: tb_bev_accel_top_cdc");
        $finish;
    end
endmodule
