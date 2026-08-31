`timescale 1ns/1ps

module tb_dma_arbiter_stage_ready;
    reg clk=0,rst_n=0;
    always #2.5 clk=~clk;

    reg [1:0] stage=0;
    reg qrr=0,qwr=0,lrr=0,lwr=0,srr=0,swr=0;
    reg arready=0,awready=0,rvalid=0;
    wire qrg,qwg,lrg,lwg,srg,swg;
    wire [31:0] araddr,awaddr;
    wire arvalid,awvalid,rready;
    wire [511:0] awdata;

    dma_arbiter_stage dut(
        .clk(clk),.rst_n(rst_n),.active_stage(stage),
        .quant_rd_addr(32'h1000),.quant_rd_req(qrr),.quant_rd_grant(qrg),
        .quant_rd_data(),.quant_rd_data_valid(),.quant_rd_data_ready(1'b1),
        .quant_wr_addr(32'h1100),.quant_wr_data(512'h11),
        .quant_wr_req(qwr),.quant_wr_grant(qwg),
        .lut_rd_addr(32'h2000),.lut_rd_req(lrr),.lut_rd_grant(lrg),
        .lut_rd_data(),.lut_rd_data_valid(),.lut_rd_data_ready(1'b1),
        .lut_wr_addr(32'h2200),.lut_wr_data(512'h22),
        .lut_wr_req(lwr),.lut_wr_grant(lwg),
        .sa_rd_addr(32'h3000),.sa_rd_req(srr),.sa_rd_grant(srg),
        .sa_rd_data(),.sa_rd_data_valid(),.sa_rd_data_ready(1'b1),
        .sa_wr_addr(32'h3300),.sa_wr_data(512'h33),
        .sa_wr_req(swr),.sa_wr_grant(swg),
        .ddr_araddr(araddr),.ddr_arvalid(arvalid),.ddr_arready(arready),
        .ddr_rdata(512'h5a),.ddr_rvalid(rvalid),.ddr_rready(rready),
        .ddr_awaddr(awaddr),.ddr_awdata(awdata),
        .ddr_awvalid(awvalid),.ddr_awready(awready)
    );

    initial begin
        repeat(2) @(posedge clk);
        @(negedge clk);
        rst_n=1;stage=2'b11;arready=1;awready=1;
        #1;
        if(!srg||!swg||arvalid||awvalid) begin
            $display("FAIL: reset/local ready semantics");$finish;
        end

        // Read request is registered; write request retains direct external
        // handshake semantics so the final completion cannot be early.
        srr=1;swr=1;#1;
        if(arvalid||!awvalid||awaddr!=32'h3300||awdata!=512'h33) begin
            $display("FAIL: read was not isolated or write was not direct");
            $finish;
        end
        @(posedge clk);#1;srr=0;swr=0;
        if(!arvalid||araddr!=32'h3000) begin
            $display("FAIL: queued SA read was not forwarded");$finish;
        end
        @(posedge clk);#1;
        if(arvalid) begin
            $display("FAIL: accepted SA read did not leave queue");$finish;
        end

        // External read backpressure must not feed combinationally into grant
        // while the local queue has room.  Write grant must still track ready.
        stage=2'b01;arready=0;awready=0;qrr=1;qwr=1;#1;
        if(!qrg||qwg||srg||swg||!awvalid||awaddr!=32'h1100) begin
            $display("FAIL: owner/read isolation/write ready semantics");
            $finish;
        end
        @(posedge clk);#1;qrr=0;
        if(!arvalid||araddr!=32'h1000||!awvalid) begin
            $display("FAIL: stalled requests were not held");$finish;
        end
        arready=1;awready=1;#1;
        if(!qwg) begin
            $display("FAIL: write grant did not follow external ready");$finish;
        end
        @(posedge clk);#1;qwr=0;
        if(arvalid) begin
            $display("FAIL: stalled read did not drain");$finish;
        end

        rvalid=1;#1;
        if(!rready) begin $display("FAIL: read response ready");$finish;end
        $display("PASS: registered read queue, write completion and stage isolation");
        $finish;
    end
endmodule
