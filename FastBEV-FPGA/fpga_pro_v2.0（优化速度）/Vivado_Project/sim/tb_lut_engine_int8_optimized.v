`timescale 1ns/1ps
module tb_lut_engine_int8_optimized;
    reg clk=0,rst_n=0,start=0;
    always #5 clk=~clk;
    wire done;
    wire [31:0] rd_addr,wr_addr;
    wire rd_req,rd_ready,wr_req;
    wire rd_grant,wr_grant;
    reg rd_valid=0;
    assign rd_grant=rd_req;
    assign wr_grant=wr_req;
    reg [511:0] rd_data=0;
    wire [511:0] wr_data;
    localparam LUT=32'h10000,DST=32'h80000;
    lut_engine_int8 dut(.engine_start(start),.engine_done(done),
        .lut_base_addr(LUT),.lut_size(16),.feat2d_base_addr(32'h40000),
        .dst_base_addr(DST),.dst_mode(1'b1),.img_w(12'd2),.img_h(12'd2),
        .bev_x(8'd2),.bev_y(8'd2),.rd_addr(rd_addr),.rd_req(rd_req),
        .rd_grant(rd_grant),.rd_data(rd_data),.rd_data_valid(rd_valid),
        .rd_data_ready(rd_ready),.wr_addr(wr_addr),.wr_data(wr_data),
        .wr_req(wr_req),.wr_grant(wr_grant),.clk(clk),.rst_n(rst_n));
    reg [3:0] rv=0;
    integer reads=0,writes=0,cycles=0,k;
    reg [31:0] waddr[0:31];
    function [511:0] invalid_lut;
        input dummy;
        integer n;
        begin
            invalid_lut=0;
            for(n=0;n<8;n=n+1)invalid_lut[n*64+:16]=16'hffff;
        end
    endfunction
    always @(posedge clk)begin
        cycles<=cycles+1;rv<={rv[2:0],rd_req};rd_valid<=rv[3];
        rd_data<=invalid_lut(1'b0);
        if(rd_req&&rd_grant)reads<=reads+1;
        if(wr_req&&wr_grant)begin
            waddr[writes]<=wr_addr;writes<=writes+1;
            if(wr_data!==0)begin $display("FAIL nonzero output");$finish;end
        end
        if(cycles>3000)begin $display("FAIL timeout");$finish;end
    end
    initial begin
        repeat(4)@(posedge clk);rst_n=1;@(posedge clk);start=1;
        @(posedge clk);start=0;wait(done);@(posedge clk);
        if(reads!==2||writes!==16)begin
            $display("FAIL reads=%0d writes=%0d",reads,writes);$finish;end
        if(waddr[0]!==DST||waddr[1]!==DST+32'h80)begin
            $display("FAIL addresses %h %h",waddr[0],waddr[1]);$finish;end
        $display("PASS tb_lut_engine_int8_optimized cycles=%0d",cycles);
        $finish;
    end
endmodule
