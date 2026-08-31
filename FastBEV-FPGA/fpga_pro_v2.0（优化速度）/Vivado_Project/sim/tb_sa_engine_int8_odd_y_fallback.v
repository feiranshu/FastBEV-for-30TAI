`timescale 1ns/1ps
module tb_sa_engine_int8_odd_y_fallback;
    reg clk=0,rst_n=0,start=0;
    always #5 clk=~clk;
    wire done,rd_req,rd_ready,wr_req;
    wire [31:0] rd_addr,wr_addr;
    wire [511:0] wr_data;
    reg [7:0] lfsr=8'h5a;
    wire rd_grant=rd_req&&lfsr[0];
    wire wr_grant=wr_req&&lfsr[1];
    reg [2:0] rv=0;
    reg rd_valid=0;
    reg [511:0] rd_data={512{1'b1}};
    integer reads=0,writes=0,cycles=0;

    localparam SRC=32'h0010_0000, DST=32'h0100_0000;
    sa_engine_int8 #(.INTERP_FRAC_BITS(6),.MAX_OUTSTANDING(4)) dut(
        .engine_start(start),.engine_done(done),.sa_src_addr(SRC),
        .concat_base_addr(DST),.sa_size(6),.bev_x(8'd2),.bev_y(8'd3),
        .bev_z(8'd1),.temporal_idx(2'd0),.xform_a00(0),.xform_a01(0),
        .xform_a02(32'hffff_0000),.xform_a10(0),.xform_a11(0),
        .xform_a12(32'hffff_0000),.rd_addr(rd_addr),.rd_req(rd_req),
        .rd_grant(rd_grant),.rd_data(rd_data),.rd_data_valid(rd_valid),
        .rd_data_ready(rd_ready),.wr_addr(wr_addr),.wr_data(wr_data),
        .wr_req(wr_req),.wr_grant(wr_grant),.clk(clk),.rst_n(rst_n));

    always @(posedge clk) begin
        lfsr<={lfsr[6:0],lfsr[7]^lfsr[5]^lfsr[4]^lfsr[3]};
        rv<={rv[1:0],rd_req&&rd_grant};
        rd_valid<=rv[2];
        if(rd_req&&rd_grant) reads<=reads+1;
        if(wr_req&&wr_grant) begin
            writes<=writes+1;
            if(wr_addr[5:0]!==0) begin
                $display("FAIL unaligned write %h",wr_addr);$finish;
            end
            if(!((wr_data[255:0]===256'd0&&wr_data[511:256]==={256{1'b1}})||
                 (wr_data[511:256]===256'd0&&wr_data[255:0]==={256{1'b1}}))) begin
                $display("FAIL RMW merge write=%0d data=%h",writes,wr_data);$finish;
            end
        end
        cycles<=cycles+1;
        if(cycles>3000) begin $display("FAIL timeout");$finish;end
    end

    initial begin
        repeat(4) @(posedge clk);rst_n=1;
        @(posedge clk);start=1;@(posedge clk);start=0;
        wait(done);@(posedge clk);
        if(reads!==12||writes!==12) begin
            $display("FAIL transactions read=%0d write=%0d",reads,writes);$finish;
        end
        if(dut.perf_read_requests!==12||dut.perf_write_requests!==12) begin
            $display("FAIL perf read=%0d write=%0d",dut.perf_read_requests,
                     dut.perf_write_requests);$finish;
        end
        $display("PASS tb_sa_engine_int8_odd_y_fallback cycles=%0d",
                 dut.perf_cycles);
        $finish;
    end
endmodule
