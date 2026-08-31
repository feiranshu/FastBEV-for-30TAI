`timescale 1ns/1ps
module tb_sa_engine_int8_optimized;
    reg clk=0,rst_n=0,start=0;
    always #5 clk=~clk;
    wire done;
    wire [31:0] rd_addr,wr_addr;
    wire rd_req,rd_ready,wr_req;
    wire rd_grant,wr_grant;
    reg rd_valid=0;
    // Stall the first cache miss for one cycle to exercise lookup-pipeline
    // backpressure; all later requests are accepted immediately.
    reg rd_grant_open=0;
    assign rd_grant=rd_req&&rd_grant_open;
    assign wr_grant=wr_req;
    reg [511:0] rd_data=0;
    wire [511:0] wr_data;

    localparam SRC=32'h0010_0000, DST=32'h0100_0000;
    sa_engine_int8 #(.INTERP_FRAC_BITS(6),.MAX_OUTSTANDING(4)) dut(
        .engine_start(start),.engine_done(done),.sa_src_addr(SRC),
        .concat_base_addr(DST),.sa_size(4),.bev_x(2),.bev_y(2),.bev_z(1),
        .temporal_idx(0),.xform_a00(32'h0001_0000),.xform_a01(0),
        .xform_a02(32'h0000_8000),.xform_a10(0),.xform_a11(32'h0001_0000),
        .xform_a12(32'h0000_8000),.rd_addr(rd_addr),.rd_req(rd_req),
        .rd_grant(rd_grant),.rd_data(rd_data),.rd_data_valid(rd_valid),
        .rd_data_ready(rd_ready),.wr_addr(wr_addr),.wr_data(wr_data),
        .wr_req(wr_req),.wr_grant(wr_grant),.clk(clk),.rst_n(rst_n));

    reg [4:0] rv=0;
    reg [31:0] ra0=0,ra1=0,ra2=0,ra3=0,ra4=0;
    integer reads=0,writes=0,inflight=0,max_inflight=0,cycles=0,i;
    reg [31:0] waddr[0:7];
    reg [511:0] wdata[0:7];

    function [511:0] source_data;
        input [31:0] addr;
        integer k;
        reg [7:0] value;
        begin
            case(addr)
                SRC:       value=8'h00;
                SRC+64:    value=8'h40;
                SRC+128:   value=8'hc0;
                default:   value=8'h7f;
            endcase
            for(k=0;k<64;k=k+1) source_data[k*8+:8]=value;
        end
    endfunction

    always @(posedge clk) begin
        cycles<=cycles+1;
        if(!rst_n) rd_grant_open<=0;
        else if(rd_req) rd_grant_open<=1;
        rv<={rv[3:0],rd_req&&rd_grant};
        ra4<=ra3;ra3<=ra2;ra2<=ra1;ra1<=ra0;ra0<=rd_addr;
        rd_valid<=rv[4];
        rd_data<=source_data(ra4);
        if(rd_req&&rd_grant) begin
            reads<=reads+1;
            if(inflight+1>max_inflight) max_inflight<=inflight+1;
        end
        case({rd_req&&rd_grant,rd_valid&&rd_ready})
            2'b10:inflight<=inflight+1;
            2'b01:inflight<=inflight-1;
            default:begin end
        endcase
        if(wr_req&&wr_grant) begin
            waddr[writes]<=wr_addr;wdata[writes]<=wr_data;writes<=writes+1;
        end
        if(cycles>2000) begin $display("FAIL timeout");$finish;end
    end

    initial begin
        repeat(4) @(posedge clk);rst_n=1;
        @(posedge clk);start=1;@(posedge clk);start=0;
        wait(done);@(posedge clk);
        if(reads!==4) begin $display("FAIL reads=%0d",reads);$finish;end
        if(writes!==4) begin $display("FAIL writes=%0d",writes);$finish;end
        if(max_inflight<4) begin $display("FAIL max outstanding=%0d",max_inflight);$finish;end
        if(waddr[0]!==DST) begin $display("FAIL first addr=%h",waddr[0]);$finish;end
        for(i=0;i<32;i=i+1) begin
            if(wdata[0][i*8+:8]!==8'h20) begin
                $display("FAIL interp lane %0d=%h",i,wdata[0][i*8+:8]);$finish;
            end
            if(wdata[0][256+i*8+:8]!==8'h00) begin
                $display("FAIL invalid lane %0d",i);$finish;
            end
        end
        if(dut.perf_read_requests!==4||dut.perf_write_requests!==4) begin
            $display("FAIL perf read=%0d write=%0d",dut.perf_read_requests,
                     dut.perf_write_requests);$finish;
        end
        if(dut.perf_cache_hits!==4) begin
            $display("FAIL cache hits=%0d",dut.perf_cache_hits);$finish;
        end
        $display("PASS tb_sa_engine_int8_optimized cycles=%0d",dut.perf_cycles);
        $finish;
    end
endmodule
