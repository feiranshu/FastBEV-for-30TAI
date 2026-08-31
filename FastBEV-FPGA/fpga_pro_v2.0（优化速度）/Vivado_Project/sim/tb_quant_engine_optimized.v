`timescale 1ns/1ps
module tb_quant_engine_optimized;
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
    quant_engine #(.MAX_OUTSTANDING(4)) dut(
        .engine_start(start),.engine_done(done),.fp32_base_addr(32'h1000),
        .int8_base_addr(32'h8000),.total_pixels(1),.rd_addr(rd_addr),
        .rd_req(rd_req),.rd_grant(rd_grant),.rd_data(rd_data),
        .rd_data_valid(rd_valid),.rd_data_ready(rd_ready),.wr_addr(wr_addr),
        .wr_data(wr_data),.wr_req(wr_req),.wr_grant(wr_grant),
        .clk(clk),.rst_n(rst_n));
    reg [3:0] rv=0;
    integer reads=0,writes=0,inflight=0,max_inflight=0,cycles=0;
    always @(posedge clk) begin
        cycles<=cycles+1;
        rv<={rv[2:0],rd_req};rd_valid<=rv[3];rd_data<=0;
        if(rd_req&&rd_grant) begin reads<=reads+1;
            if(inflight+1>max_inflight)max_inflight<=inflight+1;end
        case({rd_req&&rd_grant,rd_valid&&rd_ready})
            2'b10:inflight<=inflight+1;
            2'b01:inflight<=inflight-1;
            default:begin end
        endcase
        if(wr_req&&wr_grant)writes<=writes+1;
        if(cycles>1000)begin $display("FAIL timeout");$finish;end
    end
    initial begin
        repeat(4)@(posedge clk);rst_n=1;@(posedge clk);start=1;
        @(posedge clk);start=0;wait(done);@(posedge clk);
        if(reads!==4||writes!==1||max_inflight<4||wr_data!==0)begin
            $display("FAIL r=%0d w=%0d max=%0d data=%h",reads,writes,
                     max_inflight,wr_data);$finish;end
        $display("PASS tb_quant_engine_optimized cycles=%0d",dut.perf_cycles);
        $finish;
    end
endmodule
