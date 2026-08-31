`timescale 1ns/1ps

module tb_quant_engine_pingpong;
    reg clk=0,rst_n=0,start=0;
    always #2.5 clk=~clk;

    wire done;
    wire [31:0] rd_addr,wr_addr;
    wire rd_req,rd_ready,wr_req;
    reg rd_grant=0,rd_valid=0,wr_grant=0;
    reg [511:0] rd_data=0;
    wire [511:0] wr_data;

    integer cycles=0,reads=0,writes=0,max_pending=0;
    integer reads_before_first_write=0;
    reg [7:0] return_pipe=0;

    quant_engine #(.MAX_OUTSTANDING(4)) dut(
        .engine_start(start),.engine_done(done),
        .fp32_base_addr(32'h0000_1000),.int8_base_addr(32'h0000_8000),
        .total_pixels(32'd2),
        .rd_addr(rd_addr),.rd_req(rd_req),.rd_grant(rd_grant),
        .rd_data(rd_data),.rd_data_valid(rd_valid),.rd_data_ready(rd_ready),
        .wr_addr(wr_addr),.wr_data(wr_data),.wr_req(wr_req),
        .wr_grant(wr_grant),.clk(clk),.rst_n(rst_n)
    );

    always @(posedge clk) begin
        cycles<=cycles+1;
        rd_grant<=rd_req;
        return_pipe<={return_pipe[6:0],rd_req&&rd_grant};
        rd_valid<=return_pipe[3];
        rd_data<=0;

        // Hold both output writes until all eight source beats were accepted.
        wr_grant<=wr_req&&(reads>=8);

        if(rd_req&&rd_grant) begin
            if(rd_addr !== 32'h0000_1000+(reads<<6)) begin
                $display("FAIL: read %0d address %h",reads,rd_addr);
                $finish;
            end
            reads<=reads+1;
        end
        if(wr_req&&wr_grant) begin
            if(wr_addr !== 32'h0000_8000+(writes<<6)) begin
                $display("FAIL: write %0d address %h",writes,wr_addr);
                $finish;
            end
            if(wr_data !== 512'd0) begin
                $display("FAIL: zero FP32 input did not quantize to zero");
                $finish;
            end
            if(writes==0) reads_before_first_write<=reads;
            writes<=writes+1;
        end
        if(dut.pending_count>max_pending) max_pending<=dut.pending_count;
        if(cycles>300) begin
            $display("FAIL: timeout reads=%0d writes=%0d",reads,writes);
            $finish;
        end
    end

    initial begin
        repeat(4) @(posedge clk); rst_n<=1;
        repeat(2) @(posedge clk); start<=1;
        @(posedge clk); start<=0;
        wait(done);
        @(posedge clk);
        if(reads!=8||writes!=2) begin
            $display("FAIL: transactions reads=%0d writes=%0d",reads,writes);
            $finish;
        end
        if(reads_before_first_write!=8) begin
            $display("FAIL: second pack did not hide write stall (%0d reads)",
                     reads_before_first_write);
            $finish;
        end
        if(max_pending<4) begin
            $display("FAIL: outstanding depth only reached %0d",max_pending);
            $finish;
        end
        $display("PASS: quant ping-pong reads=%0d writes=%0d cycles=%0d max_pending=%0d",
                 reads,writes,cycles,max_pending);
        $finish;
    end
endmodule
