`timescale 1ns/1ps

// Quant + registered DMA queue integration test.
//
// This test is intentionally separate from the direct quant_engine tests.  It
// checks that the timing-isolation read queue neither duplicates nor drops
// requests, preserves address order under backpressure, and does not weaken
// the direct external-handshake completion rule for writes.
module tb_quant_dma_queued;
    reg clk=1'b0;
    reg rst_n=1'b0;
    reg start=1'b0;
    reg [1:0] stage=2'b00;
    always #2.5 clk=~clk;

    wire done;
    wire [31:0] q_rd_addr;
    wire q_rd_req;
    wire q_rd_grant;
    wire [511:0] q_rd_data;
    wire q_rd_data_valid;
    wire q_rd_data_ready;
    wire [31:0] q_wr_addr;
    wire [511:0] q_wr_data;
    wire q_wr_req;
    wire q_wr_grant;

    wire [31:0] ddr_araddr;
    wire ddr_arvalid;
    reg ddr_arready=1'b0;
    reg [511:0] ddr_rdata=512'd0;
    reg ddr_rvalid=1'b0;
    wire ddr_rready;
    wire [31:0] ddr_awaddr;
    wire [511:0] ddr_awdata;
    wire ddr_awvalid;
    reg ddr_awready=1'b0;

    integer cycles=0;
    integer local_reads=0;
    integer external_reads=0;
    integer local_writes=0;
    integer external_writes=0;
    integer max_rd_queue=0;
    reg [7:0] response_pipe=8'd0;
    reg done_seen=1'b0;

    quant_engine #(.MAX_OUTSTANDING(4)) U_quant_engine(
        .engine_start(start),
        .engine_done(done),
        .fp32_base_addr(32'h0000_1000),
        .int8_base_addr(32'h0000_8000),
        .total_pixels(32'd2),
        .rd_addr(q_rd_addr),
        .rd_req(q_rd_req),
        .rd_grant(q_rd_grant),
        .rd_data(q_rd_data),
        .rd_data_valid(q_rd_data_valid),
        .rd_data_ready(q_rd_data_ready),
        .wr_addr(q_wr_addr),
        .wr_data(q_wr_data),
        .wr_req(q_wr_req),
        .wr_grant(q_wr_grant),
        .clk(clk),
        .rst_n(rst_n)
    );

    dma_arbiter_stage U_dma_arbiter_stage(
        .active_stage(stage),
        .quant_rd_addr(q_rd_addr),
        .quant_rd_req(q_rd_req),
        .quant_rd_grant(q_rd_grant),
        .quant_rd_data(q_rd_data),
        .quant_rd_data_valid(q_rd_data_valid),
        .quant_rd_data_ready(q_rd_data_ready),
        .quant_wr_addr(q_wr_addr),
        .quant_wr_data(q_wr_data),
        .quant_wr_req(q_wr_req),
        .quant_wr_grant(q_wr_grant),
        .lut_rd_addr(32'd0),
        .lut_rd_req(1'b0),
        .lut_rd_grant(),
        .lut_rd_data(),
        .lut_rd_data_valid(),
        .lut_rd_data_ready(1'b0),
        .lut_wr_addr(32'd0),
        .lut_wr_data(512'd0),
        .lut_wr_req(1'b0),
        .lut_wr_grant(),
        .sa_rd_addr(32'd0),
        .sa_rd_req(1'b0),
        .sa_rd_grant(),
        .sa_rd_data(),
        .sa_rd_data_valid(),
        .sa_rd_data_ready(1'b0),
        .sa_wr_addr(32'd0),
        .sa_wr_data(512'd0),
        .sa_wr_req(1'b0),
        .sa_wr_grant(),
        .ddr_araddr(ddr_araddr),
        .ddr_arvalid(ddr_arvalid),
        .ddr_arready(ddr_arready),
        .ddr_rdata(ddr_rdata),
        .ddr_rvalid(ddr_rvalid),
        .ddr_rready(ddr_rready),
        .ddr_awaddr(ddr_awaddr),
        .ddr_awdata(ddr_awdata),
        .ddr_awvalid(ddr_awvalid),
        .ddr_awready(ddr_awready),
        .clk(clk),
        .rst_n(rst_n)
    );

    always @(posedge clk) begin
        if(!rst_n) begin
            cycles<=0;
            local_reads<=0;
            external_reads<=0;
            local_writes<=0;
            external_writes<=0;
            max_rd_queue<=0;
            response_pipe<=0;
            ddr_rvalid<=0;
            ddr_rdata<=0;
            ddr_arready<=0;
            ddr_awready<=0;
            done_seen<=0;
        end else begin
            cycles<=cycles+1;

            // Periodic read backpressure fills the timing-isolation queue;
            // delayed write ready verifies direct completion semantics.
            ddr_arready <= (cycles[1:0] != 2'b01);
            ddr_awready <= (cycles>35) && (cycles[1:0] != 2'b10);

            response_pipe <= {response_pipe[6:0],
                              ddr_arvalid&&ddr_arready};
            ddr_rvalid <= response_pipe[3];
            ddr_rdata <= 512'd0;

            if(q_rd_req&&q_rd_grant) begin
                if(q_rd_addr !== 32'h0000_1000+(local_reads<<6)) begin
                    $display("FAIL: local read %0d address %h",
                             local_reads,q_rd_addr);
                    $finish;
                end
                local_reads<=local_reads+1;
            end

            if(ddr_arvalid&&ddr_arready) begin
                if(ddr_araddr !== 32'h0000_1000+(external_reads<<6)) begin
                    $display("FAIL: external read %0d address %h",
                             external_reads,ddr_araddr);
                    $finish;
                end
                external_reads<=external_reads+1;
            end

            if(q_wr_req&&q_wr_grant) begin
                if(q_wr_addr !== 32'h0000_8000+(local_writes<<6)) begin
                    $display("FAIL: local write %0d address %h",
                             local_writes,q_wr_addr);
                    $finish;
                end
                if(q_wr_data !== 512'd0) begin
                    $display("FAIL: local write data is not zero");
                    $finish;
                end
                local_writes<=local_writes+1;
            end

            if(ddr_awvalid&&ddr_awready) begin
                if(ddr_awaddr !== 32'h0000_8000+(external_writes<<6)) begin
                    $display("FAIL: external write %0d address %h",
                             external_writes,ddr_awaddr);
                    $finish;
                end
                if(ddr_awdata !== 512'd0) begin
                    $display("FAIL: external write data is not zero");
                    $finish;
                end
                external_writes<=external_writes+1;
            end

            if(U_dma_arbiter_stage.rd_fifo_count>max_rd_queue)
                max_rd_queue<=U_dma_arbiter_stage.rd_fifo_count;
            if(U_dma_arbiter_stage.rd_fifo_count>2) begin
                $display("FAIL: read queue overflow");
                $finish;
            end
            if(ddr_rvalid&&!ddr_rready) begin
                $display("FAIL: response model encountered unexpected stall");
                $finish;
            end
            if(done) begin
                done_seen<=1'b1;
            end

            if(cycles>500) begin
                $display("FAIL: timeout lr=%0d er=%0d lw=%0d ew=%0d",
                         local_reads,external_reads,
                         local_writes,external_writes);
                $finish;
            end
        end
    end

    initial begin
        repeat(4) @(posedge clk);
        @(negedge clk);
        rst_n=1'b1;
        stage=2'b01;
        @(negedge clk);
        start=1'b1;
        @(negedge clk);
        start=1'b0;

        wait(done_seen);
        stage=2'b00;
        repeat(2) @(posedge clk);

        if(local_reads!=8 || external_reads!=8 ||
           local_writes!=2 || external_writes!=2) begin
            $display("FAIL: transaction count lr=%0d er=%0d lw=%0d ew=%0d",
                     local_reads,external_reads,
                     local_writes,external_writes);
            $finish;
        end
        if(response_pipe!=0 || ddr_rvalid) begin
            $display("FAIL: response pipeline did not drain");
            $finish;
        end
        if(max_rd_queue<2) begin
            $display("FAIL: read queue not exercised depth=%0d",
                     max_rd_queue);
            $finish;
        end
        $display("PASS: Quant+DMA queued integration cycles=%0d rdq=%0d",
                 cycles,max_rd_queue);
        $finish;
    end
endmodule
