`timescale 1ns/1ps

module tb_lut_engine_cache;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg engine_start = 1'b0;
    wire engine_done;

    wire [31:0] rd_addr;
    wire rd_req;
    wire rd_grant = rd_req;
    reg [511:0] rd_data = 512'd0;
    reg rd_data_valid = 1'b0;
    wire rd_data_ready;
    wire [31:0] wr_addr;
    wire [511:0] wr_data;
    wire wr_req;
    wire wr_grant = wr_req;

    wire [31:0] perf_lut_wait, perf_feat_wait, perf_write_wait;
    wire [31:0] perf_valid_voxel, perf_lut_reads, perf_feat_reads;
    wire [31:0] perf_writes, perf_pipe_stall;

    reg [2:0] read_pending = 3'b000;
    integer timeout_cycles;

    lut_engine dut (
        .engine_start(engine_start), .engine_done(engine_done),
        .lut_base_addr(32'h10000000), .lut_size(32'd1600),
        .feat2d_base_addr(32'h20000000), .feat3d_wr_addr(32'h30000000),
        .bev_x(8'd200), .bev_y(8'd2), .bev_z(8'd4),
        .img_w(12'd176), .img_h(12'd64),
        .rd_addr(rd_addr), .rd_req(rd_req), .rd_grant(rd_grant),
        .rd_data(rd_data), .rd_data_valid(rd_data_valid),
        .rd_data_ready(rd_data_ready),
        .wr_addr(wr_addr), .wr_data(wr_data), .wr_req(wr_req), .wr_grant(wr_grant),
        .perf_lut_wait(perf_lut_wait), .perf_feat_wait(perf_feat_wait),
        .perf_write_wait(perf_write_wait), .perf_valid_voxel(perf_valid_voxel),
        .perf_lut_reads(perf_lut_reads), .perf_feat_reads(perf_feat_reads),
        .perf_writes(perf_writes), .perf_pipe_stall(perf_pipe_stall),
        .clk(clk), .rst_n(rst_n)
    );

    always #2.5 clk = ~clk;

    // Ordered single-outstanding DDR model with a fixed three-cycle response.
    always @(posedge clk) begin
        if (!rst_n) begin
            read_pending <= 3'b000;
            rd_data_valid <= 1'b0;
        end else begin
            read_pending <= {read_pending[1:0], rd_grant};
            rd_data_valid <= read_pending[2];
            rd_data <= 512'd0; // cam=0,u=0,v=0 and FP32 zero are both valid data.
        end
    end

    task fail;
        input [8*120-1:0] message;
        begin
            $display("TEST_FAIL: %0s", message);
            $finish;
        end
    endtask

    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);
        @(negedge clk);
        engine_start = 1'b1;
        @(negedge clk);
        engine_start = 1'b0;

        timeout_cycles = 0;
        while (!engine_done && timeout_cycles < 200000) begin
            @(posedge clk);
            timeout_cycles = timeout_cycles + 1;
        end
        if (timeout_cycles >= 200000) fail("engine timeout");
        #1;
        if (perf_valid_voxel !== 32'd1600) fail("valid voxel count mismatch");
        if (perf_lut_reads !== 32'd200)
            fail("LUT line cache must reduce 1600 entry reads to 200 line reads");
        if (perf_feat_reads !== 32'd6400) fail("feature read count mismatch");
        if (perf_writes !== 32'd6400) fail("output write count mismatch");

        $display("CACHE_PERF: cycles=%0d lut_reads=%0d feat_reads=%0d writes=%0d stalls=%0d",
                 timeout_cycles, perf_lut_reads, perf_feat_reads, perf_writes,
                 perf_pipe_stall);
        $display("TEST_PASS: tb_lut_engine_cache");
        $finish;
    end
endmodule
