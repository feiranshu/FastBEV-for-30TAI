`timescale 1ns/1ps

// Real-LUT full-frame regression. Feature DDR returns FP32 zero so the test
// isolates all 160000 LUT entries, NHWC address bounds, output traversal and
// the complete [1,16,200,200,16] FP16 address range.
module tb_lut_engine_full_frame;
    localparam [31:0] LUT_BASE    = 32'h10000000;
    localparam [31:0] FEAT_BASE   = 32'h20000000;
    localparam [31:0] OUT_BASE    = 32'h30000000;
    localparam [31:0] LUT_BYTES   = 32'd1280000;
    localparam [31:0] FEAT_BYTES  = 32'd29491200;
    localparam [31:0] CBLK_STRIDE = 32'd1280000;

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

    reg [63:0] lut_mem [0:159999];
    reg pending = 1'b0;
    reg [31:0] pending_addr = 32'd0;
    integer lut_reads = 0;
    integer feature_reads = 0;
    integer accepted_writes = 0;
    integer timeout_cycles = 0;
    integer pair_index;
    integer pair_in_z;
    integer zslice;
    integer xcoord;
    integer y_even;
    integer source_block;
    integer beat;
    integer lane;
    reg [31:0] expected_addr;

    lut_engine dut (
        .engine_start(engine_start), .engine_done(engine_done),
        .lut_base_addr(LUT_BASE), .lut_size(32'd160000),
        .feat2d_base_addr(FEAT_BASE), .feat3d_wr_addr(OUT_BASE),
        .img_w(12'd160), .img_h(12'd120), .cameras(8'd6),
        .bev_x(8'd200), .bev_y(8'd200), .bev_z(8'd4),
        .rd_addr(rd_addr), .rd_req(rd_req), .rd_grant(rd_grant),
        .rd_data(rd_data), .rd_data_valid(rd_data_valid),
        .rd_data_ready(rd_data_ready),
        .wr_addr(wr_addr), .wr_data(wr_data),
        .wr_req(wr_req), .wr_grant(wr_grant),
        .clk(clk), .rst_n(rst_n)
    );

    always #2.5 clk = ~clk;

    task fail;
        input [8*180-1:0] message;
        begin $fatal(1, "TEST_FAIL: %0s", message); end
    endtask

    function [511:0] response_for_addr;
        input [31:0] addr;
        integer local_beat;
        integer local_lane;
        begin
            response_for_addr = 512'd0;
            if (addr >= LUT_BASE && addr < LUT_BASE + LUT_BYTES) begin
                local_beat = (addr - LUT_BASE) >> 6;
                for (local_lane = 0; local_lane < 8; local_lane = local_lane + 1)
                    response_for_addr[local_lane*64 +: 64] =
                        lut_mem[local_beat*8 + local_lane];
            end
        end
    endfunction

    // Ordered one-entry DDR response queue.
    always @(posedge clk) begin
        if (!rst_n) begin
            pending <= 1'b0;
            rd_data_valid <= 1'b0;
            rd_data <= 512'd0;
        end else begin
            rd_data_valid <= pending && rd_data_ready;
            if (pending && rd_data_ready)
                rd_data <= response_for_addr(pending_addr);
            pending <= rd_req && rd_grant;
            if (rd_req && rd_grant)
                pending_addr <= rd_addr;
        end
    end

    always @(posedge clk) begin
        if (rst_n && rd_req && rd_grant) begin
            if (rd_addr >= LUT_BASE && rd_addr < LUT_BASE + LUT_BYTES) begin
                if (rd_addr[5:0] != 6'd0)
                    fail("unaligned LUT read");
                lut_reads <= lut_reads + 1;
            end else if (rd_addr >= FEAT_BASE && rd_addr < FEAT_BASE + FEAT_BYTES) begin
                if (rd_addr[5:0] != 6'd0)
                    fail("unaligned NHWC feature read");
                feature_reads <= feature_reads + 1;
            end else begin
                fail("DDR read outside LUT or [6,120,160,64] feature buffers");
            end
        end
    end

    // Write order is z, y-pair, x, source-cblk. Address layout is
    // [global_cblk=z*4+source_cblk][x][y][c16].
    always @(posedge clk) begin
        if (rst_n && wr_req && wr_grant) begin
            pair_index = accepted_writes / 4;
            source_block = accepted_writes % 4;
            zslice = pair_index / 20000;
            pair_in_z = pair_index % 20000;
            xcoord = pair_in_z % 200;
            y_even = (pair_in_z / 200) * 2;
            expected_addr = OUT_BASE + (zslice*4+source_block)*CBLK_STRIDE +
                            (xcoord*200+y_even)*32;
            if (wr_addr !== expected_addr)
                fail("full-frame NCHWc16 output address mismatch");
            if (wr_data !== 512'd0)
                fail("zero feature input must produce zero FP16 output");
            accepted_writes <= accepted_writes + 1;
        end
    end

    initial begin
        $readmemh("Vivado_Project/sim/build/fastbev_lut_table.hex", lut_mem);
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);
        @(negedge clk); engine_start = 1'b1;
        @(negedge clk); engine_start = 1'b0;
        while (!engine_done && timeout_cycles < 10000000) begin
            @(posedge clk);
            timeout_cycles = timeout_cycles + 1;
        end
        #1;
        if (timeout_cycles >= 10000000) fail("full-frame engine timeout");
        if (lut_reads != 20000) fail("full-frame LUT read count mismatch");
        if (feature_reads != 624500) fail("real-LUT valid entry/read count mismatch");
        if (accepted_writes != 320000) fail("full-frame output write count mismatch");
        $display("FULL_FRAME: cycles=%0d lut_reads=%0d feature_reads=%0d writes=%0d",
                 timeout_cycles, lut_reads, feature_reads, accepted_writes);
        $display("TEST_PASS: tb_lut_engine_full_frame");
        $finish;
    end
endmodule
