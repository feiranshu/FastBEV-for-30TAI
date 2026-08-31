`timescale 1ns/1ps

module tb_lut_engine_backpressure;
    localparam [31:0] LUT_BASE  = 32'h10000000;
    localparam [31:0] FEAT_BASE = 32'h20000000;
    localparam [31:0] OUT_BASE  = 32'h30000000;
    localparam integer READ_FIFO_DEPTH = 32;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg engine_start = 1'b0;
    wire engine_done;

    wire [31:0] rd_addr;
    wire rd_req;
    wire rd_grant;
    reg [511:0] rd_data = 512'd0;
    reg rd_data_valid = 1'b0;
    wire rd_data_ready;

    wire [31:0] wr_addr;
    wire [511:0] wr_data;
    wire wr_req;
    wire wr_grant;

    wire [31:0] perf_lut_wait, perf_feat_wait, perf_write_wait;
    wire [31:0] perf_valid_voxel, perf_lut_reads, perf_feat_reads;
    wire [31:0] perf_writes, perf_pipe_stall;

    reg [31:0] read_addr_fifo [0:READ_FIFO_DEPTH-1];
    integer read_head = 0;
    integer read_tail = 0;
    integer read_count = 0;
    reg [15:0] lfsr = 16'h1ace;

    integer accepted_writes = 0;
    integer frame_writes = 0;
    integer timeout_cycles;
    integer expected_pair;
    integer expected_k;
    reg [31:0] expected_addr;
    reg [511:0] expected_data;
    reg write_stalled = 1'b0;
    reg [31:0] stalled_addr = 32'd0;
    reg [511:0] stalled_data = 512'd0;

    lut_engine #(.FEAT_MAX_OUTSTANDING(4)) dut (
        .engine_start(engine_start), .engine_done(engine_done),
        .lut_base_addr(LUT_BASE), .lut_size(32'd72),
        .feat2d_base_addr(FEAT_BASE), .feat3d_wr_addr(OUT_BASE),
        .bev_x(8'd200), .bev_y(8'd2), .bev_z(8'd4),
        .img_w(12'd176), .img_h(12'd64),
        .rd_addr(rd_addr), .rd_req(rd_req), .rd_grant(rd_grant),
        .rd_data(rd_data), .rd_data_valid(rd_data_valid),
        .rd_data_ready(rd_data_ready),
        .wr_addr(wr_addr), .wr_data(wr_data),
        .wr_req(wr_req), .wr_grant(wr_grant),
        .perf_lut_wait(perf_lut_wait), .perf_feat_wait(perf_feat_wait),
        .perf_write_wait(perf_write_wait), .perf_valid_voxel(perf_valid_voxel),
        .perf_lut_reads(perf_lut_reads), .perf_feat_reads(perf_feat_reads),
        .perf_writes(perf_writes), .perf_pipe_stall(perf_pipe_stall),
        .clk(clk), .rst_n(rst_n)
    );

    always #2.5 clk = ~clk;

    // Deterministic pseudo-random request admission, response gaps and write
    // backpressure. The response FIFO is ordered, matching the fixed DDR bridge.
    assign rd_grant = rd_req && lfsr[0] && (read_count < READ_FIFO_DEPTH);
    assign wr_grant = wr_req && lfsr[3];

    function [511:0] response_for_addr;
        input [31:0] addr;
        integer lane;
        reg [31:0] fp;
        begin
            if (addr[31:28] == 4'h1) begin
                // cam=0,u=0,v=0, except lane 7 which is invalid. This covers
                // cached valid/invalid entries at x=7 while x=8 refills.
                response_for_addr = 512'd0;
                response_for_addr[7*64 +: 16] = 16'hffff;
            end else begin
                // Four distinguishable feature beats. Expected quantized bytes:
                // 0.0 -> 0, +1.0 -> 14, -1.0 -> -14, +2.0 -> 29.
                case (addr[7:6])
                    2'd0: fp = 32'h00000000;
                    2'd1: fp = 32'h3f800000;
                    2'd2: fp = 32'hbf800000;
                    default: fp = 32'h40000000;
                endcase
                for (lane = 0; lane < 16; lane = lane + 1)
                    response_for_addr[lane*32 +: 32] = fp;
            end
        end
    endfunction

    function [127:0] repeat_byte16;
        input [7:0] value;
        integer idx;
        begin
            for (idx = 0; idx < 16; idx = idx + 1)
                repeat_byte16[idx*8 +: 8] = value;
        end
    endfunction

    task fail;
        input [8*160-1:0] message;
        begin
            $display("TEST_FAIL: %0s", message);
            $finish;
        end
    endtask

    task pulse_start;
        begin
            @(negedge clk);
            engine_start = 1'b1;
            @(negedge clk);
            engine_start = 1'b0;
        end
    endtask

    task run_complete_frame;
        begin
            frame_writes = 0;
            pulse_start;
            timeout_cycles = 0;
            while (!engine_done && timeout_cycles < 250000) begin
                @(posedge clk);
                timeout_cycles = timeout_cycles + 1;
            end
            if (timeout_cycles >= 250000)
                fail("engine timeout under random DDR latency/backpressure");
            #1;
            if (frame_writes != 288) fail("accepted write count mismatch");
            if (perf_valid_voxel !== 32'd64) fail("valid voxel count mismatch");
            if (perf_lut_reads !== 32'd16) fail("cached LUT read count mismatch");
            if (perf_feat_reads !== 32'd256) fail("feature request count mismatch");
            if (perf_writes !== 32'd288) fail("performance write count mismatch");
            if (read_count != 0) fail("engine_done asserted before read queue drained");
            $display("BACKPRESSURE_FRAME: cycles=%0d lut_wait=%0d feat_wait=%0d write_wait=%0d stalls=%0d",
                     timeout_cycles, perf_lut_wait, perf_feat_wait,
                     perf_write_wait, perf_pipe_stall);
            repeat (3) @(posedge clk);
        end
    endtask

    // Ordered DDR response model. It permits four accepted feature requests to
    // queue and inserts random response bubbles while never reordering data.
    always @(posedge clk) begin : ddr_read_model
        reg push;
        reg pop;
        if (!rst_n) begin
            read_head <= 0;
            read_tail <= 0;
            read_count <= 0;
            rd_data_valid <= 1'b0;
            rd_data <= 512'd0;
            lfsr <= 16'h1ace;
        end else begin
            lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
            rd_data_valid <= 1'b0;
            push = rd_req && rd_grant;
            pop = (read_count != 0) && lfsr[5] && rd_data_ready;

            if (push) begin
                read_addr_fifo[read_tail] <= rd_addr;
                read_tail <= (read_tail + 1) % READ_FIFO_DEPTH;
            end
            if (pop) begin
                rd_data <= response_for_addr(read_addr_fifo[read_head]);
                rd_data_valid <= 1'b1;
                read_head <= (read_head + 1) % READ_FIFO_DEPTH;
            end
            case ({push, pop})
                2'b10: read_count <= read_count + 1;
                2'b01: read_count <= read_count - 1;
                default: read_count <= read_count;
            endcase
        end
    end

    // Check exact output ordering/data and the ready/valid stability rule.
    always @(posedge clk) begin
        if (!rst_n) begin
            write_stalled <= 1'b0;
            accepted_writes <= 0;
            frame_writes <= 0;
        end else if (wr_req) begin
            if (write_stalled && (wr_addr !== stalled_addr || wr_data !== stalled_data))
                fail("write address/data changed while wr_req was backpressured");

            if (!wr_grant) begin
                write_stalled <= 1'b1;
                stalled_addr <= wr_addr;
                stalled_data <= wr_data;
            end else begin
                write_stalled <= 1'b0;
                expected_pair = frame_writes / 32;
                expected_k = frame_writes % 32;
                expected_addr = OUT_BASE + expected_pair*64 + expected_k*12800;
                if (expected_pair == 7)
                    expected_data = 512'd0;
                else if ((expected_k & 1) == 0)
                    expected_data = {repeat_byte16(8'h0e), repeat_byte16(8'h00),
                                     repeat_byte16(8'h0e), repeat_byte16(8'h00)};
                else
                    expected_data = {repeat_byte16(8'h1d), repeat_byte16(8'hf2),
                                     repeat_byte16(8'h1d), repeat_byte16(8'hf2)};
                if (wr_addr !== expected_addr) fail("output write address mismatch");
                if (wr_data !== expected_data) fail("output write data/order mismatch");
                frame_writes <= frame_writes + 1;
                accepted_writes <= accepted_writes + 1;
            end
        end else begin
            write_stalled <= 1'b0;
        end
    end

    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        // Abort one in-flight run to verify reset recovery and stale-response
        // flushing before the two complete consecutive frames.
        pulse_start;
        repeat (120) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b0;
        repeat (3) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);

        run_complete_frame;
        run_complete_frame;
        if (accepted_writes != 576)
            fail("two-frame accepted write total mismatch");

        $display("TEST_PASS: tb_lut_engine_backpressure");
        $finish;
    end
endmodule
