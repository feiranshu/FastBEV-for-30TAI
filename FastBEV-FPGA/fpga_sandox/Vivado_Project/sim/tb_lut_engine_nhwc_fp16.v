`timescale 1ns/1ps

module tb_lut_engine_nhwc_fp16;
    localparam [31:0] LUT_BASE  = 32'h10000000;
    localparam [31:0] FEAT_BASE = 32'h20000000;
    localparam [31:0] OUT_BASE  = 32'h30000000;
    localparam [31:0] CBLK_STRIDE = 32'd128; // 2*2*16*2 bytes

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

    reg rd_allow = 1'b0;
    reg wr_allow = 1'b0;
    reg write_stalled = 1'b0;
    reg [31:0] stalled_addr = 32'd0;
    reg [511:0] stalled_data = 512'd0;
    assign rd_grant = rd_req && rd_allow;
    assign wr_grant = wr_req && wr_allow;

    reg pending = 1'b0;
    reg [31:0] pending_addr = 32'd0;
    integer feature_reads = 0;
    integer lut_reads = 0;
    integer writes = 0;
    integer read_stalls = 0;
    integer write_stalls = 0;
    integer timeout = 0;
    integer valid_ordinal;
    integer beat;
    integer cam;
    integer u;
    integer v;
    integer voxel;
    integer zslice;
    integer local_voxel;
    integer cblk;
    integer spatial_offset;
    reg [31:0] expected_addr;
    reg [511:0] expected_data;
    reg [255:0] expected_half;

    lut_engine dut (
        .engine_start(engine_start), .engine_done(engine_done),
        .lut_base_addr(LUT_BASE), .lut_size(32'd8),
        .feat2d_base_addr(FEAT_BASE), .feat3d_wr_addr(OUT_BASE),
        .img_w(12'd160), .img_h(12'd120), .cameras(8'd6),
        .bev_x(8'd2), .bev_y(8'd2), .bev_z(8'd2),
        .rd_addr(rd_addr), .rd_req(rd_req), .rd_grant(rd_grant),
        .rd_data(rd_data), .rd_data_valid(rd_data_valid),
        .rd_data_ready(rd_data_ready),
        .wr_addr(wr_addr), .wr_data(wr_data),
        .wr_req(wr_req), .wr_grant(wr_grant),
        .clk(clk), .rst_n(rst_n)
    );

    always #2.5 clk = ~clk;

    // Force exactly one backpressure cycle before each accepted request.
    always @(posedge clk) begin
        if (!rst_n) begin
            rd_allow <= 1'b0;
            wr_allow <= 1'b0;
        end else begin
            if (!rd_req || rd_grant)
                rd_allow <= 1'b0;
            else
                rd_allow <= 1'b1;
            if (!wr_req || wr_grant)
                wr_allow <= 1'b0;
            else
                wr_allow <= 1'b1;
        end
    end

    function [511:0] repeat_fp32_16;
        input [31:0] value;
        integer idx;
        begin
            for (idx = 0; idx < 16; idx = idx + 1)
                repeat_fp32_16[idx*32 +: 32] = value;
        end
    endfunction

    function [255:0] repeat_fp16_16;
        input [15:0] value;
        integer idx;
        begin
            for (idx = 0; idx < 16; idx = idx + 1)
                repeat_fp16_16[idx*16 +: 16] = value;
        end
    endfunction

    function [511:0] response_for_addr;
        input [31:0] addr;
        begin
            response_for_addr = 512'd0;
            if (addr == LUT_BASE) begin
                // BEV traversal is x-fastest: (0,0), (1,0), (0,1), (1,1).
                // Include the final NHWC pixel to prove the full input extent.
                response_for_addr[  0 +: 64] = {16'd0,16'd119,16'd159,16'd5};
                response_for_addr[ 64 +: 64] = {16'd0,16'd54, 16'd159,16'd1};
                response_for_addr[128 +: 64] = {16'd0,16'd0,  16'd0,  16'hffff};
                response_for_addr[192 +: 64] = {16'd0,16'd32, 16'd0,  16'd0};
                response_for_addr[256 +: 64] = {16'd0,16'd32,16'd0,16'd0};
                response_for_addr[320 +: 64] = {16'd0,16'd0,16'd0,16'hffff};
                response_for_addr[384 +: 64] = {16'd0,16'd0,16'd0,16'hffff};
                response_for_addr[448 +: 64] = {16'd0,16'd0,16'd0,16'hffff};
            end else begin
                case (addr[7:0])
                    8'h00: response_for_addr = repeat_fp32_16(32'h3f800000); // 1
                    8'h40: response_for_addr = repeat_fp32_16(32'h40000000); // 2
                    8'h80: response_for_addr = repeat_fp32_16(32'h40800000); // 4
                    default: response_for_addr = repeat_fp32_16(32'h41000000); // 8
                endcase
            end
        end
    endfunction

    task fail;
        input [8*180-1:0] message;
        begin
            $display("TEST_FAIL: %0s", message);
            $finish;
        end
    endtask

    // One-cycle queued DDR response.
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

    // Exact NHWC address scoreboard: one pixel is four consecutive 64-byte
    // reads because all 64 FP32 channels are contiguous.
    always @(posedge clk) begin
        if (rst_n && rd_req && rd_grant) begin
            if (rd_addr == LUT_BASE) begin
                lut_reads <= lut_reads + 1;
            end else begin
                valid_ordinal = feature_reads / 4;
                beat = feature_reads % 4;
                case (valid_ordinal)
                    0: begin cam=5; u=159; v=119; end
                    1: begin cam=1; u=159; v=54;  end
                    default: begin cam=0; u=0; v=32; end
                endcase
                expected_addr = FEAT_BASE + (((cam*120 + v)*160 + u)*256) +
                                beat*64;
                if (rd_addr !== expected_addr)
                    fail("NHWC feature read address mismatch");
                feature_reads <= feature_reads + 1;
            end
        end
        if (rst_n && rd_req && !rd_grant)
            read_stalls <= read_stalls + 1;
    end

    // Check [1,cblk16,x,y,c16] for a 2x2x2 reduced grid. Each write packs
    // y_even in the low half and y_odd in the high half. Global cblk=z*4+src.
    always @(posedge clk) begin
        if (rst_n && wr_req && wr_grant) begin
            voxel = writes / 4;          // one completed Y pair
            zslice = voxel / 2;
            local_voxel = voxel % 2;     // X coordinate
            cblk = writes % 4;
            spatial_offset = local_voxel * 64; // x*Y*32, y_even=0
            case (cblk)
                0: expected_half = repeat_fp16_16(16'h3c00);
                1: expected_half = repeat_fp16_16(16'h4000);
                2: expected_half = repeat_fp16_16(16'h4400);
                default: expected_half = repeat_fp16_16(16'h4800);
            endcase
            expected_addr = OUT_BASE + (zslice*4+cblk)*CBLK_STRIDE +
                            spatial_offset;
            if (zslice == 1 && local_voxel == 1)
                expected_data = 512'd0;
            else if (local_voxel == 0)
                expected_data = {256'd0, expected_half};
            else
                expected_data = {expected_half, expected_half};
            if (wr_addr !== expected_addr)
                fail("NCHWc16 output address mismatch");
            if (wr_data !== expected_data)
                fail("FP32-to-FP16 conversion or Y-pair packing mismatch");
            writes <= writes + 1;
        end
        if (rst_n && wr_req && !wr_grant)
            write_stalls <= write_stalls + 1;
    end

    // Ready/valid rule: an unaccepted write must remain unchanged.
    always @(posedge clk) begin
        if (!rst_n) begin
            write_stalled <= 1'b0;
        end else begin
            if (write_stalled && (!wr_req || wr_addr !== stalled_addr ||
                                  wr_data !== stalled_data))
                fail("write request changed while DDR applied backpressure");
            write_stalled <= wr_req && !wr_grant;
            if (wr_req && !wr_grant) begin
                stalled_addr <= wr_addr;
                stalled_data <= wr_data;
            end
        end
    end

    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);
        @(negedge clk); engine_start = 1'b1;
        @(negedge clk); engine_start = 1'b0;
        while (!engine_done && timeout < 10000) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
        #1;
        if (timeout >= 10000) fail("engine timeout");
        if (lut_reads != 1) fail("four entries must use one LUT beat");
        if (feature_reads != 16) fail("NHWC path must read four beats per valid voxel");
        if (writes != 16) fail("2x2x2 NCHWc16 layout must contain 16 writes");
        if (read_stalls == 0 || write_stalls == 0)
            fail("test did not exercise DDR backpressure");
        $display("NHWC_FP16: cycles=%0d lut_reads=%0d feature_reads=%0d writes=%0d rd_stall=%0d wr_stall=%0d",
                 timeout, lut_reads, feature_reads, writes,
                 read_stalls, write_stalls);
        $display("TEST_PASS: tb_lut_engine_nhwc_fp16");
        $finish;
    end
endmodule
