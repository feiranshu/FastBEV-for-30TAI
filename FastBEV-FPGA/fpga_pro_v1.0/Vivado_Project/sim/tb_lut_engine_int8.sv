//==============================================================================
// tb_lut_engine_int8.sv
//   Self-checking testbench for lut_engine_int8.
//
// Coverage:
//   - LUT batch parsing and boundary crossing.
//   - INT8 NHWC feature address calculation.
//   - invalid voxel writes zero.
//   - HISTORY_CONTIG dst = base + voxel*64.
//   - FUSED_CURRENT dst uses Part3 [N][Z][C/32][X][Y][C%32] layout.
//==============================================================================
`timescale 1ns/1ps

module tb_lut_engine_int8;

    localparam int NUM_VOXELS = 24;
    localparam int BEV_X      = 3;
    localparam int BEV_Y      = 4;
    localparam int BEV_Z      = 2;
    localparam int IMG_H      = 3;
    localparam int IMG_W      = 4;
    localparam int CHANNELS   = 64;
    localparam int MEM_WORDS  = 32768;

    localparam logic [31:0] LUT_BASE    = 32'h0000_0000;
    localparam logic [31:0] FEAT_BASE   = 32'h0000_1000;
    localparam logic [31:0] HIST_BASE   = 32'h0000_4000;
    localparam logic [31:0] CONCAT_BASE = 32'h0000_8000;

    localparam logic DST_HISTORY_CONTIG = 1'b0;
    localparam logic DST_FUSED_CURRENT  = 1'b1;

    logic clk = 1'b0;
    always #5 clk = ~clk;

    logic rst_n = 1'b0;
    logic engine_start = 1'b0;
    logic engine_done;
    logic dst_mode;
    logic [31:0] dst_base_addr;

    logic [31:0] rd_addr;
    logic        rd_req;
    logic        rd_grant;
    logic [511:0] rd_data;
    logic        rd_data_valid;
    logic        rd_data_ready;

    logic [31:0] wr_addr;
    logic [511:0] wr_data;
    logic        wr_req;
    logic        wr_grant;

    lut_engine_int8 dut (
        .engine_start     ( engine_start      ),
        .engine_done      ( engine_done       ),
        .lut_base_addr    ( LUT_BASE          ),
        .lut_size         ( NUM_VOXELS[31:0]  ),
        .feat2d_base_addr ( FEAT_BASE         ),
        .dst_base_addr    ( dst_base_addr     ),
        .dst_mode         ( dst_mode          ),
        .img_w            ( IMG_W[11:0]       ),
        .img_h            ( IMG_H[11:0]       ),
        .bev_x            ( BEV_X[7:0]        ),
        .bev_y            ( BEV_Y[7:0]        ),
        .rd_addr          ( rd_addr           ),
        .rd_req           ( rd_req            ),
        .rd_grant         ( rd_grant          ),
        .rd_data          ( rd_data           ),
        .rd_data_valid    ( rd_data_valid     ),
        .rd_data_ready    ( rd_data_ready     ),
        .wr_addr          ( wr_addr           ),
        .wr_data          ( wr_data           ),
        .wr_req           ( wr_req            ),
        .wr_grant         ( wr_grant          ),
        .clk              ( clk               ),
        .rst_n            ( rst_n             )
    );

    logic [31:0] mem32 [0:MEM_WORDS-1];
    int lut_cam [0:NUM_VOXELS-1];
    int lut_u   [0:NUM_VOXELS-1];
    int lut_v   [0:NUM_VOXELS-1];

    task automatic set_byte(input int byte_addr, input logic [7:0] value);
        begin
            mem32[byte_addr >> 2][(byte_addr & 3)*8 +: 8] = value;
        end
    endtask

    function automatic logic [7:0] get_byte(input int byte_addr);
        begin
            get_byte = mem32[byte_addr >> 2][(byte_addr & 3)*8 +: 8];
        end
    endfunction

    function automatic logic [7:0] feature_value(input int cam, input int v, input int u, input int ch);
        int signed_val;
        begin
            signed_val = ((cam * 53 + v * 29 + u * 11 + ch * 7) % 256) - 128;
            feature_value = signed_val & 8'hFF;
        end
    endfunction

    task automatic set_lut_entry(input int idx, input int cam, input int u, input int v);
        logic [15:0] cam16;
        logic [15:0] u16;
        logic [15:0] v16;
        int word_idx;
        begin
            cam16 = cam & 16'hFFFF;
            u16 = u & 16'hFFFF;
            v16 = v & 16'hFFFF;
            word_idx = (LUT_BASE >> 2) + idx * 2;
            mem32[word_idx]     = {u16, cam16};
            mem32[word_idx + 1] = {16'd0, v16};
            lut_cam[idx] = cam;
            lut_u[idx]   = u;
            lut_v[idx]   = v;
        end
    endtask

    task automatic init_memory;
        int i;
        int cam;
        int v;
        int u;
        int ch;
        int pixel_idx;
        int byte_addr;
        begin
            for (i = 0; i < MEM_WORDS; i = i + 1)
                mem32[i] = 32'd0;

            set_lut_entry( 0,  0, 0, 0);
            set_lut_entry( 1, -1, 0, 0);
            set_lut_entry( 2,  1, 3, 2);
            set_lut_entry( 3,  0, 2, 1);
            set_lut_entry( 4, -1, 0, 0);
            set_lut_entry( 5, -1, 0, 0);
            set_lut_entry( 6,  1, 0, 0);
            set_lut_entry( 7,  0, 3, 2);
            set_lut_entry( 8,  1, 1, 1);
            set_lut_entry( 9, -1, 0, 0);
            set_lut_entry(10,  0, 0, 0);
            set_lut_entry(11,  1, 2, 0);
            set_lut_entry(12, -1, 0, 0);
            set_lut_entry(13,  0, 1, 0);
            set_lut_entry(14, -1, 0, 0);
            set_lut_entry(15,  1, 3, 1);
            set_lut_entry(16,  0, 0, 2);
            set_lut_entry(17, -1, 0, 0);
            set_lut_entry(18, -1, 0, 0);
            set_lut_entry(19,  1, 0, 2);
            set_lut_entry(20,  0, 2, 0);
            set_lut_entry(21, -1, 0, 0);
            set_lut_entry(22,  1, 1, 2);
            set_lut_entry(23,  0, 3, 1);

            for (cam = 0; cam < 2; cam = cam + 1) begin
                for (v = 0; v < IMG_H; v = v + 1) begin
                    for (u = 0; u < IMG_W; u = u + 1) begin
                        pixel_idx = (cam * IMG_H + v) * IMG_W + u;
                        for (ch = 0; ch < CHANNELS; ch = ch + 1) begin
                            byte_addr = FEAT_BASE + pixel_idx * 64 + ch;
                            set_byte(byte_addr, feature_value(cam, v, u, ch));
                        end
                    end
                end
            end
        end
    endtask

    int cycle_cnt;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cycle_cnt <= 0;
        else
            cycle_cnt <= cycle_cnt + 1;
    end

    typedef enum logic [1:0] {RD_IDLE, RD_DELAY, RD_RESP} rd_state_t;
    rd_state_t rd_state;
    logic [31:0] rd_addr_buf;
    int rd_delay_cnt;
    logic arready_model;

    assign rd_grant = rd_req && arready_model;

    integer ri;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_state      <= RD_IDLE;
            arready_model <= 1'b0;
            rd_data_valid <= 1'b0;
            rd_data       <= 512'd0;
            rd_addr_buf   <= 32'd0;
            rd_delay_cnt  <= 0;
        end else begin
            case (rd_state)
                RD_IDLE: begin
                    arready_model <= (cycle_cnt[1:0] != 2'b10);
                    rd_data_valid <= 1'b0;
                    if (rd_req && arready_model) begin
                        rd_addr_buf   <= rd_addr;
                        rd_delay_cnt  <= 1 + (cycle_cnt % 4);
                        arready_model <= 1'b0;
                        rd_state      <= RD_DELAY;
                    end
                end
                RD_DELAY: begin
                    if (rd_delay_cnt == 0) begin
                        for (ri = 0; ri < 16; ri = ri + 1)
                            rd_data[ri*32 +: 32] <= mem32[(rd_addr_buf >> 2) + ri];
                        rd_data_valid <= 1'b1;
                        rd_state      <= RD_RESP;
                    end else begin
                        rd_delay_cnt <= rd_delay_cnt - 1;
                    end
                end
                RD_RESP: begin
                    if (rd_data_valid && rd_data_ready) begin
                        rd_data_valid <= 1'b0;
                        rd_state      <= RD_IDLE;
                    end
                end
                default: rd_state <= RD_IDLE;
            endcase
        end
    end

    logic awready_model;
    assign wr_grant = wr_req && awready_model;

    integer wi;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            awready_model <= 1'b0;
        end else begin
            awready_model <= (cycle_cnt[2:0] != 3'd2) && (cycle_cnt[2:0] != 3'd5);
            if (wr_req && awready_model) begin
                for (wi = 0; wi < 16; wi = wi + 1)
                    mem32[(wr_addr >> 2) + wi] <= wr_data[wi*32 +: 32];
            end
        end
    end

    task automatic reset_dut;
        begin
            rst_n = 1'b0;
            engine_start = 1'b0;
            repeat (6) @(posedge clk);
            rst_n = 1'b1;
            repeat (4) @(posedge clk);
        end
    endtask

    task automatic run_engine(input logic mode, input logic [31:0] base);
        int timeout;
        begin
            dst_mode      = mode;
            dst_base_addr = base;
            @(posedge clk);
            engine_start <= 1'b1;
            @(posedge clk);
            engine_start <= 1'b0;

            timeout = 0;
            while (!engine_done && timeout < 10000) begin
                @(posedge clk);
                timeout++;
            end
            if (!engine_done) begin
                $display("FAIL: timeout mode=%0d", mode);
                $finish;
            end
            repeat (3) @(posedge clk);
        end
    endtask

    function automatic logic [7:0] expected_byte(input int voxel, input int ch);
        begin
            if (lut_cam[voxel] < 0)
                expected_byte = 8'd0;
            else
                expected_byte = feature_value(lut_cam[voxel], lut_v[voxel], lut_u[voxel], ch);
        end
    endfunction

    task automatic verify_history(output int errors);
        int voxel;
        int ch;
        logic [7:0] actual;
        logic [7:0] expected;
        begin
            errors = 0;
            for (voxel = 0; voxel < NUM_VOXELS; voxel = voxel + 1) begin
                for (ch = 0; ch < CHANNELS; ch = ch + 1) begin
                    actual   = get_byte(HIST_BASE + voxel * 64 + ch);
                    expected = expected_byte(voxel, ch);
                    if (actual !== expected) begin
                        if (errors < 16)
                            $display("HISTORY mismatch voxel=%0d ch=%0d actual=%02x expected=%02x",
                                     voxel, ch, actual, expected);
                        errors++;
                    end
                end
            end
        end
    endtask

    function automatic int part3_addr(input int z, input int x, input int y, input int c);
        int cg;
        int lane;
        begin
            cg = c / 32;
            lane = c % 32;
            part3_addr = CONCAT_BASE + ((((z * 8 + cg) * BEV_X + x) * BEV_Y + y) * 32) + lane;
        end
    endfunction

    task automatic verify_fused(output int errors);
        int voxel;
        int ch;
        int c;
        int x;
        int y;
        int z;
        logic [7:0] actual;
        logic [7:0] expected;
        begin
            errors = 0;
            for (voxel = 0; voxel < NUM_VOXELS; voxel = voxel + 1) begin
                x = voxel % BEV_X;
                y = (voxel / BEV_X) % BEV_Y;
                z = voxel / (BEV_X * BEV_Y);

                for (c = 0; c < 256; c = c + 1) begin
                    actual = get_byte(part3_addr(z, x, y, c));
                    if (c < 64) begin
                        ch = c;
                        expected = expected_byte(voxel, ch);
                    end else begin
                        expected = 8'hA5;
                    end

                    if (actual !== expected) begin
                        if (errors < 16)
                            $display("FUSED part3 mismatch z=%0d x=%0d y=%0d c=%0d actual=%02x expected=%02x",
                                     z, x, y, c, actual, expected);
                        errors++;
                    end
                end
            end
        end
    endtask

    task automatic fill_region(input int base, input int bytes, input logic [7:0] value);
        int i;
        begin
            for (i = 0; i < bytes; i = i + 1)
                set_byte(base + i, value);
        end
    endtask

    int errors_h;
    int errors_f;

    initial begin
        init_memory();

        fill_region(HIST_BASE, NUM_VOXELS * 64, 8'hCC);
        reset_dut();
        run_engine(DST_HISTORY_CONTIG, HIST_BASE);
        verify_history(errors_h);
        if (errors_h == 0)
            $display("*** PASS *** lut_engine_int8 HISTORY_CONTIG");
        else
            $display("*** FAIL *** lut_engine_int8 HISTORY_CONTIG errors=%0d", errors_h);

        fill_region(CONCAT_BASE, NUM_VOXELS * 256, 8'hA5);
        reset_dut();
        run_engine(DST_FUSED_CURRENT, CONCAT_BASE);
        verify_fused(errors_f);
        if (errors_f == 0)
            $display("*** PASS *** lut_engine_int8 FUSED_CURRENT Part3 layout");
        else
            $display("*** FAIL *** lut_engine_int8 FUSED_CURRENT errors=%0d", errors_f);

        if (errors_h == 0 && errors_f == 0)
            $display("*** PASS *** lut_engine_int8 all checks");
        else
            $display("*** FAIL *** lut_engine_int8 total_errors=%0d", errors_h + errors_f);

        $finish;
    end

endmodule
