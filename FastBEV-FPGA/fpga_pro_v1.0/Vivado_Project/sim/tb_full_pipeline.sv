//==============================================================================
// tb_bev_accel_top_phaseF.sv
//   Phase F compact full-pipeline numerical test for INT8 group4.
//
//   Scope:
//     - Program bev_accel_top through the public register interface.
//     - Run four group4 starts with real Quant -> LUT -> SA datapath.
//     - Use a compact 1 camera, 3x3 image/BEV, 1 z-slice data set.
//     - Check final Part3 physical layout:
//         [N][Z][C/32][X][Y][C%32]
//     - Exercise deterministic DDR read/write backpressure and read back
//       performance counters after the fourth frame.
//
//   Notes:
//     - One LUT voxel is intentionally invalid to verify zero-fill.
//     - SA identity alignment writes zero on the right/bottom border because
//       the bilinear engine requires a valid 2x2 source neighborhood.
//==============================================================================
`timescale 1ns/1ps

module tb_bev_accel_top_phaseF;

    localparam CTRL_START_ADDR       = 16'h00;
    localparam LUT_BASE_ADDR         = 16'h01;
    localparam LUT_SIZE_ADDR         = 16'h02;
    localparam BEV_PARAMS_ADDR       = 16'h09;
    localparam IMG_PARAMS_ADDR       = 16'h0A;
    localparam FRAME0_ADDR           = 16'h11;
    localparam FRAME1_ADDR           = 16'h12;
    localparam FRAME2_ADDR           = 16'h13;
    localparam FRAME_SIZE_ADDR       = 16'h15;
    localparam COMP_DONE_ADDR        = 16'h20;
    localparam PERF_CNT_LO_ADDR      = 16'h22;
    localparam EXT_MODE_ADDR         = 16'h24;
    localparam FEAT2D_FP32_ADDR      = 16'h25;
    localparam FEAT2D_INT8_ADDR      = 16'h26;
    localparam CONCAT_OUT_ADDR       = 16'h27;
    localparam GROUP_STATUS_ADDR     = 16'h28;
    localparam ERROR_STATUS_ADDR     = 16'h2A;
    localparam PERF_STAGE_SEL_ADDR   = 16'h2B;

    localparam FEAT2D_FP32_BASE      = 32'h0000_0000;
    localparam LUT_BASE              = 32'h0000_4000;
    localparam FEAT2D_INT8_BASE      = 32'h0000_8000;
    localparam HIST0_BASE            = 32'h0001_0000;
    localparam HIST1_BASE            = 32'h0001_1000;
    localparam HIST2_BASE            = 32'h0001_2000;
    localparam CONCAT_BASE           = 32'h0001_4000;
    localparam FRAME_SIZE_BYTES      = 32'h0000_1000;

    localparam MEM_WORDS             = 4096;
    localparam FRAMES                = 4;
    localparam BEV_X                 = 3;
    localparam BEV_Y                 = 3;
    localparam BEV_Z                 = 1;
    localparam IMG_W                 = 3;
    localparam IMG_H                 = 3;
    localparam CAMERAS               = 1;
    localparam VOXELS                = BEV_X * BEV_Y * BEV_Z;
    localparam TOTAL_PIXELS          = CAMERAS * IMG_W * IMG_H;
    localparam FINAL_CGROUP_BYTES    = BEV_X * BEV_Y * 32;
    localparam FINAL_CONCAT_BYTES    = BEV_Z * 8 * FINAL_CGROUP_BYTES;
    localparam GOLD_WORDS            = FRAMES * VOXELS * 64;
    localparam real SCALE            = 0.06905783;

    reg clk;
    reg ra_clk;
    reg rst_n;
    reg ra_rst_n;

    initial clk = 1'b0;
    always #5 clk = ~clk;

    initial ra_clk = 1'b0;
    always #7 ra_clk = ~ra_clk;

    reg  [31:0] itf_ra_awaddr;
    reg  [31:0] itf_ra_awdata;
    reg         itf_ra_awvalid;
    wire        itf_ra_awready;
    reg  [31:0] itf_ra_araddr;
    reg         itf_ra_arvalid;
    wire        itf_ra_arready;
    wire [31:0] itf_ra_rdata;
    wire        itf_ra_rvalid;
    reg         itf_ra_rready;

    wire [31:0]  itf_awaddr;
    wire [511:0] itf_awdata;
    wire         itf_awvalid;
    reg          itf_awready;
    wire [31:0]  itf_araddr;
    wire         itf_arvalid;
    reg          itf_arready;
    reg  [511:0] itf_rdata;
    reg          itf_rvalid;
    wire         itf_rready;
    wire         reset_reg;

    integer errors;
    integer cycle_count;
    integer hist0_writes;
    integer hist1_writes;
    integer hist2_writes;
    integer concat_writes;
    integer int8_temp_writes;
    integer ddr_rd_waits;
    integer ddr_wr_waits;
    integer ddr_rd_grants;
    integer ddr_wr_grants;
    integer i;
    integer frame;
    integer x;
    integer y;
    integer z;
    integer ch;
    integer pix;
    integer mismatches;
    integer gold [0:GOLD_WORDS-1];
    reg [31:0] perf_total;
    reg [31:0] perf_quant;
    reg [31:0] perf_lut;
    reg [31:0] perf_sa;
    reg [31:0] perf_rd_wait;
    reg [31:0] perf_wr_wait;
    reg [31:0] perf_rd_grant;
    reg [31:0] perf_wr_grant;

    reg [511:0] mem [0:MEM_WORDS-1];
    reg [31:0]  read_addr_q;
    reg [2:0]   read_delay_q;
    reg         read_pending;
    reg [15:0]  lfsr;

    bev_accel_top dut (
        .itf_ra_awaddr  ( itf_ra_awaddr  ),
        .itf_ra_awdata  ( itf_ra_awdata  ),
        .itf_ra_awvalid ( itf_ra_awvalid ),
        .itf_ra_awready ( itf_ra_awready ),
        .itf_ra_araddr  ( itf_ra_araddr  ),
        .itf_ra_arvalid ( itf_ra_arvalid ),
        .itf_ra_arready ( itf_ra_arready ),
        .itf_ra_rdata   ( itf_ra_rdata   ),
        .itf_ra_rvalid  ( itf_ra_rvalid  ),
        .itf_ra_rready  ( itf_ra_rready  ),
        .itf_awaddr     ( itf_awaddr     ),
        .itf_awdata     ( itf_awdata     ),
        .itf_awvalid    ( itf_awvalid    ),
        .itf_awready    ( itf_awready    ),
        .itf_araddr     ( itf_araddr     ),
        .itf_arvalid    ( itf_arvalid    ),
        .itf_arready    ( itf_arready    ),
        .itf_rdata      ( itf_rdata      ),
        .itf_rvalid     ( itf_rvalid     ),
        .itf_rready     ( itf_rready     ),
        .reset_reg      ( reset_reg      ),
        .clk            ( clk            ),
        .ra_clk         ( ra_clk         ),
        .rst_n          ( rst_n          ),
        .ra_rst_n       ( ra_rst_n       )
    );

    function integer mem_index;
        input [31:0] addr;
        begin
            mem_index = addr[17:6];
        end
    endfunction

    function [63:0] lut_entry;
        input signed [15:0] cam;
        input [15:0] u;
        input [15:0] v;
        begin
            lut_entry = {16'd0, v, u, cam[15:0]};
        end
    endfunction

    function is_addr_in_range;
        input [31:0] addr;
        input [31:0] base;
        input [31:0] bytes;
        begin
            is_addr_in_range = (addr >= base) && (addr < (base + bytes));
        end
    endfunction

    function integer part3_addr;
        input integer z_i;
        input integer x_i;
        input integer y_i;
        input integer c_i;
        integer group_i;
        integer xy_i;
        begin
            group_i = z_i * 8 + (c_i / 32);
            xy_i = ((group_i * BEV_X + x_i) * BEV_Y + y_i);
            part3_addr = CONCAT_BASE + xy_i * 32 + (c_i % 32);
        end
    endfunction

    function [7:0] get_mem_byte;
        input [31:0] addr;
        begin
            get_mem_byte = mem[mem_index(addr)][addr[5:0] * 8 +: 8];
        end
    endfunction

    function automatic integer golden_quant;
        input [31:0] b;
        reg [7:0] e;
        reg [22:0] m;
        shortreal sf;
        real v;
        real qf;
        integer q;
        begin
            e = b[30:23];
            m = b[22:0];
            if (e == 8'hFF) begin
                if (m != 0)
                    golden_quant = 0;
                else
                    golden_quant = b[31] ? -128 : 127;
            end else begin
                sf = $bitstoshortreal(b);
                v  = sf;
                qf = v / SCALE;
                if (qf >= 0.0)
                    q = $rtoi(qf + 0.5);
                else
                    q = $rtoi(qf - 0.5);
                if (q > 127)
                    q = 127;
                if (q < -128)
                    q = -128;
                golden_quant = q;
            end
        end
    endfunction

    function automatic [31:0] make_fp32_bits;
        input integer frame_i;
        input integer pix_i;
        input integer ch_i;
        integer q_target;
        real val_real;
        shortreal val_short;
        begin
            q_target = ((frame_i * 41 + pix_i * 17 + ch_i * 7) % 101) - 50;
            if ((ch_i % 19) == 0)
                q_target = q_target / 2;
            val_real = q_target * SCALE;
            val_short = val_real;
            make_fp32_bits = $shortrealtobits(val_short);
        end
    endfunction

    function lut_valid_xy;
        input integer x_i;
        input integer y_i;
        begin
            lut_valid_xy = !((x_i == 1) && (y_i == 1));
        end
    endfunction

    function sa_identity_valid_xy;
        input integer x_i;
        input integer y_i;
        begin
            sa_identity_valid_xy = (x_i < (BEV_X - 1)) && (y_i < (BEV_Y - 1));
        end
    endfunction

    function integer gold_index;
        input integer frame_i;
        input integer pix_i;
        input integer ch_i;
        begin
            gold_index = frame_i * VOXELS * 64 + pix_i * 64 + ch_i;
        end
    endfunction

    task fail;
        input [255:0] msg;
        begin
            errors = errors + 1;
            $display("[FAIL] %0s", msg);
        end
    endtask

    task check_eq;
        input [31:0] got;
        input [31:0] exp;
        input [255:0] msg;
        begin
            if (got !== exp) begin
                errors = errors + 1;
                $display("[FAIL] %0s got=0x%08h exp=0x%08h", msg, got, exp);
            end
        end
    endtask

    task check_ge;
        input integer got;
        input integer exp;
        input [255:0] msg;
        begin
            if (got < exp) begin
                errors = errors + 1;
                $display("[FAIL] %0s got=%0d exp>=%0d", msg, got, exp);
            end
        end
    endtask

    task set_word32;
        input [31:0] addr;
        input [31:0] data;
        begin
            mem[mem_index(addr)][addr[5:2] * 32 +: 32] = data;
        end
    endtask

    task set_lut_entry;
        input integer voxel_i;
        input [63:0] entry;
        reg [31:0] addr;
        begin
            addr = LUT_BASE + voxel_i * 8;
            mem[mem_index(addr)][addr[5:0] * 8 +: 64] = entry;
        end
    endtask

    task ra_write;
        input [15:0] word_addr;
        input [31:0] data;
        begin
            @(posedge ra_clk);
            itf_ra_awaddr  <= {14'd0, word_addr, 2'b00};
            itf_ra_awdata  <= data;
            itf_ra_awvalid <= 1'b1;
            @(posedge ra_clk);
            while (!itf_ra_awready)
                @(posedge ra_clk);
            itf_ra_awvalid <= 1'b0;
            itf_ra_awaddr  <= 32'd0;
            itf_ra_awdata  <= 32'd0;
        end
    endtask

    task ra_read;
        input  [15:0] word_addr;
        output [31:0] data;
        begin
            @(posedge ra_clk);
            itf_ra_araddr  <= {14'd0, word_addr, 2'b00};
            itf_ra_arvalid <= 1'b1;
            itf_ra_rready  <= 1'b0;
            @(posedge ra_clk);
            while (!itf_ra_arready)
                @(posedge ra_clk);
            itf_ra_arvalid <= 1'b0;
            wait (itf_ra_rvalid == 1'b1);
            data = itf_ra_rdata;
            @(posedge ra_clk);
            itf_ra_rready <= 1'b1;
            @(posedge ra_clk);
            itf_ra_rready <= 1'b0;
            itf_ra_araddr <= 32'd0;
        end
    endtask

    task start_and_wait_done;
        input [255:0] tag;
        integer polls;
        reg [31:0] done_val;
        begin
            ra_write(CTRL_START_ADDR, 32'h0000_0001);
            repeat (6) @(posedge ra_clk);
            done_val = 32'd0;
            polls = 0;
            while (done_val[0] == 1'b0 && polls < 50000) begin
                ra_read(COMP_DONE_ADDR, done_val);
                polls = polls + 1;
            end
            if (done_val[0] != 1'b1) begin
                errors = errors + 1;
                $display("[FAIL] timeout waiting COMP_DONE for %0s", tag);
            end else begin
                $display("[INFO] %0s done after %0d polls", tag, polls);
            end
        end
    endtask

    task read_perf;
        input [3:0] sel;
        output [31:0] value;
        begin
            ra_write(PERF_STAGE_SEL_ADDR, {28'd0, sel});
            ra_read(PERF_CNT_LO_ADDR, value);
        end
    endtask

    task check_group_status;
        input [1:0] exp_phase;
        input [2:0] exp_hist;
        input       exp_ready;
        input [255:0] msg;
        reg [31:0] status;
        reg [31:0] err_status;
        begin
            ra_read(GROUP_STATUS_ADDR, status);
            ra_read(ERROR_STATUS_ADDR, err_status);
            check_eq({30'd0, status[4:3]}, {30'd0, exp_phase}, msg);
            check_eq({29'd0, status[7:5]}, {29'd0, exp_hist}, msg);
            check_eq({31'd0, status[2]}, {31'd0, exp_ready}, msg);
            check_eq({31'd0, status[0]}, 32'd0, msg);
            check_eq(err_status, 32'd0, msg);
        end
    endtask

    task program_group4_registers;
        begin
            ra_write(LUT_BASE_ADDR,       LUT_BASE);
            ra_write(LUT_SIZE_ADDR,       VOXELS);
            ra_write(BEV_PARAMS_ADDR,     {8'd1, 8'd3, 8'd3, 8'd64});
            ra_write(IMG_PARAMS_ADDR,     {8'd1, 12'd3, 12'd3});
            ra_write(FRAME0_ADDR,         HIST0_BASE);
            ra_write(FRAME1_ADDR,         HIST1_BASE);
            ra_write(FRAME2_ADDR,         HIST2_BASE);
            ra_write(FRAME_SIZE_ADDR,     FRAME_SIZE_BYTES);
            ra_write(EXT_MODE_ADDR,       32'd1);
            ra_write(FEAT2D_FP32_ADDR,    FEAT2D_FP32_BASE);
            ra_write(FEAT2D_INT8_ADDR,    FEAT2D_INT8_BASE);
            ra_write(CONCAT_OUT_ADDR,     CONCAT_BASE);
        end
    endtask

    task init_lut;
        integer voxel_i;
        integer vx;
        integer vy;
        begin
            for (voxel_i = 0; voxel_i < 16; voxel_i = voxel_i + 1)
                set_lut_entry(voxel_i, 64'd0);

            for (voxel_i = 0; voxel_i < VOXELS; voxel_i = voxel_i + 1) begin
                vx = voxel_i % BEV_X;
                vy = (voxel_i / BEV_X) % BEV_Y;
                if (lut_valid_xy(vx, vy))
                    set_lut_entry(voxel_i, lut_entry(16'sd0, vx, vy));
                else
                    set_lut_entry(voxel_i, lut_entry(-16'sd1, 16'd0, 16'd0));
            end
        end
    endtask

    task load_frame_fp32;
        input integer frame_i;
        integer pix_i;
        integer ch_i;
        reg [31:0] bits;
        begin
            for (pix_i = 0; pix_i < TOTAL_PIXELS; pix_i = pix_i + 1) begin
                for (ch_i = 0; ch_i < 64; ch_i = ch_i + 1) begin
                    bits = make_fp32_bits(frame_i, pix_i, ch_i);
                    set_word32(FEAT2D_FP32_BASE + pix_i * 256 + ch_i * 4, bits);
                    gold[gold_index(frame_i, pix_i, ch_i)] = golden_quant(bits);
                end
            end
        end
    endtask

    task init_memory;
        begin
            for (i = 0; i < MEM_WORDS; i = i + 1)
                mem[i] = {16{32'hA5A5_A5A5}};
            for (i = 0; i < GOLD_WORDS; i = i + 1)
                gold[i] = 0;
            init_lut();
        end
    endtask

    task verify_final_layout;
        integer logical_c;
        integer frame_i;
        integer ch_i;
        integer pix_i;
        integer exp_i;
        reg [7:0] got_b;
        reg [7:0] exp_b;
        begin
            mismatches = 0;
            for (z = 0; z < BEV_Z; z = z + 1) begin
                for (x = 0; x < BEV_X; x = x + 1) begin
                    for (y = 0; y < BEV_Y; y = y + 1) begin
                        pix_i = y * BEV_X + x;
                        for (logical_c = 0; logical_c < 256; logical_c = logical_c + 1) begin
                            frame_i = 3 - logical_c / 64;
                            ch_i    = logical_c % 64;
                            if (frame_i < 3) begin
                                if (lut_valid_xy(x, y) && sa_identity_valid_xy(x, y))
                                    exp_i = gold[gold_index(frame_i, pix_i, ch_i)];
                                else
                                    exp_i = 0;
                            end else begin
                                if (lut_valid_xy(x, y))
                                    exp_i = gold[gold_index(frame_i, pix_i, ch_i)];
                                else
                                    exp_i = 0;
                            end

                            got_b = get_mem_byte(part3_addr(z, x, y, logical_c));
                            exp_b = exp_i[7:0];
                            if (got_b !== exp_b) begin
                                if (mismatches < 24) begin
                                    $display("[MISMATCH] z=%0d x=%0d y=%0d c=%0d got=%0d exp=%0d addr=0x%08h",
                                             z, x, y, logical_c, $signed(got_b),
                                             $signed(exp_b), part3_addr(z, x, y, logical_c));
                                end
                                mismatches = mismatches + 1;
                            end
                        end
                    end
                end
            end

            if (mismatches != 0) begin
                errors = errors + 1;
                $display("[FAIL] final Part3 layout mismatches=%0d", mismatches);
            end else begin
                $display("[INFO] final Part3 layout numerical check passed");
            end
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cycle_count  <= 0;
            itf_arready  <= 1'b0;
            itf_awready  <= 1'b0;
            itf_rvalid   <= 1'b0;
            itf_rdata    <= 512'd0;
            read_addr_q  <= 32'd0;
            read_delay_q <= 3'd0;
            read_pending <= 1'b0;
            lfsr         <= 16'hACE1;
        end else begin
            cycle_count <= cycle_count + 1;
            lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};

            itf_arready <= !read_pending && !itf_rvalid &&
                           (lfsr[0] || lfsr[3] || (cycle_count[2:0] == 3'd0));
            itf_awready <= (lfsr[1] || lfsr[4] || (cycle_count[3:0] == 4'hF));

            if (itf_arvalid && !itf_arready)
                ddr_rd_waits <= ddr_rd_waits + 1;
            if (itf_awvalid && !itf_awready)
                ddr_wr_waits <= ddr_wr_waits + 1;

            if (itf_arvalid && itf_arready) begin
                read_addr_q  <= itf_araddr;
                read_delay_q <= {1'b0, lfsr[6:5]};
                read_pending <= 1'b1;
                ddr_rd_grants <= ddr_rd_grants + 1;
            end

            if (!itf_rvalid && read_pending) begin
                if (read_delay_q == 3'd0) begin
                    itf_rdata    <= mem[mem_index(read_addr_q)];
                    itf_rvalid   <= 1'b1;
                    read_pending <= 1'b0;
                end else begin
                    read_delay_q <= read_delay_q - 1'b1;
                end
            end else if (itf_rvalid && itf_rready) begin
                itf_rvalid <= 1'b0;
            end

            if (itf_awvalid && itf_awready) begin
                mem[mem_index(itf_awaddr)] <= itf_awdata;
                ddr_wr_grants <= ddr_wr_grants + 1;

                if (is_addr_in_range(itf_awaddr, FEAT2D_INT8_BASE, TOTAL_PIXELS * 64))
                    int8_temp_writes <= int8_temp_writes + 1;
                if (is_addr_in_range(itf_awaddr, HIST0_BASE, VOXELS * 64))
                    hist0_writes <= hist0_writes + 1;
                if (is_addr_in_range(itf_awaddr, HIST1_BASE, VOXELS * 64))
                    hist1_writes <= hist1_writes + 1;
                if (is_addr_in_range(itf_awaddr, HIST2_BASE, VOXELS * 64))
                    hist2_writes <= hist2_writes + 1;
                if (is_addr_in_range(itf_awaddr, CONCAT_BASE, FINAL_CONCAT_BYTES))
                    concat_writes <= concat_writes + 1;
            end
        end
    end

    initial begin
        errors = 0;
        hist0_writes = 0;
        hist1_writes = 0;
        hist2_writes = 0;
        concat_writes = 0;
        int8_temp_writes = 0;
        ddr_rd_waits = 0;
        ddr_wr_waits = 0;
        ddr_rd_grants = 0;
        ddr_wr_grants = 0;

        itf_ra_awaddr = 32'd0;
        itf_ra_awdata = 32'd0;
        itf_ra_awvalid = 1'b0;
        itf_ra_araddr = 32'd0;
        itf_ra_arvalid = 1'b0;
        itf_ra_rready = 1'b0;

        init_memory();

        rst_n = 1'b0;
        ra_rst_n = 1'b0;
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge ra_clk);
        ra_rst_n = 1'b1;
        repeat (20) @(posedge ra_clk);

        program_group4_registers();
        check_group_status(2'd0, 3'b000, 1'b0, "initial group status");

        for (frame = 0; frame < FRAMES; frame = frame + 1) begin
            load_frame_fp32(frame);
            case (frame)
                0: begin
                    start_and_wait_done("frame1 history");
                    check_group_status(2'd1, 3'b001, 1'b0, "after frame1");
                end
                1: begin
                    start_and_wait_done("frame2 history");
                    check_group_status(2'd2, 3'b011, 1'b0, "after frame2");
                end
                2: begin
                    start_and_wait_done("frame3 history");
                    check_group_status(2'd3, 3'b111, 1'b0, "after frame3");
                end
                default: begin
                    start_and_wait_done("frame4 current+SA");
                    check_group_status(2'd0, 3'b000, 1'b1, "after frame4");
                end
            endcase
        end

        check_ge(int8_temp_writes, TOTAL_PIXELS * FRAMES, "quant temp writes");
        check_ge(hist0_writes, VOXELS, "history slot 0 writes");
        check_ge(hist1_writes, VOXELS, "history slot 1 writes");
        check_ge(hist2_writes, VOXELS, "history slot 2 writes");
        check_ge(concat_writes, VOXELS * 8, "concat Part3 RMW writes");
        check_ge(ddr_rd_waits, 1, "DDR read backpressure observed");
        check_ge(ddr_wr_waits, 1, "DDR write backpressure observed");

        verify_final_layout();

        read_perf(4'h0, perf_total);
        read_perf(4'h1, perf_quant);
        read_perf(4'h2, perf_lut);
        read_perf(4'h3, perf_sa);
        read_perf(4'h4, perf_rd_wait);
        read_perf(4'h5, perf_wr_wait);
        read_perf(4'h6, perf_rd_grant);
        read_perf(4'h7, perf_wr_grant);

        check_ge(perf_total, 1, "perf total cycles");
        check_ge(perf_quant, 1, "perf quant cycles");
        check_ge(perf_lut, 1, "perf LUT cycles");
        check_ge(perf_sa, 1, "perf SA cycles");
        check_ge(perf_rd_grant, 1, "perf DDR read grants");
        check_ge(perf_wr_grant, 1, "perf DDR write grants");

        $display("[INFO] perf total=%0d quant=%0d lut=%0d sa=%0d rd_wait=%0d wr_wait=%0d rd_grant=%0d wr_grant=%0d",
                 perf_total, perf_quant, perf_lut, perf_sa,
                 perf_rd_wait, perf_wr_wait, perf_rd_grant, perf_wr_grant);

        if (errors == 0) begin
            $display("*** PASS *** Phase F bev_accel_top full pipeline numerical/layout/backpressure/perf");
        end else begin
            $display("*** FAIL *** Phase F full pipeline errors=%0d", errors);
            $fatal(1);
        end

        #1000;
        $finish;
    end

    initial begin
        #50000000;
        $display("[TIMEOUT] Phase F bev_accel_top full pipeline timeout");
        $fatal(1);
    end

endmodule
