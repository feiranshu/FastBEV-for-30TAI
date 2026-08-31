//==============================================================================
// tb_group4_pipeline.sv
//   Phase F self-checking testbench for the group4 controller and stage arbiter.
//
//   This test uses stubbed engine done pulses so it can stress frame_phase,
//   hist_valid_mask, temporal_idx order, metadata clear, and DDR stage ownership
//   without requiring full-size Quant/LUT/SA memory contents.
//==============================================================================
`timescale 1ns/1ps

module tb_group4_pipeline;

    /* verilator lint_off UNUSEDSIGNAL */
    /* verilator lint_off SYNCASYNCNET */

    localparam integer STRESS_GROUPS = 1000;

    reg clk;
    reg rst_n;

    initial clk = 1'b0;
    always #5 clk = ~clk;

    reg         group_start;
    reg  [1:0]  comp_mode;
    reg  [2:0]  ext_mode;
    reg  [31:0] group_ctrl;

    reg         quant_done;
    reg         lut_done;
    reg         sa_done;
    reg  [31:0] error_status_w1c;
    reg         error_status_w1c_valid;

    wire        quant_start;
    wire        lut_start;
    wire        sa_start;
    wire        group_done;
    wire [1:0]  active_stage;
    wire [31:0] quant_fp32_base;
    wire [31:0] quant_int8_base;
    wire [31:0] quant_total_pixels;
    wire [31:0] lut_feat2d_base;
    wire [31:0] lut_dst_base;
    wire        lut_dst_mode;
    wire [31:0] sa_src_addr;
    wire [31:0] sa_concat_base;
    wire [1:0]  sa_temporal_idx;
    wire [31:0] group_status;
    wire [31:0] error_status;

    wire [2:0] status_hist   = group_status[7:5];
    wire [1:0] status_phase  = group_status[4:3];
    wire       status_ready  = group_status[2];
    wire       status_error  = group_status[0];

    integer errors;
    integer g;

    pipeline_ctrl_group4 dut_ctrl (
        .clk                    ( clk                   ),
        .rst_n                  ( rst_n                 ),
        .group_start            ( group_start           ),
        .comp_mode              ( comp_mode             ),
        .ext_mode               ( ext_mode              ),
        .group_ctrl             ( group_ctrl            ),
        .feat2d_fp32_base       ( 32'h0020_0000         ),
        .feat2d_int8_base       ( 32'h0140_0000         ),
        .concat_out_base        ( 32'h0380_0000         ),
        .lut_base_addr          ( 32'h0000_0000         ),
        .lut_size               ( 32'd8                 ),
        .frame0_addr            ( 32'h01A0_0000         ),
        .frame1_addr            ( 32'h0240_0000         ),
        .frame2_addr            ( 32'h02E0_0000         ),
        .frame_size             ( 32'd1024              ),
        .img_w                  ( 12'd1                 ),
        .img_h                  ( 12'd1                 ),
        .cameras                ( 8'd1                  ),
        .quant_done             ( quant_done            ),
        .lut_done               ( lut_done              ),
        .sa_done                ( sa_done               ),
        .error_status_w1c       ( error_status_w1c      ),
        .error_status_w1c_valid ( error_status_w1c_valid),
        .quant_start            ( quant_start           ),
        .lut_start              ( lut_start             ),
        .sa_start               ( sa_start              ),
        .group_done             ( group_done            ),
        .active_stage           ( active_stage          ),
        .quant_fp32_base        ( quant_fp32_base       ),
        .quant_int8_base        ( quant_int8_base       ),
        .quant_total_pixels     ( quant_total_pixels    ),
        .lut_feat2d_base        ( lut_feat2d_base       ),
        .lut_dst_base           ( lut_dst_base          ),
        .lut_dst_mode           ( lut_dst_mode          ),
        .sa_src_addr            ( sa_src_addr           ),
        .sa_concat_base         ( sa_concat_base        ),
        .sa_temporal_idx        ( sa_temporal_idx       ),
        .group_status           ( group_status          ),
        .error_status           ( error_status          )
    );

    reg  [31:0] quant_rd_addr;
    reg         quant_rd_req;
    wire        quant_rd_grant;
    wire [511:0] quant_rd_data;
    wire        quant_rd_data_valid;
    reg         quant_rd_data_ready;
    reg  [31:0] quant_wr_addr;
    reg  [511:0] quant_wr_data;
    reg         quant_wr_req;
    wire        quant_wr_grant;

    reg  [31:0] lut_rd_addr;
    reg         lut_rd_req;
    wire        lut_rd_grant;
    wire [511:0] lut_rd_data;
    wire        lut_rd_data_valid;
    reg         lut_rd_data_ready;
    reg  [31:0] lut_wr_addr;
    reg  [511:0] lut_wr_data;
    reg         lut_wr_req;
    wire        lut_wr_grant;

    reg  [31:0] sa_rd_addr;
    reg         sa_rd_req;
    wire        sa_rd_grant;
    wire [511:0] sa_rd_data;
    wire        sa_rd_data_valid;
    reg         sa_rd_data_ready;
    reg  [31:0] sa_wr_addr;
    reg  [511:0] sa_wr_data;
    reg         sa_wr_req;
    wire        sa_wr_grant;

    wire [31:0] ddr_araddr;
    wire        ddr_arvalid;
    reg         ddr_arready;
    reg  [511:0] ddr_rdata;
    reg         ddr_rvalid;
    wire        ddr_rready;
    wire [31:0] ddr_awaddr;
    wire [511:0] ddr_awdata;
    wire        ddr_awvalid;
    reg         ddr_awready;

    dma_arbiter_stage dut_arb (
        .active_stage          ( active_stage          ),
        .quant_rd_addr         ( quant_rd_addr         ),
        .quant_rd_req          ( quant_rd_req          ),
        .quant_rd_grant        ( quant_rd_grant        ),
        .quant_rd_data         ( quant_rd_data         ),
        .quant_rd_data_valid   ( quant_rd_data_valid   ),
        .quant_rd_data_ready   ( quant_rd_data_ready   ),
        .quant_wr_addr         ( quant_wr_addr         ),
        .quant_wr_data         ( quant_wr_data         ),
        .quant_wr_req          ( quant_wr_req          ),
        .quant_wr_grant        ( quant_wr_grant        ),
        .lut_rd_addr           ( lut_rd_addr           ),
        .lut_rd_req            ( lut_rd_req            ),
        .lut_rd_grant          ( lut_rd_grant          ),
        .lut_rd_data           ( lut_rd_data           ),
        .lut_rd_data_valid     ( lut_rd_data_valid     ),
        .lut_rd_data_ready     ( lut_rd_data_ready     ),
        .lut_wr_addr           ( lut_wr_addr           ),
        .lut_wr_data           ( lut_wr_data           ),
        .lut_wr_req            ( lut_wr_req            ),
        .lut_wr_grant          ( lut_wr_grant          ),
        .sa_rd_addr            ( sa_rd_addr            ),
        .sa_rd_req             ( sa_rd_req             ),
        .sa_rd_grant           ( sa_rd_grant           ),
        .sa_rd_data            ( sa_rd_data            ),
        .sa_rd_data_valid      ( sa_rd_data_valid      ),
        .sa_rd_data_ready      ( sa_rd_data_ready      ),
        .sa_wr_addr            ( sa_wr_addr            ),
        .sa_wr_data            ( sa_wr_data            ),
        .sa_wr_req             ( sa_wr_req             ),
        .sa_wr_grant           ( sa_wr_grant           ),
        .ddr_araddr            ( ddr_araddr            ),
        .ddr_arvalid           ( ddr_arvalid           ),
        .ddr_arready           ( ddr_arready           ),
        .ddr_rdata             ( ddr_rdata             ),
        .ddr_rvalid            ( ddr_rvalid            ),
        .ddr_rready            ( ddr_rready            ),
        .ddr_awaddr            ( ddr_awaddr            ),
        .ddr_awdata            ( ddr_awdata            ),
        .ddr_awvalid           ( ddr_awvalid           ),
        .ddr_awready           ( ddr_awready           ),
        .clk                   ( clk                   ),
        .rst_n                 ( rst_n                 )
    );

    reg [15:0] lfsr;
    reg        read_pending;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            lfsr         <= 16'hACE1;
            ddr_arready  <= 1'b0;
            ddr_awready  <= 1'b0;
            ddr_rvalid   <= 1'b0;
            ddr_rdata    <= 512'd0;
            read_pending <= 1'b0;
        end else begin
            lfsr        <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
            ddr_arready <= lfsr[0] | lfsr[3];
            ddr_awready <= lfsr[1] | lfsr[4];

            if(ddr_arvalid && ddr_arready)
                read_pending <= 1'b1;
            else if(ddr_rvalid && ddr_rready)
                read_pending <= 1'b0;

            ddr_rvalid <= read_pending;
            if(read_pending)
                ddr_rdata <= {16{ddr_araddr}};
        end
    end

    always @(*) begin
        quant_rd_addr = 32'h1000_0000;
        quant_rd_req  = (active_stage != 2'd0);
        quant_rd_data_ready = 1'b1;
        quant_wr_addr = 32'h1000_0040;
        quant_wr_data = {16{32'h5155_AA10}};
        quant_wr_req  = (active_stage != 2'd0);

        lut_rd_addr = 32'h2000_0000;
        lut_rd_req  = (active_stage != 2'd0);
        lut_rd_data_ready = 1'b1;
        lut_wr_addr = 32'h2000_0040;
        lut_wr_data = {16{32'h5155_AA20}};
        lut_wr_req  = (active_stage != 2'd0);

        sa_rd_addr = 32'h3000_0000;
        sa_rd_req  = (active_stage != 2'd0);
        sa_rd_data_ready = 1'b1;
        sa_wr_addr = 32'h3000_0040;
        sa_wr_data = {16{32'h5155_AA30}};
        sa_wr_req  = (active_stage != 2'd0);
    end

    task fail;
        input [511:0] msg;
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
            if(got !== exp) begin
                errors = errors + 1;
                $display("[FAIL] %0s got=0x%08h exp=0x%08h", msg, got, exp);
            end
        end
    endtask

    task issue_start;
        begin
            @(posedge clk);
            group_start = 1'b1;
            @(posedge clk);
            group_start = 1'b0;
        end
    endtask

    task pulse_quant_done_after;
        input integer cycles;
        begin
            repeat(cycles) @(posedge clk);
            quant_done = 1'b1;
            @(posedge clk);
            quant_done = 1'b0;
        end
    endtask

    task pulse_lut_done_after;
        input integer cycles;
        begin
            repeat(cycles) @(posedge clk);
            lut_done = 1'b1;
            @(posedge clk);
            lut_done = 1'b0;
        end
    endtask

    task pulse_sa_done_after;
        input integer cycles;
        begin
            repeat(cycles) @(posedge clk);
            sa_done = 1'b1;
            @(posedge clk);
            sa_done = 1'b0;
        end
    endtask

    task check_arbiter_owner;
        begin
            if(active_stage == 2'd1) begin
                if(lut_rd_grant || lut_wr_grant || sa_rd_grant || sa_wr_grant)
                    fail("non-quant grant during quant stage");
            end else if(active_stage == 2'd2) begin
                if(quant_rd_grant || quant_wr_grant || sa_rd_grant || sa_wr_grant)
                    fail("non-lut grant during lut stage");
            end else if(active_stage == 2'd3) begin
                if(quant_rd_grant || quant_wr_grant || lut_rd_grant || lut_wr_grant)
                    fail("non-sa grant during sa stage");
            end else begin
                if(quant_rd_grant || quant_wr_grant || lut_rd_grant || lut_wr_grant ||
                   sa_rd_grant || sa_wr_grant)
                    fail("grant while no stage active");
            end
        end
    endtask

    task run_one_frame;
        input [1:0] expected_phase;
        begin
            issue_start();

            wait(quant_start);
            check_eq({30'd0, active_stage}, 32'd1, "quant active_stage");
            check_eq(quant_fp32_base, 32'h0020_0000, "quant fp32 base");
            check_eq(quant_int8_base, 32'h0140_0000, "quant int8 base");
            check_eq(quant_total_pixels, 32'd1, "quant total pixels");
            pulse_quant_done_after(2);

            wait(lut_start);
            check_eq({30'd0, active_stage}, 32'd2, "lut active_stage");
            check_eq(lut_feat2d_base, 32'h0140_0000, "lut int8 source base");
            if(expected_phase == 2'd0) begin
                check_eq(lut_dst_base, 32'h01A0_0000, "frame1 history dst");
                check_eq({31'd0, lut_dst_mode}, 32'd0, "frame1 history mode");
            end else if(expected_phase == 2'd1) begin
                check_eq(lut_dst_base, 32'h0240_0000, "frame2 history dst");
                check_eq({31'd0, lut_dst_mode}, 32'd0, "frame2 history mode");
            end else if(expected_phase == 2'd2) begin
                check_eq(lut_dst_base, 32'h02E0_0000, "frame3 history dst");
                check_eq({31'd0, lut_dst_mode}, 32'd0, "frame3 history mode");
            end else begin
                check_eq(lut_dst_base, 32'h0380_0000, "frame4 concat dst");
                check_eq({31'd0, lut_dst_mode}, 32'd1, "frame4 fused current mode");
            end
            pulse_lut_done_after(2);

            if(expected_phase == 2'd3) begin
                wait(sa_start);
                check_eq({30'd0, active_stage}, 32'd3, "sa0 active_stage");
                check_eq({30'd0, sa_temporal_idx}, 32'd3, "sa0 temporal");
                check_eq(sa_src_addr, 32'h01A0_0000, "sa0 source");
                check_eq(sa_concat_base, 32'h0380_0000, "sa concat");
                pulse_sa_done_after(2);

                wait(sa_start);
                check_eq({30'd0, active_stage}, 32'd3, "sa1 active_stage");
                check_eq({30'd0, sa_temporal_idx}, 32'd2, "sa1 temporal");
                check_eq(sa_src_addr, 32'h0240_0000, "sa1 source");
                pulse_sa_done_after(2);

                wait(sa_start);
                check_eq({30'd0, active_stage}, 32'd3, "sa2 active_stage");
                check_eq({30'd0, sa_temporal_idx}, 32'd1, "sa2 temporal");
                check_eq(sa_src_addr, 32'h02E0_0000, "sa2 source");
                pulse_sa_done_after(2);
            end

            wait(group_done);
            @(posedge clk);
            if(expected_phase == 2'd0) begin
                check_eq({30'd0, status_phase}, 32'd1, "phase after frame1");
                check_eq({29'd0, status_hist}, 32'b001, "hist after frame1");
                check_eq({31'd0, status_ready}, 32'd0, "ready after frame1");
            end else if(expected_phase == 2'd1) begin
                check_eq({30'd0, status_phase}, 32'd2, "phase after frame2");
                check_eq({29'd0, status_hist}, 32'b011, "hist after frame2");
                check_eq({31'd0, status_ready}, 32'd0, "ready after frame2");
            end else if(expected_phase == 2'd2) begin
                check_eq({30'd0, status_phase}, 32'd3, "phase after frame3");
                check_eq({29'd0, status_hist}, 32'b111, "hist after frame3");
                check_eq({31'd0, status_ready}, 32'd0, "ready after frame3");
            end else begin
                check_eq({30'd0, status_phase}, 32'd0, "phase after frame4");
                check_eq({29'd0, status_hist}, 32'b000, "hist cleared after frame4");
                check_eq({31'd0, status_ready}, 32'd1, "ready after frame4");
            end
            check_eq({31'd0, status_error}, 32'd0, "status error clear");
        end
    endtask

    always @(posedge clk) begin
        if(rst_n)
            check_arbiter_owner();
    end

    initial begin
        errors = 0;
        group_start = 1'b0;
        comp_mode = 2'b11;
        ext_mode = 3'd5;
        group_ctrl = 32'h0000_0004;
        quant_done = 1'b0;
        lut_done = 1'b0;
        sa_done = 1'b0;
        error_status_w1c = 32'd0;
        error_status_w1c_valid = 1'b0;
        rst_n = 1'b0;

        repeat(5) @(posedge clk);
        rst_n = 1'b1;
        repeat(2) @(posedge clk);

        check_eq({30'd0, status_phase}, 32'd0, "reset phase");
        check_eq({29'd0, status_hist}, 32'd0, "reset hist");

        for(g = 0; g < STRESS_GROUPS; g = g + 1) begin
            run_one_frame(2'd0);
            run_one_frame(2'd1);
            run_one_frame(2'd2);
            run_one_frame(2'd3);
        end

        if(errors == 0)
            $display("*** PASS *** group4 pipeline phase/stage/arbiter stress groups=%0d", STRESS_GROUPS);
        else
            $display("*** FAIL *** group4 pipeline errors=%0d", errors);

        #100;
        $finish;
    end

    initial begin
        #20000000;
        $display("[TIMEOUT] group4 pipeline simulation timeout");
        $finish;
    end

endmodule
