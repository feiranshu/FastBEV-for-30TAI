`timescale 1ns/1ps

module tb_bev_alert_regs;

    reg         clk;
    reg         rst_n;
    reg [15:0]  ra_awaddr;
    reg [31:0]  ra_awdata;
    reg         ra_awvalid;
    wire        ra_awready;
    reg [15:0]  ra_araddr;
    reg         ra_arvalid;
    wire        ra_arready;
    wire [31:0] ra_rdata;
    wire        ra_rvalid;
    reg         ra_rready;

    wire        comp_start;
    wire [1:0]  comp_mode;
    wire        frame_shift_en;
    reg         comp_done;
    wire [31:0] lut_base_addr;
    wire [31:0] lut_size;
    wire [31:0] feat2d_base_addr;
    wire [31:0] feat3d_wr_addr;
    wire [31:0] feat3d_wr_size;
    wire [7:0]  bev_channels;
    wire [7:0]  bev_x;
    wire [7:0]  bev_y;
    wire [7:0]  bev_z;
    wire [11:0] img_w;
    wire [11:0] img_h;
    wire [7:0]  cameras;

    wire [1:0]  alert_level;
    wire        alert_trigger;
    wire        alert_clear;
    wire [31:0] alert_duration_ms;
    wire [31:0] alert_danger_toggle_ms;
    wire [31:0] alert_emergency_toggle_ms;
    reg         alert_active;
    reg [1:0]   alert_active_level;
    reg [1:0]   alert_led;

    reg [7:0]   status_reg;
    reg [31:0]  perf_cnt_lo;
    reg [31:0]  perf_cnt_hi;
    reg [31:0]  perf_lut_wait;
    reg [31:0]  perf_feat_wait;
    reg [31:0]  perf_write_wait;
    reg [31:0]  perf_valid_voxel;
    reg [31:0]  perf_lut_reads;
    reg [31:0]  perf_feat_reads;
    reg [31:0]  perf_writes;
    reg [31:0]  perf_pipe_stall;

    reg [31:0]  read_value;

    bev_reg_ctrl dut (
        .ra_awaddr        (ra_awaddr),
        .ra_awdata        (ra_awdata),
        .ra_awvalid       (ra_awvalid),
        .ra_awready       (ra_awready),
        .ra_araddr        (ra_araddr),
        .ra_arvalid       (ra_arvalid),
        .ra_arready       (ra_arready),
        .ra_rdata         (ra_rdata),
        .ra_rvalid        (ra_rvalid),
        .ra_rready        (ra_rready),
        .comp_start       (comp_start),
        .comp_mode        (comp_mode),
        .frame_shift_en   (frame_shift_en),
        .comp_done        (comp_done),
        .lut_base_addr    (lut_base_addr),
        .lut_size         (lut_size),
        .feat2d_base_addr (feat2d_base_addr),
        .feat3d_wr_addr   (feat3d_wr_addr),
        .feat3d_wr_size   (feat3d_wr_size),
        .bev_channels     (bev_channels),
        .bev_x            (bev_x),
        .bev_y            (bev_y),
        .bev_z            (bev_z),
        .img_w            (img_w),
        .img_h            (img_h),
        .cameras          (cameras),
        .alert_level      (alert_level),
        .alert_trigger    (alert_trigger),
        .alert_clear      (alert_clear),
        .alert_duration_ms (alert_duration_ms),
        .alert_danger_toggle_ms (alert_danger_toggle_ms),
        .alert_emergency_toggle_ms (alert_emergency_toggle_ms),
        .alert_active     (alert_active),
        .alert_active_level (alert_active_level),
        .alert_led        (alert_led),
        .status_reg       (status_reg),
        .perf_cnt_lo      (perf_cnt_lo),
        .perf_cnt_hi      (perf_cnt_hi),
        .perf_lut_wait    (perf_lut_wait),
        .perf_feat_wait   (perf_feat_wait),
        .perf_write_wait  (perf_write_wait),
        .perf_valid_voxel (perf_valid_voxel),
        .perf_lut_reads   (perf_lut_reads),
        .perf_feat_reads  (perf_feat_reads),
        .perf_writes      (perf_writes),
        .perf_pipe_stall  (perf_pipe_stall),
        .clk              (clk),
        .rst_n            (rst_n)
    );

    always #5 clk = ~clk;

    task fail;
        input [8*100-1:0] message;
        begin
            $display("TEST_FAIL: %0s", message);
            $finish;
        end
    endtask

    task write_reg;
        input [15:0] address;
        input [31:0] value;
        begin
            @(negedge clk);
            ra_awaddr  = address;
            ra_awdata  = value;
            ra_awvalid = 1'b1;
            @(negedge clk);
            ra_awvalid = 1'b0;
            #1;
        end
    endtask

    task read_reg;
        input [15:0] address;
        output [31:0] value;
        begin
            @(negedge clk);
            ra_araddr  = address;
            ra_arvalid = 1'b1;
            @(posedge clk);
            #1;
            if (ra_rvalid !== 1'b1)
                fail("register read must assert rvalid");
            value = ra_rdata;
            @(negedge clk);
            ra_arvalid = 1'b0;
        end
    endtask

    initial begin
        clk                = 1'b0;
        rst_n              = 1'b0;
        ra_awaddr          = 16'd0;
        ra_awdata          = 32'd0;
        ra_awvalid         = 1'b0;
        ra_araddr          = 16'd0;
        ra_arvalid         = 1'b0;
        ra_rready          = 1'b1;
        comp_done          = 1'b0;
        alert_active       = 1'b0;
        alert_active_level = 2'b00;
        alert_led          = 2'b00;
        status_reg         = 8'h5A;
        perf_cnt_lo        = 32'h01234567;
        perf_cnt_hi        = 32'h89ABCDEF;
        perf_lut_wait       = 32'd11;
        perf_feat_wait      = 32'd22;
        perf_write_wait     = 32'd33;
        perf_valid_voxel    = 32'd44;
        perf_lut_reads      = 32'd55;
        perf_feat_reads     = 32'd66;
        perf_writes         = 32'd77;
        perf_pipe_stall     = 32'd88;

        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);
        #1;

        if (alert_duration_ms !== 32'd3000)
            fail("default alert duration must be 3000 ms");
        if (alert_danger_toggle_ms !== 32'd500)
            fail("default danger toggle must be 500 ms");
        if (alert_emergency_toggle_ms !== 32'd125)
            fail("default emergency toggle must be 125 ms");

        read_reg(16'h0C, read_value);
        if (read_value !== 32'd3000)
            fail("duration register readback failed");
        read_reg(16'h0D, read_value);
        if (read_value !== 32'd0)
            fail("reserved register must read zero");
        read_reg(16'h0E, read_value);
        if (read_value !== 32'd500)
            fail("danger period register readback failed");
        read_reg(16'h0F, read_value);
        if (read_value !== 32'd125)
            fail("emergency period register readback failed");
        read_reg(16'h25, read_value);
        if (read_value !== 32'h414C0001)
            fail("ALERT_CAPS readback failed");
        read_reg(16'h30, read_value);
        if (read_value !== 32'hBE080003)
            fail("BEV_VERSION readback failed");
        read_reg(16'h26, read_value);
        if (read_value !== 32'd11) fail("PERF_LUT_WAIT readback failed");
        read_reg(16'h27, read_value);
        if (read_value !== 32'd22) fail("PERF_FEAT_WAIT readback failed");
        read_reg(16'h28, read_value);
        if (read_value !== 32'd33) fail("PERF_WRITE_WAIT readback failed");
        read_reg(16'h29, read_value);
        if (read_value !== 32'd44) fail("PERF_VALID_VOXEL readback failed");
        read_reg(16'h2A, read_value);
        if (read_value !== 32'd55) fail("PERF_LUT_READS readback failed");
        read_reg(16'h2B, read_value);
        if (read_value !== 32'd66) fail("PERF_FEAT_READS readback failed");
        read_reg(16'h2C, read_value);
        if (read_value !== 32'd77) fail("PERF_WRITES readback failed");
        read_reg(16'h2D, read_value);
        if (read_value !== 32'd88) fail("PERF_PIPE_STALL readback failed");
        read_reg(16'h2E, read_value);
        if (read_value !== 32'h50320001) fail("PART2_CAPS readback failed");

        write_reg(16'h0C, 32'd4321);
        write_reg(16'h0E, 32'd321);
        write_reg(16'h0F, 32'd87);
        if (alert_duration_ms !== 32'd4321 ||
            alert_danger_toggle_ms !== 32'd321 ||
            alert_emergency_toggle_ms !== 32'd87)
            fail("alert configuration writes failed");

        // trigger and level are emitted after one command write.
        write_reg(16'h0B, 32'h00000102);
        if (alert_level !== 2'b10 || alert_trigger !== 1'b1 || alert_clear !== 1'b0)
            fail("danger trigger command failed");
        @(posedge clk);
        #1;
        if (alert_trigger !== 1'b0)
            fail("trigger must be a one-cycle pulse");

        // Status bit layout: LED=01 at [9:8], level=10 at [2:1], active=1.
        alert_active       = 1'b1;
        alert_active_level = 2'b10;
        alert_led          = 2'b01;
        read_reg(16'h24, read_value);
        if (read_value !== 32'h00000105)
            fail("ALERT_STATUS bit layout failed");

        write_reg(16'h0B, 32'h00000200);
        if (alert_trigger !== 1'b0 || alert_clear !== 1'b1)
            fail("clear command failed");
        @(posedge clk);
        #1;
        if (alert_clear !== 1'b0)
            fail("clear must be a one-cycle pulse");

        $display("TEST_PASS: tb_bev_alert_regs");
        $finish;
    end

endmodule
