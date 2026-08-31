//==============================================================================
// bev_accel_top.v
//------------------------------------------------------------------------------
// Top wrapper for the FastBEV Part2 INT8 delivery package.
//
// This package implements LUT mode only:
//   CTRL_START = 0x03 -> start=1, mode=01.
// SA mode is intentionally not implemented in this delivery because the target
// data path directly creates the 4x repeated Decoder INT8 input.
//==============================================================================
`timescale 1ns/1ps

module bev_accel_top(
    input        [31:0]   itf_ra_awaddr,
    input        [31:0]   itf_ra_awdata,
    input                 itf_ra_awvalid,
    output                itf_ra_awready,

    input        [31:0]   itf_ra_araddr,
    input                 itf_ra_arvalid,
    output                itf_ra_arready,

    output       [31:0]   itf_ra_rdata,
    output                itf_ra_rvalid,
    input                 itf_ra_rready,

    output       [31:0]   itf_awaddr,
    output       [511:0]  itf_awdata,
    output                itf_awvalid,
    input                 itf_awready,

    output       [31:0]   itf_araddr,
    output                itf_arvalid,
    input                 itf_arready,

    input        [511:0]  itf_rdata,
    input                 itf_rvalid,
    output                itf_rready,

    output reg            reset_reg,
    output       [1:0]    alert_led,

    input                 clk,
    input                 ra_clk,
    input                 rst_n,
    input                 ra_rst_n
);

    localparam RESET_ADDR = 16'h77;

    (* MAX_FANOUT = 256 *) reg rst_n_1;
    (* MAX_FANOUT = 256 *) reg ra_rst_n_1;

    wire comp_start, comp_done, comp_done_cross, comp_start_cross;
    wire [1:0] comp_mode;
    wire frame_shift_en;
    reg [1:0] comp_mode_sync;

    wire [31:0] lut_base_addr, lut_size, feat2d_base_addr, feat3d_wr_addr, feat3d_wr_size;
    wire [7:0]  bev_channels, bev_x, bev_y, bev_z;
    wire [11:0] img_w, img_h;
    wire [7:0]  cameras;

    wire [1:0]  alert_level;
    wire        alert_trigger, alert_clear;
    wire [31:0] alert_duration_ms;
    wire [31:0] alert_danger_toggle_ms;
    wire [31:0] alert_emergency_toggle_ms;
    wire        alert_active;
    wire [1:0]  alert_active_level;

    wire lut_start, lut_done;
    assign lut_start = comp_start_cross && (comp_mode_sync == 2'b01);
    assign comp_done = lut_done;

    wire [31:0]  lut_rd_addr;
    wire         lut_rd_req, lut_rd_grant;
    wire [511:0] lut_rd_data;
    wire         lut_rd_data_valid, lut_rd_data_ready;
    wire [31:0]  lut_wr_addr;
    wire [511:0] lut_wr_data;
    wire         lut_wr_req, lut_wr_grant;

    wire [31:0] perf_cnt_lo, perf_cnt_hi;
    wire [31:0] perf_lut_wait, perf_feat_wait, perf_write_wait;
    wire [31:0] perf_valid_voxel, perf_lut_reads, perf_feat_reads;
    wire [31:0] perf_writes, perf_pipe_stall;
    reg [63:0] perf_counter;
    reg perf_en;
    wire [7:0] status_reg = {7'd0, lut_done};

    assign perf_cnt_lo = perf_counter[31:0];
    assign perf_cnt_hi = perf_counter[63:32];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            comp_mode_sync <= 2'b00;
        else
            comp_mode_sync <= comp_mode;
    end

    always @(posedge clk or negedge rst_n_1) begin
        if (!rst_n_1)
            perf_en <= 1'b0;
        else if (comp_start_cross)
            perf_en <= 1'b1;
        else if (comp_done)
            perf_en <= 1'b0;
    end

    always @(posedge clk or negedge rst_n_1) begin
        if (!rst_n_1)
            perf_counter <= 64'd0;
        else if (comp_start_cross)
            perf_counter <= 64'd0;
        else if (perf_en)
            perf_counter <= perf_counter + 64'd1;
    end

    bev_reg_ctrl U_bev_reg_ctrl (
        .ra_awaddr        (itf_ra_awaddr[17:2]),
        .ra_awdata        (itf_ra_awdata),
        .ra_awvalid       (itf_ra_awvalid),
        .ra_awready       (itf_ra_awready),
        .ra_araddr        (itf_ra_araddr[17:2]),
        .ra_arvalid       (itf_ra_arvalid),
        .ra_arready       (itf_ra_arready),
        .ra_rdata         (itf_ra_rdata),
        .ra_rvalid        (itf_ra_rvalid),
        .ra_rready        (itf_ra_rready),
        .comp_start       (comp_start),
        .comp_mode        (comp_mode),
        .frame_shift_en   (frame_shift_en),
        .comp_done        (comp_done_cross),
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
        .clk              (ra_clk),
        .rst_n            (ra_rst_n_1)
    );

    alert_led_ctrl #(
        .CLK_FREQ_HZ (100_000_000)
    ) U_alert_led_ctrl (
        .clk                 (ra_clk),
        .rst_n               (ra_rst_n_1),
        .level               (alert_level),
        .trigger             (alert_trigger),
        .clear               (alert_clear),
        .duration_ms         (alert_duration_ms),
        .danger_toggle_ms    (alert_danger_toggle_ms),
        .emergency_toggle_ms (alert_emergency_toggle_ms),
        .active              (alert_active),
        .active_level        (alert_active_level),
        .led                 (alert_led)
    );

    lut_engine #(
        // The fixed EDIF DDR bridge has not yet been proven on hardware with
        // more than one outstanding request. Keep the production-safe depth.
        .FEAT_MAX_OUTSTANDING (1)
    ) U_lut_engine (
        .engine_start     (lut_start),
        .engine_done      (lut_done),
        .lut_base_addr    (lut_base_addr),
        .lut_size         (lut_size),
        .feat2d_base_addr (feat2d_base_addr),
        .feat3d_wr_addr   (feat3d_wr_addr),
        .bev_x            (bev_x),
        .bev_y            (bev_y),
        .bev_z            (bev_z),
        .img_w            (img_w),
        .img_h            (img_h),
        .rd_addr          (lut_rd_addr),
        .rd_req           (lut_rd_req),
        .rd_grant         (lut_rd_grant),
        .rd_data          (lut_rd_data),
        .rd_data_valid    (lut_rd_data_valid),
        .rd_data_ready    (lut_rd_data_ready),
        .wr_addr          (lut_wr_addr),
        .wr_data          (lut_wr_data),
        .wr_req           (lut_wr_req),
        .wr_grant         (lut_wr_grant),
        .perf_lut_wait    (perf_lut_wait),
        .perf_feat_wait   (perf_feat_wait),
        .perf_write_wait  (perf_write_wait),
        .perf_valid_voxel (perf_valid_voxel),
        .perf_lut_reads   (perf_lut_reads),
        .perf_feat_reads  (perf_feat_reads),
        .perf_writes      (perf_writes),
        .perf_pipe_stall  (perf_pipe_stall),
        .clk              (clk),
        .rst_n            (rst_n_1)
    );

    dma_arbiter U_dma_arbiter (
        .active_engine      (comp_mode_sync),
        .lut_rd_addr        (lut_rd_addr),
        .lut_rd_req         (lut_rd_req),
        .lut_rd_grant       (lut_rd_grant),
        .lut_rd_data        (lut_rd_data),
        .lut_rd_data_valid  (lut_rd_data_valid),
        .lut_rd_data_ready  (lut_rd_data_ready),
        .lut_wr_addr        (lut_wr_addr),
        .lut_wr_data        (lut_wr_data),
        .lut_wr_req         (lut_wr_req),
        .lut_wr_grant       (lut_wr_grant),
        .sa_rd_addr         (32'd0),
        .sa_rd_req          (1'b0),
        .sa_rd_grant        (),
        .sa_rd_data         (),
        .sa_rd_data_valid   (),
        .sa_rd_data_ready   (1'b0),
        .sa_wr_addr         (32'd0),
        .sa_wr_data         (512'd0),
        .sa_wr_req          (1'b0),
        .sa_wr_grant        (),
        .ddr_araddr         (itf_araddr),
        .ddr_arvalid        (itf_arvalid),
        .ddr_arready        (itf_arready),
        .ddr_rdata          (itf_rdata),
        .ddr_rvalid         (itf_rvalid),
        .ddr_rready         (itf_rready),
        .ddr_awaddr         (itf_awaddr),
        .ddr_awdata         (itf_awdata),
        .ddr_awvalid        (itf_awvalid),
        .ddr_awready        (itf_awready),
        .clk                (clk),
        .rst_n              (rst_n_1)
    );

    pulse_cross U_done_cross (
        .a2(comp_done_cross), .clk2(ra_clk), .rst2(~ra_rst_n_1),
        .rdy1(), .a1(comp_done), .clk1(clk), .rst1(~rst_n_1)
    );

    pulse_cross U_start_cross (
        .a2(comp_start_cross), .clk2(clk), .rst2(~rst_n_1),
        .rdy1(), .a1(comp_start), .clk1(ra_clk), .rst1(~ra_rst_n_1)
    );

    always @(posedge clk or negedge ra_rst_n) begin
        if (!ra_rst_n)
            reset_reg <= 1'b0;
        else if (itf_ra_awvalid && itf_ra_awready && itf_ra_awaddr[17:2] == RESET_ADDR)
            reset_reg <= itf_ra_awdata[0];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rst_n_1 <= 1'b0;
        else        rst_n_1 <= ~reset_reg;
    end

    always @(posedge ra_clk or negedge ra_rst_n) begin
        if (!ra_rst_n) ra_rst_n_1 <= 1'b0;
        else           ra_rst_n_1 <= ~reset_reg;
    end

endmodule
