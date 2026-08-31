//==============================================================================
// bev_reg_ctrl.v
//------------------------------------------------------------------------------
// Register controller for the INT8 Part2 delivery.
//
// Word address map, CPU byte address = 0x400C_0000 + word_addr * 4:
//   0x00 CTRL_START       [0]start [2:1]mode(01=LUT)
//   0x01 LUT_BASE_ADDR
//   0x02 LUT_SIZE         total voxels, default 160000
//   0x03 FEAT2D_BASE_ADDR FP32 NHWC Part1 output
//   0x04 FEAT3D_WR_ADDR   final INT8 Decoder input buffer
//   0x05 FEAT3D_WR_SIZE   default 40960000 bytes
//   0x09 BEV_PARAMS       {bev_z, bev_y, bev_x, channels} = {4,200,200,64}
//   0x0A IMG_PARAMS       {cameras, img_h, img_w} = {6,64,176}
//   0x20 COMP_DONE        sticky done, cleared by next start
//   0x21 STATUS
//   0x22 PERF_CNT_LO
//   0x23 PERF_CNT_HI
//   0x30 VERSION
//   0x77 RESET
//==============================================================================
`timescale 1ns/1ps

module bev_reg_ctrl(
    input        [15:0]  ra_awaddr,
    input        [31:0]  ra_awdata,
    input                ra_awvalid,
    output               ra_awready,

    input        [15:0]  ra_araddr,
    input                ra_arvalid,
    output               ra_arready,

    output reg   [31:0]  ra_rdata,
    output reg           ra_rvalid,
    input                ra_rready,

    output reg           comp_start,
    output reg   [1:0]   comp_mode,
    output reg           frame_shift_en,
    input                comp_done,

    output reg   [31:0]  lut_base_addr,
    output reg   [31:0]  lut_size,
    output reg   [31:0]  feat2d_base_addr,
    output reg   [31:0]  feat3d_wr_addr,
    output reg   [31:0]  feat3d_wr_size,

    output reg   [7:0]   bev_channels,
    output reg   [7:0]   bev_x,
    output reg   [7:0]   bev_y,
    output reg   [7:0]   bev_z,
    output reg   [11:0]  img_w,
    output reg   [11:0]  img_h,
    output reg   [7:0]   cameras,

    input        [7:0]   status_reg,
    input        [31:0]  perf_cnt_lo,
    input        [31:0]  perf_cnt_hi,

    input                clk,
    input                rst_n
);

    localparam CTRL_START_ADDR     = 16'h00;
    localparam LUT_BASE_ADDR_A     = 16'h01;
    localparam LUT_SIZE_ADDR       = 16'h02;
    localparam FEAT2D_BASE_ADDR_A  = 16'h03;
    localparam FEAT3D_WR_ADDR_A    = 16'h04;
    localparam FEAT3D_WR_SIZE_A    = 16'h05;
    localparam BEV_PARAMS_ADDR     = 16'h09;
    localparam IMG_PARAMS_ADDR     = 16'h0A;
    localparam COMP_DONE_ADDR      = 16'h20;
    localparam STATUS_ADDR         = 16'h21;
    localparam PERF_CNT_LO_ADDR    = 16'h22;
    localparam PERF_CNT_HI_ADDR    = 16'h23;
    localparam VERSION_ADDR        = 16'h30;
    localparam DEBUG_REG_ADDR      = 16'hFF;

    localparam BEV_VERSION         = 32'hBE080001;
    localparam DECODER_INT8_BYTES  = 32'd40960000;

    reg [31:0] debug_reg;
    reg        comp_done_reg;

    assign ra_awready = 1'b1;
    assign ra_arready = !ra_rvalid || ra_rready;
    wire ra_awevent = ra_awvalid && ra_awready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ra_rvalid <= 1'b0;
        else if (ra_arvalid && ra_arready)
            ra_rvalid <= 1'b1;
        else if (ra_rready)
            ra_rvalid <= 1'b0;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            comp_start <= 1'b0;
        else
            comp_start <= ra_awevent && (ra_awaddr == CTRL_START_ADDR) && ra_awdata[0];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            comp_mode <= 2'b00;
            frame_shift_en <= 1'b0;
        end else if (ra_awevent && ra_awaddr == CTRL_START_ADDR) begin
            comp_mode <= ra_awdata[2:1];
            frame_shift_en <= ra_awdata[3];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) lut_base_addr <= 32'd0;
        else if (ra_awevent && ra_awaddr == LUT_BASE_ADDR_A) lut_base_addr <= ra_awdata;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) lut_size <= 32'd160000;
        else if (ra_awevent && ra_awaddr == LUT_SIZE_ADDR) lut_size <= ra_awdata;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) feat2d_base_addr <= 32'd0;
        else if (ra_awevent && ra_awaddr == FEAT2D_BASE_ADDR_A) feat2d_base_addr <= ra_awdata;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) feat3d_wr_addr <= 32'd0;
        else if (ra_awevent && ra_awaddr == FEAT3D_WR_ADDR_A) feat3d_wr_addr <= ra_awdata;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) feat3d_wr_size <= DECODER_INT8_BYTES;
        else if (ra_awevent && ra_awaddr == FEAT3D_WR_SIZE_A) feat3d_wr_size <= ra_awdata;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bev_channels <= 8'd64;
            bev_x <= 8'd200;
            bev_y <= 8'd200;
            bev_z <= 8'd4;
        end else if (ra_awevent && ra_awaddr == BEV_PARAMS_ADDR) begin
            bev_z <= ra_awdata[31:24];
            bev_y <= ra_awdata[23:16];
            bev_x <= ra_awdata[15:8];
            bev_channels <= ra_awdata[7:0];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cameras <= 8'd6;
            img_h <= 12'd64;
            img_w <= 12'd176;
        end else if (ra_awevent && ra_awaddr == IMG_PARAMS_ADDR) begin
            cameras <= ra_awdata[31:24];
            img_h <= ra_awdata[23:12];
            img_w <= ra_awdata[11:0];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) debug_reg <= 32'd0;
        else if (ra_awevent && ra_awaddr == DEBUG_REG_ADDR) debug_reg <= ra_awdata;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) comp_done_reg <= 1'b0;
        else if (comp_start) comp_done_reg <= 1'b0;
        else if (comp_done) comp_done_reg <= 1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ra_rdata <= 32'd0;
        else if (ra_arvalid && ra_arready) begin
            case (ra_araddr)
                CTRL_START_ADDR:    ra_rdata <= {28'd0, frame_shift_en, comp_mode, 1'b0};
                LUT_BASE_ADDR_A:    ra_rdata <= lut_base_addr;
                LUT_SIZE_ADDR:      ra_rdata <= lut_size;
                FEAT2D_BASE_ADDR_A: ra_rdata <= feat2d_base_addr;
                FEAT3D_WR_ADDR_A:   ra_rdata <= feat3d_wr_addr;
                FEAT3D_WR_SIZE_A:   ra_rdata <= feat3d_wr_size;
                BEV_PARAMS_ADDR:    ra_rdata <= {bev_z, bev_y, bev_x, bev_channels};
                IMG_PARAMS_ADDR:    ra_rdata <= {cameras, img_h, img_w};
                COMP_DONE_ADDR:     ra_rdata <= {31'd0, comp_done_reg};
                STATUS_ADDR:        ra_rdata <= {24'd0, status_reg};
                PERF_CNT_LO_ADDR:   ra_rdata <= perf_cnt_lo;
                PERF_CNT_HI_ADDR:   ra_rdata <= perf_cnt_hi;
                VERSION_ADDR:       ra_rdata <= BEV_VERSION;
                DEBUG_REG_ADDR:     ra_rdata <= debug_reg;
                default:            ra_rdata <= 32'hFFFF_FFFF;
            endcase
        end
    end

endmodule

