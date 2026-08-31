`timescale 1ns/1ps
//==============================================================================
// File Name     : bev_reg_ctrl.v
// Project       : Fast-BEV Part2 FPGA Accelerator
// Description   : Register Controller — PS writes config, reads status.
//   Strictly follows FPAI demo reg_ctrl.v pattern:
//     - ra_awready always high (no backpressure)
//     - ra_arready gated by ra_rvalid
//     - Single-cycle comp_start pulse
//     - Ready/Valid handshake protocol
//
//   Register Map (word-addressed via itf_ra_awaddr[17:2]):
//     0x00  CTRL_START       [0]start [2:1]mode(01=LUT,10=SA) [3]frame_shift
//     0x01  LUT_BASE_ADDR    [31:0]
//     0x02  LUT_SIZE          [31:0] total voxels
//     0x03  FEAT2D_BASE_ADDR [31:0]
//     0x04  FEAT3D_WR_ADDR   [31:0]
//     0x05  FEAT3D_WR_SIZE   [31:0] LUT output bytes (FP16: 20,480,000)
//     0x06  SA_SRC_ADDR      [31:0]
//     0x07  SA_DST_ADDR      [31:0]
//     0x08  SA_SIZE           [31:0] bev_x * bev_y
//     0x09  BEV_PARAMS        {bev_z[8], bev_y[8], bev_x[8], channels[8]}
//     0x0A  IMG_PARAMS        {cameras[8], img_h[12], img_w[12]}
//     0x0B~0x10  XFORM matrix (Q16.16)
//     0x11~0x14  FRAME0~3 addresses
//     0x15  FRAME_SIZE
//     0x20  COMP_DONE (R)
//     0x21  STATUS (R)
//     0x22~0x23  PERF_CNT (R)
//     0x30  VERSION (R)
//     0x77  RESET (W)
//     0xFF  DEBUG (R/W)
//==============================================================================

module bev_reg_ctrl(    

    input         [   15 : 0]   ra_awaddr        ,  
    input         [   31 : 0]   ra_awdata        ,
    input                       ra_awvalid       ,
    output                      ra_awready       ,

    input         [   15 : 0]   ra_araddr        ,  
    input                       ra_arvalid       ,
    output                      ra_arready       ,

    output  reg   [   31 : 0]   ra_rdata         ,
    output  reg                 ra_rvalid        ,
    input                       ra_rready        ,

    output  reg                 comp_start       ,
    output  reg   [    1 : 0]   comp_mode        ,
    output  reg                 frame_shift_en   ,
    input                       comp_done        ,

    output reg    [   31 : 0]   lut_base_addr    ,  //register的端口中，output意味着是cpu写、供fpga读的；input意味着是fpga写、供cpu读的
    output reg    [   31 : 0]   lut_size         ,
    output reg    [   31 : 0]   feat2d_base_addr ,
    output reg    [   31 : 0]   feat3d_wr_addr   ,
    output reg    [   31 : 0]   feat3d_wr_size   ,

    output reg    [    7 : 0]   bev_channels     ,
    output reg    [    7 : 0]   bev_x            ,
    output reg    [    7 : 0]   bev_y            ,
    output reg    [    7 : 0]   bev_z            ,
    output reg    [   11 : 0]   img_w            ,
    output reg    [   11 : 0]   img_h            ,
    output reg    [    7 : 0]   cameras          ,

    output reg    [   31 : 0]   sa_src_addr      ,
    output reg    [   31 : 0]   sa_dst_addr      ,
    output reg    [   31 : 0]   sa_size          ,
    output reg    [   31 : 0]   xform_a00        ,
    output reg    [   31 : 0]   xform_a01        ,
    output reg    [   31 : 0]   xform_a02        ,
    output reg    [   31 : 0]   xform_a10        ,
    output reg    [   31 : 0]   xform_a11        ,
    output reg    [   31 : 0]   xform_a12        ,

    output reg    [   31 : 0]   frame0_addr      ,
    output reg    [   31 : 0]   frame1_addr      ,
    output reg    [   31 : 0]   frame2_addr      ,
    output reg    [   31 : 0]   frame3_addr      ,
    output reg    [   31 : 0]   frame_size       ,

    input         [    7 : 0]   status_reg       ,
    input         [   31 : 0]   perf_cnt_lo      ,
    input         [   31 : 0]   perf_cnt_hi      ,

    input                              clk       ,
    input                            rst_n 
);

    // ===================== Address Map (identical to previous version) =====================
    localparam CTRL_START_ADDR    = 16'h00;
    localparam LUT_BASE_ADDR_A   = 16'h01;
    localparam LUT_SIZE_ADDR     = 16'h02;
    localparam FEAT2D_BASE_ADDR_A= 16'h03;
    localparam FEAT3D_WR_ADDR_A  = 16'h04;
    localparam FEAT3D_WR_SIZE_A  = 16'h05;
    localparam SA_SRC_ADDR_A     = 16'h06;
    localparam SA_DST_ADDR_A     = 16'h07;
    localparam SA_SIZE_ADDR      = 16'h08;
    localparam BEV_PARAMS_ADDR   = 16'h09;
    localparam IMG_PARAMS_ADDR   = 16'h0A;
    localparam XFORM_A00_ADDR    = 16'h0B;
    localparam XFORM_A01_ADDR    = 16'h0C;
    localparam XFORM_A02_ADDR    = 16'h0D;
    localparam XFORM_A10_ADDR    = 16'h0E;
    localparam XFORM_A11_ADDR    = 16'h0F;
    localparam XFORM_A12_ADDR    = 16'h10;
    localparam FRAME0_ADDR_A     = 16'h11;
    localparam FRAME1_ADDR_A     = 16'h12;
    localparam FRAME2_ADDR_A     = 16'h13;
    localparam FRAME3_ADDR_A     = 16'h14;
    localparam FRAME_SIZE_ADDR   = 16'h15;
    localparam COMP_DONE_ADDR    = 16'h20;
    localparam STATUS_ADDR       = 16'h21;
    localparam PERF_CNT_LO_ADDR  = 16'h22;
    localparam PERF_CNT_HI_ADDR  = 16'h23;
    localparam VERSION_ADDR      = 16'h30;
    localparam RESET_ADDR        = 16'h77;
    localparam DEBUG_REG_ADDR    = 16'hFF;

    localparam BEV_VERSION       = 32'h2026_0818;

    wire              ra_awevent      ;
    reg   [31 : 0]    debug_reg       ;
    reg               comp_done_reg   ;

    // ===================== Handshake (same as demo reg_ctrl) =====================
    assign ra_awready = 1'b1;
    assign ra_arready = (ra_rvalid == 1'b0);
    assign ra_awevent = ra_awvalid && ra_awready;

    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0)
            ra_rvalid <= #0.1 1'b0;
        else if(ra_arvalid && ra_arready)
            ra_rvalid <= #0.1 1'b1;
        else if(ra_rready)
            ra_rvalid <= #0.1 1'b0;
    end

    // ===================== CTRL_START (single-cycle pulse) =====================
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) begin
            comp_start     <= #0.1 1'b0;
            comp_mode      <= #0.1 2'b00;
            frame_shift_en <= #0.1 1'b0;
        end
        else if(comp_start==1'b1)
            comp_start     <= #0.1 1'b0;
        else if(ra_awevent && ra_awaddr==CTRL_START_ADDR) begin
            comp_start     <= #0.1 ra_awdata[0];
            comp_mode      <= #0.1 ra_awdata[2:1];
            frame_shift_en <= #0.1 ra_awdata[3];
        end
    end

    // ===================== LUT Registers =====================
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) lut_base_addr <= #0.1 32'd0;
        else if(ra_awevent && ra_awaddr==LUT_BASE_ADDR_A) lut_base_addr <= #0.1 ra_awdata;
    end
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) lut_size <= #0.1 32'd160000; // default: 200×200×4
        else if(ra_awevent && ra_awaddr==LUT_SIZE_ADDR) lut_size <= #0.1 ra_awdata;
    end
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) feat2d_base_addr <= #0.1 32'd0;
        else if(ra_awevent && ra_awaddr==FEAT2D_BASE_ADDR_A) feat2d_base_addr <= #0.1 ra_awdata;
    end
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) feat3d_wr_addr <= #0.1 32'd0;
        else if(ra_awevent && ra_awaddr==FEAT3D_WR_ADDR_A) feat3d_wr_addr <= #0.1 ra_awdata;
    end
    always@(posedge clk or negedge rst_n) begin
        // [1,16,200,200,16] FP16 output buffer.
        if(rst_n==1'b0) feat3d_wr_size <= #0.1 32'd20480000;
        else if(ra_awevent && ra_awaddr==FEAT3D_WR_SIZE_A) feat3d_wr_size <= #0.1 ra_awdata;
    end

    // ===================== BEV/Image Parameters (defaults for vehicle FP16) =====================
    // Physical Part1 input buffer: [6,120,160,64] NHWC FP32.
    // Output: [1,16,200,200,16] FP16, with Z folded into cblk16.
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) begin
            bev_channels <= #0.1 8'd64;   // 64 channels per voxel
            bev_x        <= #0.1 8'd200;  // BEV X dimension
            bev_y        <= #0.1 8'd200;  // BEV Y dimension
            bev_z        <= #0.1 8'd4;    // BEV Z (height) dimension
        end
        else if(ra_awevent && ra_awaddr==BEV_PARAMS_ADDR) begin
            bev_channels <= #0.1 ra_awdata[7:0];
            bev_x        <= #0.1 ra_awdata[15:8];
            bev_y        <= #0.1 ra_awdata[23:16];
            bev_z        <= #0.1 ra_awdata[31:24];
        end
    end
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) begin
            img_w    <= #0.1 12'd160;  // 2D feature map width
            img_h    <= #0.1 12'd120;  // 2D feature map height
            cameras  <= #0.1 8'd6;     // number of cameras
        end
        else if(ra_awevent && ra_awaddr==IMG_PARAMS_ADDR) begin
            img_w    <= #0.1 ra_awdata[11:0];
            img_h    <= #0.1 ra_awdata[23:12];
            cameras  <= #0.1 ra_awdata[31:24];
        end
    end

    // ===================== SA Registers =====================
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) sa_src_addr <= #0.1 32'd0;
        else if(ra_awevent && ra_awaddr==SA_SRC_ADDR_A) sa_src_addr <= #0.1 ra_awdata;
    end
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) sa_dst_addr <= #0.1 32'd0;
        else if(ra_awevent && ra_awaddr==SA_DST_ADDR_A) sa_dst_addr <= #0.1 ra_awdata;
    end
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) sa_size <= #0.1 32'd40000; // default: 200×200
        else if(ra_awevent && ra_awaddr==SA_SIZE_ADDR) sa_size <= #0.1 ra_awdata;
    end

    // ===================== Transform Matrix (Q16.16 fixed-point) =====================
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) xform_a00 <= #0.1 32'h0001_0000; // 1.0
        else if(ra_awevent && ra_awaddr==XFORM_A00_ADDR) xform_a00 <= #0.1 ra_awdata;
    end
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) xform_a01 <= #0.1 32'd0;
        else if(ra_awevent && ra_awaddr==XFORM_A01_ADDR) xform_a01 <= #0.1 ra_awdata;
    end
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) xform_a02 <= #0.1 32'd0;
        else if(ra_awevent && ra_awaddr==XFORM_A02_ADDR) xform_a02 <= #0.1 ra_awdata;
    end
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) xform_a10 <= #0.1 32'd0;
        else if(ra_awevent && ra_awaddr==XFORM_A10_ADDR) xform_a10 <= #0.1 ra_awdata;
    end
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) xform_a11 <= #0.1 32'h0001_0000; // 1.0
        else if(ra_awevent && ra_awaddr==XFORM_A11_ADDR) xform_a11 <= #0.1 ra_awdata;
    end
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) xform_a12 <= #0.1 32'd0;
        else if(ra_awevent && ra_awaddr==XFORM_A12_ADDR) xform_a12 <= #0.1 ra_awdata;
    end

    // ===================== Frame Slot Addresses =====================
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) frame0_addr <= #0.1 32'd0;
        else if(ra_awevent && ra_awaddr==FRAME0_ADDR_A) frame0_addr <= #0.1 ra_awdata;
    end
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) frame1_addr <= #0.1 32'd0;
        else if(ra_awevent && ra_awaddr==FRAME1_ADDR_A) frame1_addr <= #0.1 ra_awdata;
    end
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) frame2_addr <= #0.1 32'd0;
        else if(ra_awevent && ra_awaddr==FRAME2_ADDR_A) frame2_addr <= #0.1 ra_awdata;
    end
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) frame3_addr <= #0.1 32'd0;
        else if(ra_awevent && ra_awaddr==FRAME3_ADDR_A) frame3_addr <= #0.1 ra_awdata;
    end
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) frame_size <= #0.1 32'd40960000; // 160000 voxels × 64 ×4B
        else if(ra_awevent && ra_awaddr==FRAME_SIZE_ADDR) frame_size <= #0.1 ra_awdata;
    end

    // ===================== Debug & Status =====================
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0) debug_reg <= #0.1 32'd0;
        else if(ra_awevent && ra_awaddr==DEBUG_REG_ADDR) debug_reg <= #0.1 ra_awdata;
    end
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0)      comp_done_reg <= #0.1 1'b0;
        else if(comp_start)  comp_done_reg <= #0.1 1'b0;
        else if(comp_done)   comp_done_reg <= #0.1 1'b1;
    end

    // ===================== Read Mux =====================
    always@(posedge clk or negedge rst_n) begin
        if(rst_n==1'b0)
            ra_rdata <= #0.1 32'd0;
        else if(ra_arvalid && ra_arready) begin
            case(ra_araddr)
                CTRL_START_ADDR   : ra_rdata <= #0.1 {28'd0, frame_shift_en, comp_mode, 1'b0};
                LUT_BASE_ADDR_A   : ra_rdata <= #0.1 lut_base_addr;
                LUT_SIZE_ADDR     : ra_rdata <= #0.1 lut_size;
                FEAT2D_BASE_ADDR_A: ra_rdata <= #0.1 feat2d_base_addr;
                FEAT3D_WR_ADDR_A  : ra_rdata <= #0.1 feat3d_wr_addr;
                FEAT3D_WR_SIZE_A  : ra_rdata <= #0.1 feat3d_wr_size;
                SA_SRC_ADDR_A     : ra_rdata <= #0.1 sa_src_addr;
                SA_DST_ADDR_A     : ra_rdata <= #0.1 sa_dst_addr;
                SA_SIZE_ADDR      : ra_rdata <= #0.1 sa_size;
                BEV_PARAMS_ADDR   : ra_rdata <= #0.1 {bev_z, bev_y, bev_x, bev_channels};
                IMG_PARAMS_ADDR   : ra_rdata <= #0.1 {cameras, img_h, img_w};
                XFORM_A00_ADDR    : ra_rdata <= #0.1 xform_a00;
                XFORM_A01_ADDR    : ra_rdata <= #0.1 xform_a01;
                XFORM_A02_ADDR    : ra_rdata <= #0.1 xform_a02;
                XFORM_A10_ADDR    : ra_rdata <= #0.1 xform_a10;
                XFORM_A11_ADDR    : ra_rdata <= #0.1 xform_a11;
                XFORM_A12_ADDR    : ra_rdata <= #0.1 xform_a12;
                FRAME0_ADDR_A     : ra_rdata <= #0.1 frame0_addr;
                FRAME1_ADDR_A     : ra_rdata <= #0.1 frame1_addr;
                FRAME2_ADDR_A     : ra_rdata <= #0.1 frame2_addr;
                FRAME3_ADDR_A     : ra_rdata <= #0.1 frame3_addr;
                FRAME_SIZE_ADDR   : ra_rdata <= #0.1 frame_size;
                COMP_DONE_ADDR    : ra_rdata <= #0.1 {31'd0, comp_done_reg};
                STATUS_ADDR       : ra_rdata <= #0.1 {24'd0, status_reg};
                PERF_CNT_LO_ADDR  : ra_rdata <= #0.1 perf_cnt_lo;
                PERF_CNT_HI_ADDR  : ra_rdata <= #0.1 perf_cnt_hi;
                VERSION_ADDR      : ra_rdata <= #0.1 BEV_VERSION;
                DEBUG_REG_ADDR    : ra_rdata <= #0.1 debug_reg;
                default           : ra_rdata <= #0.1 32'hFFFF_FFFF;
            endcase
        end
    end

endmodule
