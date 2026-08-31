`timescale 1ns/1ps
// ============================================================================
// FastBEV Part2 top-level
// ============================================================================
//
// PORT CHANGES:
//   - itf_awdata / itf_rdata widened from 64-bit to 512-bit to match the
//     actual User_Ddr bus width in ps_ai_wrap_demo (one beat = 64 bytes).
//   - All register-access (itf_ra_*) ports, reset_reg, clocks, and resets
//     remain identical in name and width.
//
// INTERNAL CHANGES:
//   - adder_top instantiates: reg_ctrl + dma + adder (simple accumulator).
//     bev_accel_top instantiates: bev_reg_ctrl + lut_engine + sa_engine +
//     dma_arbiter (two-engine architecture with shared DDR port).
//   - Added 2-bit comp_mode to select engine (01=LUT, 10=SA), plus
//     frame_shift_en for temporal frame rotation.
//   - Added 64-bit performance counter (perf_counter) gated by
//     comp_start / comp_done.
//   - rst_n_1 / ra_rst_n_1 changed from combinational assign to registered
//     outputs with MAX_FANOUT=256 attribute to fix recovery timing on
//     FDCE CLR pins.
// ============================================================================
//   *** DATA BUS WIDTH NOTE ***
//   The demo's adder_top uses 64-bit itf_awdata/itf_rdata, which
//   creates a width mismatch with the 512-bit User_Ddr ports in
//   ps_ai_wrap_demo (Vivado truncates/zero-extends silently).
//   The demo's DMA increments addresses by 64 bytes per beat, confirming
//   each User_Ddr transaction moves a 64-byte (512-bit) block.
//
//   The LUT path consumes vehicle Part1 FP32 NHWC [6,120,160,64] features and
//   writes [1,16,200,200,16] in FP16 (20,480,000 bytes), with BEV Z folded
//   into the 16 cblk16 values. The SA path and mode protocol are unchanged;
//   software selects
//   LUT with CTRL_START=0x03 (mode=01, start=1).
//==============================================================================

module bev_accel_top(

  //------------ Register Access Interface (same as adder_top) ----------
    input        [      31 : 0]   itf_ra_awaddr     , 
    input        [      31 : 0]   itf_ra_awdata     ,
    input                         itf_ra_awvalid    ,
    output                        itf_ra_awready    ,

    input        [      31 : 0]   itf_ra_araddr     ,  
    input                         itf_ra_arvalid    ,
    output                        itf_ra_arready    ,

    output       [      31 : 0]   itf_ra_rdata      ,
    output                        itf_ra_rvalid     ,
    input                         itf_ra_rready     ,
 
  //------------ PLDDR Data Interface (512-bit, matching User_Ddr) ------
    output       [      31 : 0]   itf_awaddr        ,
    output       [     511 : 0]   itf_awdata        ,
    output                        itf_awvalid       ,
    input                         itf_awready       ,

    output       [      31 : 0]   itf_araddr        ,
    output                        itf_arvalid       ,
    input                         itf_arready       ,

    input        [     511 : 0]   itf_rdata         ,
    input                         itf_rvalid        ,
    output                        itf_rready        ,

    output  reg                   reset_reg         ,

    input                         clk               ,  // hp_clk
    input                         ra_clk            ,  // gp_clk
    input                         rst_n             ,
    input                         ra_rst_n
);

    localparam RESET_ADDR = 16'h77;

    // ===================== Internal Signals =====================
    // rst_n_1 / ra_rst_n_1: registered resets with fanout control (was combinational assign)
    (* MAX_FANOUT = 256 *)
    reg               rst_n_1;
    (* MAX_FANOUT = 256 *)
    reg               ra_rst_n_1;
    wire              comp_start, comp_done, comp_done_cross, comp_start_cross;
    wire  [  1 : 0]   comp_mode;
    wire              frame_shift_en;
    (* ASYNC_REG = "TRUE" *) reg [1:0] comp_mode_meta;
    (* ASYNC_REG = "TRUE" *) reg [1:0] comp_mode_sync;
    reg   [  1 : 0]   active_mode_hp;
    reg               comp_start_pending;
    (* ASYNC_REG = "TRUE" *) reg reset_meta_hp;
    (* ASYNC_REG = "TRUE" *) reg reset_sync_hp;

    // Config wires
    wire  [31:0] lut_base_addr, lut_size, feat2d_base_addr, feat3d_wr_addr, feat3d_wr_size;
    wire  [ 7:0] bev_channels, bev_x, bev_y, bev_z;
    wire  [11:0] img_w, img_h;
    wire  [ 7:0] cameras;
    wire  [31:0] sa_src_addr, sa_dst_addr, sa_size;
    wire  [31:0] xform_a00, xform_a01, xform_a02, xform_a10, xform_a11, xform_a12;
    wire  [31:0] frame0_addr, frame1_addr, frame2_addr, frame3_addr, frame_size;

    // hp_clk-domain snapshots. Software writes configuration in gp_clk and
    // leaves it stable before CTRL_START; capturing it one cycle before the
    // engine pulse removes all multi-bit CDC paths from the active datapath.
    reg   [31:0] lut_base_addr_hp, lut_size_hp;
    reg   [31:0] feat2d_base_addr_hp, feat3d_wr_addr_hp;
    reg   [ 7:0] bev_x_hp, bev_y_hp, bev_z_hp;
    reg   [11:0] img_w_hp, img_h_hp;
    reg   [ 7:0] cameras_hp;
    reg   [31:0] sa_src_addr_hp, sa_dst_addr_hp, sa_size_hp;
    reg   [31:0] xform_a00_hp, xform_a01_hp, xform_a02_hp;
    reg   [31:0] xform_a10_hp, xform_a11_hp, xform_a12_hp;

    // Engine signals
    wire  lut_start, sa_start, lut_done, sa_done;

    // LUT engine DMA
    wire  [31:0]  lut_rd_addr;  wire lut_rd_req, lut_rd_grant;
    wire  [511:0] lut_rd_data;  wire lut_rd_data_valid, lut_rd_data_ready;
    wire  [31:0]  lut_wr_addr;  wire [511:0] lut_wr_data; wire lut_wr_req, lut_wr_grant;

    // SA engine DMA
    wire  [31:0]  sa_rd_addr;   wire sa_rd_req, sa_rd_grant;
    wire  [511:0] sa_rd_data;   wire sa_rd_data_valid, sa_rd_data_ready;
    wire  [31:0]  sa_wr_addr;   wire [511:0] sa_wr_data; wire sa_wr_req, sa_wr_grant;

    // Performance
    reg   [63:0] perf_counter;
    reg          perf_running;
    wire  [ 7:0] status_reg;

    // ===================== Engine Control =====================
    assign lut_start = comp_start_pending && (active_mode_hp == 2'b01);
    assign sa_start  = comp_start_pending && (active_mode_hp == 2'b10);
    assign comp_done = lut_done || sa_done;
    assign status_reg = {6'd0, active_mode_hp};

    always @(posedge clk or negedge rst_n_1) begin
        if (!rst_n_1) begin
            comp_mode_meta <= 2'b00;
            comp_mode_sync <= 2'b00;
        end else begin
            comp_mode_meta <= comp_mode;
            comp_mode_sync <= comp_mode_meta;
        end
    end

    always @(posedge clk or negedge rst_n_1) begin
        if (!rst_n_1) begin
            active_mode_hp      <= 2'b00;
            comp_start_pending  <= 1'b0;
            lut_base_addr_hp    <= 32'd0;
            lut_size_hp         <= 32'd160000;
            feat2d_base_addr_hp <= 32'd0;
            feat3d_wr_addr_hp   <= 32'd0;
            bev_x_hp            <= 8'd200;
            bev_y_hp            <= 8'd200;
            bev_z_hp            <= 8'd4;
            img_w_hp            <= 12'd160;
            img_h_hp            <= 12'd120;
            cameras_hp          <= 8'd6;
            sa_src_addr_hp      <= 32'd0;
            sa_dst_addr_hp      <= 32'd0;
            sa_size_hp          <= 32'd40000;
            xform_a00_hp        <= 32'h0001_0000;
            xform_a01_hp        <= 32'd0;
            xform_a02_hp        <= 32'd0;
            xform_a10_hp        <= 32'd0;
            xform_a11_hp        <= 32'h0001_0000;
            xform_a12_hp        <= 32'd0;
        end else begin
            // Delay start by one hp_clk so all snapshots are visible to the
            // engines on their accepted-start edge.
            comp_start_pending <= comp_start_cross;
            if (comp_start_cross) begin
                active_mode_hp      <= comp_mode_sync;
                lut_base_addr_hp    <= lut_base_addr;
                lut_size_hp         <= lut_size;
                feat2d_base_addr_hp <= feat2d_base_addr;
                feat3d_wr_addr_hp   <= feat3d_wr_addr;
                bev_x_hp            <= bev_x;
                bev_y_hp            <= bev_y;
                bev_z_hp            <= bev_z;
                img_w_hp            <= img_w;
                img_h_hp            <= img_h;
                cameras_hp          <= cameras;
                sa_src_addr_hp      <= sa_src_addr;
                sa_dst_addr_hp      <= sa_dst_addr;
                sa_size_hp          <= sa_size;
                xform_a00_hp        <= xform_a00;
                xform_a01_hp        <= xform_a01;
                xform_a02_hp        <= xform_a02;
                xform_a10_hp        <= xform_a10;
                xform_a11_hp        <= xform_a11;
                xform_a12_hp        <= xform_a12;
            end
        end
    end

    // ===================== Performance Counter =====================
    always @(posedge clk or negedge rst_n_1) begin
        if (!rst_n_1) begin
            perf_counter <= 64'd0; perf_running <= 1'b0;
        end else if (lut_start || sa_start) begin
            perf_counter <= 64'd0; perf_running <= 1'b1;
        end else if (comp_done) begin
            perf_running <= 1'b0;
        end else if (perf_running) begin
            perf_counter <= perf_counter + 1'b1;
        end
    end

    // ===================== Register Controller =====================
    bev_reg_ctrl U_bev_reg_ctrl(
        .ra_awaddr       ( itf_ra_awaddr[17:2] ),
        .ra_awdata       ( itf_ra_awdata       ),
        .ra_awvalid      ( itf_ra_awvalid      ),
        .ra_awready      ( itf_ra_awready      ),
        .ra_araddr       ( itf_ra_araddr[17:2] ),
        .ra_arvalid      ( itf_ra_arvalid      ),
        .ra_arready      ( itf_ra_arready      ),
        .ra_rdata        ( itf_ra_rdata        ),
        .ra_rvalid       ( itf_ra_rvalid       ),
        .ra_rready       ( itf_ra_rready       ),
        .comp_start      ( comp_start          ),
        .comp_mode       ( comp_mode           ),
        .frame_shift_en  ( frame_shift_en      ),
        .comp_done       ( comp_done_cross     ),
        .lut_base_addr   ( lut_base_addr       ),
        .lut_size        ( lut_size            ),
        .feat2d_base_addr( feat2d_base_addr    ),
        .feat3d_wr_addr  ( feat3d_wr_addr      ),
        .feat3d_wr_size  ( feat3d_wr_size      ),
        .bev_channels    ( bev_channels        ),
        .bev_x           ( bev_x               ),
        .bev_y           ( bev_y               ),
        .bev_z           ( bev_z               ),
        .img_w           ( img_w               ),
        .img_h           ( img_h               ),
        .cameras         ( cameras             ),
        .sa_src_addr     ( sa_src_addr         ),
        .sa_dst_addr     ( sa_dst_addr         ),
        .sa_size         ( sa_size             ),
        .xform_a00       ( xform_a00           ),
        .xform_a01       ( xform_a01           ),
        .xform_a02       ( xform_a02           ),
        .xform_a10       ( xform_a10           ),
        .xform_a11       ( xform_a11           ),
        .xform_a12       ( xform_a12           ),
        .frame0_addr     ( frame0_addr         ),
        .frame1_addr     ( frame1_addr         ),
        .frame2_addr     ( frame2_addr         ),
        .frame3_addr     ( frame3_addr         ),
        .frame_size      ( frame_size          ),
        .status_reg      ( status_reg          ),
        .perf_cnt_lo     ( perf_counter[31:0]  ),
        .perf_cnt_hi     ( perf_counter[63:32] ),
        .clk             ( ra_clk              ),
        .rst_n           ( ra_rst_n_1          )
    );

    // ===================== LUT Engine =====================
    lut_engine U_lut_engine(
        .engine_start    ( lut_start           ),
        .engine_done     ( lut_done            ),
        .lut_base_addr   ( lut_base_addr_hp    ),
        .lut_size        ( lut_size_hp         ),
        .feat2d_base_addr( feat2d_base_addr_hp ),
        .feat3d_wr_addr  ( feat3d_wr_addr_hp   ),
        .img_w           ( img_w_hp            ),
        .img_h           ( img_h_hp            ),
        .cameras         ( cameras_hp          ),
        .bev_x           ( bev_x_hp            ),
        .bev_y           ( bev_y_hp            ),
        .bev_z           ( bev_z_hp            ),
        .rd_addr         ( lut_rd_addr         ),
        .rd_req          ( lut_rd_req          ),
        .rd_grant        ( lut_rd_grant        ),
        .rd_data         ( lut_rd_data         ),
        .rd_data_valid   ( lut_rd_data_valid   ),
        .rd_data_ready   ( lut_rd_data_ready   ),
        .wr_addr         ( lut_wr_addr         ),
        .wr_data         ( lut_wr_data         ),
        .wr_req          ( lut_wr_req          ),
        .wr_grant        ( lut_wr_grant        ),
        .clk             ( clk                 ),
        .rst_n           ( rst_n_1             )
    );

    // ===================== SA Engine =====================
    sa_engine U_sa_engine(
        .engine_start    ( sa_start            ),
        .engine_done     ( sa_done             ),
        .sa_src_addr     ( sa_src_addr_hp      ),
        .sa_dst_addr     ( sa_dst_addr_hp      ),
        .sa_size         ( sa_size_hp           ),
        .bev_x           ( bev_x_hp            ),
        .bev_y           ( bev_y_hp            ),
        .bev_z           ( bev_z_hp            ),
        .xform_a00       ( xform_a00_hp        ),
        .xform_a01       ( xform_a01_hp        ),
        .xform_a02       ( xform_a02_hp        ),
        .xform_a10       ( xform_a10_hp        ),
        .xform_a11       ( xform_a11_hp        ),
        .xform_a12       ( xform_a12_hp        ),
        .rd_addr         ( sa_rd_addr          ),
        .rd_req          ( sa_rd_req           ),
        .rd_grant        ( sa_rd_grant         ),
        .rd_data         ( sa_rd_data          ),
        .rd_data_valid   ( sa_rd_data_valid    ),
        .rd_data_ready   ( sa_rd_data_ready    ),
        .wr_addr         ( sa_wr_addr          ),
        .wr_data         ( sa_wr_data          ),
        .wr_req          ( sa_wr_req           ),
        .wr_grant        ( sa_wr_grant         ),
        .clk             ( clk                 ),
        .rst_n           ( rst_n_1             )
    );

    // ===================== DMA Arbiter =====================
    dma_arbiter U_dma_arbiter(
        .active_engine      ( active_mode_hp      ),
        .lut_rd_addr        ( lut_rd_addr         ),
        .lut_rd_req         ( lut_rd_req          ),
        .lut_rd_grant       ( lut_rd_grant        ),
        .lut_rd_data        ( lut_rd_data         ),
        .lut_rd_data_valid  ( lut_rd_data_valid   ),
        .lut_rd_data_ready  ( lut_rd_data_ready   ),
        .lut_wr_addr        ( lut_wr_addr         ),
        .lut_wr_data        ( lut_wr_data         ),
        .lut_wr_req         ( lut_wr_req          ),
        .lut_wr_grant       ( lut_wr_grant        ),
        .sa_rd_addr         ( sa_rd_addr          ),
        .sa_rd_req          ( sa_rd_req           ),
        .sa_rd_grant        ( sa_rd_grant         ),
        .sa_rd_data         ( sa_rd_data          ),
        .sa_rd_data_valid   ( sa_rd_data_valid    ),
        .sa_rd_data_ready   ( sa_rd_data_ready    ),
        .sa_wr_addr         ( sa_wr_addr          ),
        .sa_wr_data         ( sa_wr_data          ),
        .sa_wr_req          ( sa_wr_req           ),
        .sa_wr_grant        ( sa_wr_grant         ),
        .ddr_araddr         ( itf_araddr          ),
        .ddr_arvalid        ( itf_arvalid         ),
        .ddr_arready        ( itf_arready         ),
        .ddr_rdata          ( itf_rdata           ),
        .ddr_rvalid         ( itf_rvalid          ),
        .ddr_rready         ( itf_rready          ),
        .ddr_awaddr         ( itf_awaddr          ),
        .ddr_awdata         ( itf_awdata          ),
        .ddr_awvalid        ( itf_awvalid         ),
        .ddr_awready        ( itf_awready         ),
        .clk                ( clk                 ),
        .rst_n              ( rst_n_1             )
    );

    // ===================== Clock Domain Crossing =====================
    pulse_cross U0_pulse_cross(
        .a2(comp_done_cross), .clk2(ra_clk), .rst2(~ra_rst_n_1),
        .rdy1(), .a1(comp_done), .clk1(clk), .rst1(~rst_n_1)
    );
    pulse_cross U1_pulse_cross(
        .a2(comp_start_cross), .clk2(clk), .rst2(~rst_n_1),
        .rdy1(), .a1(comp_start), .clk1(ra_clk), .rst1(~ra_rst_n_1)
    );

    // ===================== Software Reset (same as demo) =====================
    always @(posedge ra_clk or negedge ra_rst_n) begin
        if (ra_rst_n == 1'b0)
            reset_reg <= #0.1 1'b0;
        else if (itf_ra_awvalid && itf_ra_awready && itf_ra_awaddr[17:2] == RESET_ADDR)
            reset_reg <= #0.1 itf_ra_awdata[0];
    end

    // ===================== Registered Reset Outputs (timing fix) =====================
    // rst_n_1: hp_clk domain, combines software reset with hardware reset
    // Registered to reduce fanout and meet recovery timing on FDCE CLR pins
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reset_meta_hp <= 1'b0;
            reset_sync_hp <= 1'b0;
            rst_n_1       <= 1'b0;
        end else begin
            reset_meta_hp <= reset_reg;
            reset_sync_hp <= reset_meta_hp;
            rst_n_1       <= ~reset_sync_hp;
        end
    end
    // ra_rst_n_1: gp_clk domain, same logic
    always @(posedge ra_clk or negedge ra_rst_n) begin
        if (!ra_rst_n) ra_rst_n_1 <= 1'b0;
        else           ra_rst_n_1 <= ~reset_reg;
    end

endmodule
