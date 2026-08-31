// ============================================================================
// bev_accel_top vs adder_top - Key Differences
// ============================================================================
//
// PORT CHANGES:
//   - itf_awdata / itf_rdata widened from 64-bit to 512-bit to match the
//     actual User_Ddr bus width in ps_ai_wrap_demo (one beat = 64 bytes =
//     one voxel of 64 x int8 channels).
//   - All register-access (itf_ra_*) ports, reset_reg, clocks, and resets
//     remain identical in name and width.
//
// INTERNAL CHANGES:
//   - adder_top instantiates: reg_ctrl + dma + adder (simple accumulator).
//     bev_accel_top instantiates: bev_reg_ctrl + INT8 Quant/LUT/SA +
//     pipeline_ctrl_group4 + dma_arbiter_stage.
//   - CTRL_START now enters the INT8 group4 path only.
//   - Added performance counters for total/stage/DDR wait/grant cycles.
//   - rst_n_1 / ra_rst_n_1 are registered reset controls. The hp_clk Part2
//     consumers use synchronous reset so this net does not create high-fanout
//     asynchronous recovery paths or block DSP register packing.
// ============================================================================
//   *** DATA BUS WIDTH NOTE ***
//   The demo's adder_top uses 64-bit itf_awdata/itf_rdata, which
//   creates a width mismatch with the 512-bit User_Ddr ports in
//   ps_ai_wrap_demo (Vivado truncates/zero-extends silently).
//   The demo's DMA increments addresses by 64 bytes per beat, confirming
//   each User_Ddr transaction moves a 64-byte (512-bit) block.
//
//   Our design uses the FULL 512-bit data interface because:
//   - ps_ai_wrap_demo's User_Ddr ports are defined as 512-bit
//   - One voxel = 64 channels x int8 = 64 bytes = 512 bits = 1 beat
//   - Physical DDR3 is 1600MT/s x 64-bit; MIG internally converts
//     to 512-bit AXI at lower freq (standard DDR controller practice)
//==============================================================================

`timescale 1ns / 1ps

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
    // Registered reset controls; Part2 hp_clk consumers use synchronous reset.
    (* MAX_FANOUT = 256 *)
    reg               rst_n_1;
    (* MAX_FANOUT = 256 *)
    reg               ra_rst_n_1;
    reg               reset_reg_clk_d1;
    reg               reset_reg_clk_d2;
    wire              comp_start, comp_done, comp_done_cross, comp_start_cross;
    wire  [  1 : 0]   comp_mode;
    wire              frame_shift_en;

    // Config wires
    wire  [31:0] lut_base_addr, lut_size, feat2d_base_addr, feat3d_wr_addr, feat3d_wr_size;
    wire  [ 7:0] bev_channels, bev_x, bev_y, bev_z;
    wire  [11:0] img_w, img_h;
    wire  [ 7:0] cameras;
    wire  [31:0] sa_src_addr, sa_dst_addr, sa_size;
    wire  [31:0] xform_a00, xform_a01, xform_a02, xform_a10, xform_a11, xform_a12;
    wire  [31:0] frame0_addr, frame1_addr, frame2_addr, frame3_addr, frame_size;
    wire  [ 2:0] ext_mode;
    wire  [31:0] feat2d_fp32_base, feat2d_int8_base, concat_out_base;
    wire  [31:0] group_ctrl, group_status, group_error_status;
    wire  [31:0] error_status_w1c;
    wire         error_status_w1c_valid;
    wire  [31:0] perf_stage_sel;
    wire  [31:0] xform_h0_a00, xform_h0_a01, xform_h0_a02, xform_h0_a10, xform_h0_a11, xform_h0_a12;
    wire  [31:0] xform_h1_a00, xform_h1_a01, xform_h1_a02, xform_h1_a10, xform_h1_a11, xform_h1_a12;
    wire  [31:0] xform_h2_a00, xform_h2_a01, xform_h2_a02, xform_h2_a10, xform_h2_a11, xform_h2_a12;

    // Engine control
    wire  group_quant_start, group_lut_start, group_sa_start, group_done;
    wire  quant_done, int8_lut_done, int8_sa_done;

    // Pipeline controller outputs
    wire  [ 1:0]  group_active_stage;
    wire  [31:0]  quant_fp32_base, quant_int8_base, quant_total_pixels;
    wire  [31:0]  group_lut_feat2d_base, group_lut_dst_base;
    wire          group_lut_dst_mode;
    wire  [31:0]  group_sa_src_addr, group_sa_concat_base;
    wire  [ 1:0]  group_sa_temporal_idx;
    wire          error_status_w1c_valid_cross;

    // Selected stage interface to DMA arbiter
    wire  [ 1:0]  active_stage;
    wire  [31:0]  lut_rd_addr;  wire lut_rd_req, lut_rd_grant;
    wire  [511:0] lut_rd_data;  wire lut_rd_data_valid, lut_rd_data_ready;
    wire  [31:0]  lut_wr_addr;  wire [511:0] lut_wr_data; wire lut_wr_req, lut_wr_grant;
    wire  [31:0]  sa_rd_addr;   wire sa_rd_req, sa_rd_grant;
    wire  [511:0] sa_rd_data;   wire sa_rd_data_valid, sa_rd_data_ready;
    wire  [31:0]  sa_wr_addr;   wire [511:0] sa_wr_data; wire sa_wr_req, sa_wr_grant;

    // Quant engine DMA
    wire  [31:0]  quant_rd_addr; wire quant_rd_req, quant_rd_grant;
    wire  [511:0] quant_rd_data; wire quant_rd_data_valid, quant_rd_data_ready;
    wire  [31:0]  quant_wr_addr; wire [511:0] quant_wr_data; wire quant_wr_req, quant_wr_grant;

    // INT8 LUT engine DMA
    wire  [31:0]  int8_lut_rd_addr; wire int8_lut_rd_req, int8_lut_rd_grant;
    wire  [511:0] int8_lut_rd_data; wire int8_lut_rd_data_valid, int8_lut_rd_data_ready;
    wire  [31:0]  int8_lut_wr_addr; wire [511:0] int8_lut_wr_data; wire int8_lut_wr_req, int8_lut_wr_grant;

    // INT8 SA engine DMA
    wire  [31:0]  int8_sa_rd_addr; wire int8_sa_rd_req, int8_sa_rd_grant;
    wire  [511:0] int8_sa_rd_data; wire int8_sa_rd_data_valid, int8_sa_rd_data_ready;
    wire  [31:0]  int8_sa_wr_addr; wire [511:0] int8_sa_wr_data; wire int8_sa_wr_req, int8_sa_wr_grant;

    wire  [31:0]  int8_sa_xform_a00, int8_sa_xform_a01, int8_sa_xform_a02;
    wire  [31:0]  int8_sa_xform_a10, int8_sa_xform_a11, int8_sa_xform_a12;

    // Performance
    reg   [63:0] perf_counter;
    reg   [63:0] perf_quant_cycles;
    reg   [63:0] perf_lut_cycles;
    reg   [63:0] perf_sa_cycles;
    reg   [63:0] perf_ddr_rd_wait;
    reg   [63:0] perf_ddr_wr_wait;
    reg   [63:0] perf_ddr_rd_grant;
    reg   [63:0] perf_ddr_wr_grant;
    reg          perf_running;
    reg   [63:0] perf_selected;
    wire  [ 7:0] status_reg;

    // ===================== Engine Control =====================
    assign comp_done        = group_done;
    assign status_reg       = group_status[7:0];
    assign active_stage     = group_active_stage;

    assign lut_rd_addr       = int8_lut_rd_addr;
    assign lut_rd_req        = int8_lut_rd_req;
    assign lut_wr_addr       = int8_lut_wr_addr;
    assign lut_wr_data       = int8_lut_wr_data;
    assign lut_wr_req        = int8_lut_wr_req;
    assign lut_rd_data_ready = int8_lut_rd_data_ready;
    assign int8_lut_rd_grant      = lut_rd_grant;
    assign int8_lut_wr_grant      = lut_wr_grant;
    assign int8_lut_rd_data       = lut_rd_data;
    assign int8_lut_rd_data_valid = lut_rd_data_valid;

    assign sa_rd_addr       = int8_sa_rd_addr;
    assign sa_rd_req        = int8_sa_rd_req;
    assign sa_wr_addr       = int8_sa_wr_addr;
    assign sa_wr_data       = int8_sa_wr_data;
    assign sa_wr_req        = int8_sa_wr_req;
    assign sa_rd_data_ready = int8_sa_rd_data_ready;
    assign int8_sa_rd_grant      = sa_rd_grant;
    assign int8_sa_wr_grant      = sa_wr_grant;
    assign int8_sa_rd_data       = sa_rd_data;
    assign int8_sa_rd_data_valid = sa_rd_data_valid;

    assign int8_sa_xform_a00 = (group_sa_temporal_idx == 2'd3) ? xform_h0_a00 :
                               (group_sa_temporal_idx == 2'd2) ? xform_h1_a00 : xform_h2_a00;
    assign int8_sa_xform_a01 = (group_sa_temporal_idx == 2'd3) ? xform_h0_a01 :
                               (group_sa_temporal_idx == 2'd2) ? xform_h1_a01 : xform_h2_a01;
    assign int8_sa_xform_a02 = (group_sa_temporal_idx == 2'd3) ? xform_h0_a02 :
                               (group_sa_temporal_idx == 2'd2) ? xform_h1_a02 : xform_h2_a02;
    assign int8_sa_xform_a10 = (group_sa_temporal_idx == 2'd3) ? xform_h0_a10 :
                               (group_sa_temporal_idx == 2'd2) ? xform_h1_a10 : xform_h2_a10;
    assign int8_sa_xform_a11 = (group_sa_temporal_idx == 2'd3) ? xform_h0_a11 :
                               (group_sa_temporal_idx == 2'd2) ? xform_h1_a11 : xform_h2_a11;
    assign int8_sa_xform_a12 = (group_sa_temporal_idx == 2'd3) ? xform_h0_a12 :
                               (group_sa_temporal_idx == 2'd2) ? xform_h1_a12 : xform_h2_a12;

    // ===================== Performance Counter =====================
    always @(posedge clk) begin
        if (!rst_n_1) begin
            perf_counter      <= 64'd0;
            perf_quant_cycles <= 64'd0;
            perf_lut_cycles   <= 64'd0;
            perf_sa_cycles    <= 64'd0;
            perf_ddr_rd_wait  <= 64'd0;
            perf_ddr_wr_wait  <= 64'd0;
            perf_ddr_rd_grant <= 64'd0;
            perf_ddr_wr_grant <= 64'd0;
            perf_running      <= 1'b0;
        end else if (comp_start_cross) begin
            perf_counter      <= 64'd0;
            perf_quant_cycles <= 64'd0;
            perf_lut_cycles   <= 64'd0;
            perf_sa_cycles    <= 64'd0;
            perf_ddr_rd_wait  <= 64'd0;
            perf_ddr_wr_wait  <= 64'd0;
            perf_ddr_rd_grant <= 64'd0;
            perf_ddr_wr_grant <= 64'd0;
            perf_running      <= 1'b1;
        end else if (comp_done) begin
            perf_running <= 1'b0;
        end else if (perf_running) begin
            perf_counter <= perf_counter + 1'b1;
            if(active_stage == 2'b01) perf_quant_cycles <= perf_quant_cycles + 1'b1;
            if(active_stage == 2'b10) perf_lut_cycles   <= perf_lut_cycles + 1'b1;
            if(active_stage == 2'b11) perf_sa_cycles    <= perf_sa_cycles + 1'b1;
            if(itf_arvalid && !itf_arready) perf_ddr_rd_wait <= perf_ddr_rd_wait + 1'b1;
            if(itf_awvalid && !itf_awready) perf_ddr_wr_wait <= perf_ddr_wr_wait + 1'b1;
            if(itf_arvalid &&  itf_arready) perf_ddr_rd_grant <= perf_ddr_rd_grant + 1'b1;
            if(itf_awvalid &&  itf_awready) perf_ddr_wr_grant <= perf_ddr_wr_grant + 1'b1;
        end
    end

    always @(*) begin
        case(perf_stage_sel[3:0])
            4'h0: perf_selected = perf_counter;
            4'h1: perf_selected = perf_quant_cycles;
            4'h2: perf_selected = perf_lut_cycles;
            4'h3: perf_selected = perf_sa_cycles;
            4'h4: perf_selected = perf_ddr_rd_wait;
            4'h5: perf_selected = perf_ddr_wr_wait;
            4'h6: perf_selected = perf_ddr_rd_grant;
            4'h7: perf_selected = perf_ddr_wr_grant;
            default: perf_selected = perf_counter;
        endcase
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
        .ext_mode        ( ext_mode            ),
        .feat2d_fp32_base( feat2d_fp32_base    ),
        .feat2d_int8_base( feat2d_int8_base    ),
        .concat_out_base ( concat_out_base     ),
        .group_ctrl      ( group_ctrl          ),
        .group_status    ( group_status        ),
        .group_error_status( group_error_status),
        .error_status_w1c( error_status_w1c    ),
        .error_status_w1c_valid( error_status_w1c_valid ),
        .perf_stage_sel  ( perf_stage_sel      ),
        .xform_h0_a00    ( xform_h0_a00        ),
        .xform_h0_a01    ( xform_h0_a01        ),
        .xform_h0_a02    ( xform_h0_a02        ),
        .xform_h0_a10    ( xform_h0_a10        ),
        .xform_h0_a11    ( xform_h0_a11        ),
        .xform_h0_a12    ( xform_h0_a12        ),
        .xform_h1_a00    ( xform_h1_a00        ),
        .xform_h1_a01    ( xform_h1_a01        ),
        .xform_h1_a02    ( xform_h1_a02        ),
        .xform_h1_a10    ( xform_h1_a10        ),
        .xform_h1_a11    ( xform_h1_a11        ),
        .xform_h1_a12    ( xform_h1_a12        ),
        .xform_h2_a00    ( xform_h2_a00        ),
        .xform_h2_a01    ( xform_h2_a01        ),
        .xform_h2_a02    ( xform_h2_a02        ),
        .xform_h2_a10    ( xform_h2_a10        ),
        .xform_h2_a11    ( xform_h2_a11        ),
        .xform_h2_a12    ( xform_h2_a12        ),
        .status_reg      ( status_reg          ),
        .perf_cnt_lo     ( perf_selected[31:0] ),
        .perf_cnt_hi     ( perf_selected[63:32]),
        .clk             ( ra_clk              ),
        .rst_n           ( ra_rst_n_1          )
    );

    // ===================== Group4 Pipeline Controller =====================
    pipeline_ctrl_group4 U_pipeline_ctrl_group4(
        .clk                    ( clk                       ),
        .rst_n                  ( rst_n_1                   ),
        .group_start            ( comp_start_cross          ),
        .comp_mode              ( 2'b11                     ),
        .ext_mode               ( ext_mode                  ),
        .group_ctrl             ( group_ctrl                ),
        .feat2d_fp32_base       ( feat2d_fp32_base          ),
        .feat2d_int8_base       ( feat2d_int8_base          ),
        .concat_out_base        ( concat_out_base           ),
        .lut_base_addr          ( lut_base_addr             ),
        .lut_size               ( lut_size                  ),
        .frame0_addr            ( frame0_addr               ),
        .frame1_addr            ( frame1_addr               ),
        .frame2_addr            ( frame2_addr               ),
        .frame_size             ( frame_size                ),
        .img_w                  ( img_w                     ),
        .img_h                  ( img_h                     ),
        .cameras                ( cameras                   ),
        .quant_done             ( quant_done                ),
        .lut_done               ( int8_lut_done             ),
        .sa_done                ( int8_sa_done              ),
        .error_status_w1c       ( error_status_w1c          ),
        .error_status_w1c_valid ( error_status_w1c_valid_cross ),
        .quant_start            ( group_quant_start         ),
        .lut_start              ( group_lut_start           ),
        .sa_start               ( group_sa_start            ),
        .group_done             ( group_done                ),
        .active_stage           ( group_active_stage        ),
        .quant_fp32_base        ( quant_fp32_base           ),
        .quant_int8_base        ( quant_int8_base           ),
        .quant_total_pixels     ( quant_total_pixels        ),
        .lut_feat2d_base        ( group_lut_feat2d_base     ),
        .lut_dst_base           ( group_lut_dst_base        ),
        .lut_dst_mode           ( group_lut_dst_mode        ),
        .sa_src_addr            ( group_sa_src_addr         ),
        .sa_concat_base         ( group_sa_concat_base      ),
        .sa_temporal_idx        ( group_sa_temporal_idx     ),
        .group_status           ( group_status              ),
        .error_status           ( group_error_status        )
    );

    // ===================== Quant Engine =====================
    quant_engine U_quant_engine(
        .engine_start   ( group_quant_start       ),
        .engine_done    ( quant_done              ),
        .fp32_base_addr  ( quant_fp32_base         ),
        .int8_base_addr  ( quant_int8_base         ),
        .total_pixels    ( quant_total_pixels      ),
        .rd_addr         ( quant_rd_addr           ),
        .rd_req          ( quant_rd_req            ),
        .rd_grant        ( quant_rd_grant          ),
        .rd_data         ( quant_rd_data           ),
        .rd_data_valid   ( quant_rd_data_valid     ),
        .rd_data_ready   ( quant_rd_data_ready     ),
        .wr_addr         ( quant_wr_addr           ),
        .wr_data         ( quant_wr_data           ),
        .wr_req          ( quant_wr_req            ),
        .wr_grant        ( quant_wr_grant          ),
        .clk             ( clk                     ),
        .rst_n           ( rst_n_1                 )
    );

    // ===================== INT8 LUT Engine =====================
    lut_engine_int8 U_lut_engine_int8(
        .engine_start    ( group_lut_start        ),
        .engine_done     ( int8_lut_done          ),
        .lut_base_addr   ( lut_base_addr          ),
        .lut_size        ( lut_size               ),
        .feat2d_base_addr( group_lut_feat2d_base  ),
        .dst_base_addr   ( group_lut_dst_base     ),
        .dst_mode        ( group_lut_dst_mode     ),
        .img_w           ( img_w                  ),
        .img_h           ( img_h                  ),
        .bev_x           ( bev_x                  ),
        .bev_y           ( bev_y                  ),
        .rd_addr         ( int8_lut_rd_addr       ),
        .rd_req          ( int8_lut_rd_req        ),
        .rd_grant        ( int8_lut_rd_grant      ),
        .rd_data         ( int8_lut_rd_data       ),
        .rd_data_valid   ( int8_lut_rd_data_valid ),
        .rd_data_ready   ( int8_lut_rd_data_ready ),
        .wr_addr         ( int8_lut_wr_addr       ),
        .wr_data         ( int8_lut_wr_data       ),
        .wr_req          ( int8_lut_wr_req        ),
        .wr_grant        ( int8_lut_wr_grant      ),
        .clk             ( clk                    ),
        .rst_n           ( rst_n_1                )
    );

    // ===================== INT8 SA Engine =====================
    // In group4 mode, LUT_SIZE is the BEV voxel count processed by INT8 LUT.
    // SA must cover the same voxel count to write temporal beats 0/1/2.
    sa_engine_int8 U_sa_engine_int8(
        .engine_start    ( group_sa_start          ),
        .engine_done     ( int8_sa_done            ),
        .sa_src_addr     ( group_sa_src_addr       ),
        .concat_base_addr( group_sa_concat_base    ),
        .sa_size         ( lut_size                ),
        .bev_x           ( bev_x                   ),
        .bev_y           ( bev_y                   ),
        .bev_z           ( bev_z                   ),
        .temporal_idx    ( group_sa_temporal_idx   ),
        .xform_a00       ( int8_sa_xform_a00       ),
        .xform_a01       ( int8_sa_xform_a01       ),
        .xform_a02       ( int8_sa_xform_a02       ),
        .xform_a10       ( int8_sa_xform_a10       ),
        .xform_a11       ( int8_sa_xform_a11       ),
        .xform_a12       ( int8_sa_xform_a12       ),
        .rd_addr         ( int8_sa_rd_addr         ),
        .rd_req          ( int8_sa_rd_req          ),
        .rd_grant        ( int8_sa_rd_grant        ),
        .rd_data         ( int8_sa_rd_data         ),
        .rd_data_valid   ( int8_sa_rd_data_valid   ),
        .rd_data_ready   ( int8_sa_rd_data_ready   ),
        .wr_addr         ( int8_sa_wr_addr         ),
        .wr_data         ( int8_sa_wr_data         ),
        .wr_req          ( int8_sa_wr_req          ),
        .wr_grant        ( int8_sa_wr_grant        ),
        .clk             ( clk                     ),
        .rst_n           ( rst_n_1                 )
    );

    // ===================== Stage DMA Arbiter =====================
    dma_arbiter_stage U_dma_arbiter_stage(
        .active_stage        ( active_stage          ),
        .quant_rd_addr       ( quant_rd_addr         ),
        .quant_rd_req        ( quant_rd_req          ),
        .quant_rd_grant      ( quant_rd_grant        ),
        .quant_rd_data       ( quant_rd_data         ),
        .quant_rd_data_valid ( quant_rd_data_valid   ),
        .quant_rd_data_ready ( quant_rd_data_ready   ),
        .quant_wr_addr       ( quant_wr_addr         ),
        .quant_wr_data       ( quant_wr_data         ),
        .quant_wr_req        ( quant_wr_req          ),
        .quant_wr_grant      ( quant_wr_grant        ),
        .lut_rd_addr         ( lut_rd_addr           ),
        .lut_rd_req          ( lut_rd_req            ),
        .lut_rd_grant        ( lut_rd_grant          ),
        .lut_rd_data         ( lut_rd_data           ),
        .lut_rd_data_valid   ( lut_rd_data_valid     ),
        .lut_rd_data_ready   ( lut_rd_data_ready     ),
        .lut_wr_addr         ( lut_wr_addr           ),
        .lut_wr_data         ( lut_wr_data           ),
        .lut_wr_req          ( lut_wr_req            ),
        .lut_wr_grant        ( lut_wr_grant          ),
        .sa_rd_addr          ( sa_rd_addr            ),
        .sa_rd_req           ( sa_rd_req             ),
        .sa_rd_grant         ( sa_rd_grant           ),
        .sa_rd_data          ( sa_rd_data            ),
        .sa_rd_data_valid    ( sa_rd_data_valid      ),
        .sa_rd_data_ready    ( sa_rd_data_ready      ),
        .sa_wr_addr          ( sa_wr_addr            ),
        .sa_wr_data          ( sa_wr_data            ),
        .sa_wr_req           ( sa_wr_req             ),
        .sa_wr_grant         ( sa_wr_grant           ),
        .ddr_araddr          ( itf_araddr            ),
        .ddr_arvalid         ( itf_arvalid           ),
        .ddr_arready         ( itf_arready           ),
        .ddr_rdata           ( itf_rdata             ),
        .ddr_rvalid          ( itf_rvalid            ),
        .ddr_rready          ( itf_rready            ),
        .ddr_awaddr          ( itf_awaddr            ),
        .ddr_awdata          ( itf_awdata            ),
        .ddr_awvalid         ( itf_awvalid           ),
        .ddr_awready         ( itf_awready           ),
        .clk                 ( clk                   ),
        .rst_n               ( rst_n_1               )
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
    pulse_cross U2_pulse_cross(
        .a2(error_status_w1c_valid_cross), .clk2(clk), .rst2(~rst_n_1),
        .rdy1(), .a1(error_status_w1c_valid), .clk1(ra_clk), .rst1(~ra_rst_n_1)
    );

    // ===================== Software Reset (same as demo) =====================
    // reset_reg is written by the register-access interface, so capture it in
    // ra_clk and then synchronize it into hp_clk for the datapath reset.
    always @(posedge ra_clk or negedge ra_rst_n) begin
        if (ra_rst_n == 1'b0)
            reset_reg <= #0.1 1'b0;
        else if (itf_ra_awvalid && itf_ra_awready && itf_ra_awaddr[17:2] == RESET_ADDR)
            reset_reg <= #0.1 itf_ra_awdata[0];
    end

    // ===================== Registered Reset Outputs (timing fix) =====================
    // rst_n_1: hp_clk domain, combines software reset with hardware reset.
    // Registered to reduce fanout and meet recovery timing on FDCE CLR pins.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reset_reg_clk_d1 <= 1'b0;
            reset_reg_clk_d2 <= 1'b0;
            rst_n_1          <= 1'b0;
        end else begin
            reset_reg_clk_d1 <= reset_reg;
            reset_reg_clk_d2 <= reset_reg_clk_d1;
            rst_n_1          <= ~reset_reg_clk_d2;
        end
    end
    // ra_rst_n_1: gp_clk domain, same logic
    always @(posedge ra_clk or negedge ra_rst_n) begin
        if (!ra_rst_n) ra_rst_n_1 <= 1'b0;
        else           ra_rst_n_1 <= ~reset_reg;
    end

endmodule
