// ============================================================================
// bev_edif_top vs ai7030_edif_top — Key Differences
// ============================================================================
//
// PORT CHANGES:
//   - Removed 3 SDI-related top-level inputs that require GTX transceivers:
//       FMC_HPC_DP0_M2C_N/P   (SDI data)
//       FMC_HPC_GBTCLK0_M2C_C_N/P  (SDI reference clock)
//       rx_gtx_full_reset      (SDI reset)
//     Removing these avoids Vivado DRC errors when no GTX primitive is
//     present (Part2 does not use SDI camera input).
//   - All other top-level ports (PS DDR, PL DDR3, sys_clk, HDMI, LED,
//     PJTAG) are unchanged.
//
// INTERNAL CHANGES:
//   - rx_sdi instance removed entirely; cam_vs/cam_rgb/cam_data_en tied
//     to constant 0, cam_clk tied to gp_clk (keeps ps_ai_wrap_demo's
//     camera input port from floating).
//   - Custom operator swapped from adder_top (64-bit data bus) to
//     bev_accel_top (512-bit data bus matching User_Ddr width).
//   - KEEP-attributed intermediate wires (led1, led2, led3, cam_hs,
//     cam_rgb, cam_clk, cam_data_en, cam_vs) removed since rx_sdi no
//     longer drives them.
//   - User_Ddr_awwinfo / User_Ddr_arinfo assignments are identical.
// ============================================================================


module bev_edif_top (
  // =============== PS DDR (必须) ===============
  inout     [53 : 0]     FIXED_IO_0_mio            ,
  inout     [14 : 0]     DDR_0_addr                , 
  inout     [2  : 0]     DDR_0_ba                  , 
  inout                  DDR_0_cas_n               , 
  inout                  DDR_0_ck_n                , 
  inout                  DDR_0_ck_p                , 
  inout                  DDR_0_cke                 , 
  inout                  DDR_0_cs_n                , 
  inout     [3  : 0]     DDR_0_dm                  , 
  inout     [31 : 0]     DDR_0_dq                  , 
  inout     [3  : 0]     DDR_0_dqs_n               , 
  inout     [3  : 0]     DDR_0_dqs_p               , 
  inout                  DDR_0_odt                 , 
  inout                  DDR_0_ras_n               , 
  inout                  DDR_0_reset_n             , 
  inout                  DDR_0_we_n                , 
  inout                  FIXED_IO_0_ddr_vrn        , 
  inout                  FIXED_IO_0_ddr_vrp        , 
  inout                  FIXED_IO_0_ps_clk         ,
  inout                  FIXED_IO_0_ps_porb        ,
  inout                  FIXED_IO_0_ps_srstb       ,
         
  // =============== PL DDR3 (必须 �? MIG控制�?) ===============
  inout     [ 63 : 0]    ddr3_dq                   ,
  inout     [  7 : 0]    ddr3_dqs_n                ,
  inout     [  7 : 0]    ddr3_dqs_p                ,
  output    [  7 : 0]    ddr3_dm                   ,
  output    [ 14 : 0]    ddr3_addr                 ,
  output    [  2 : 0]    ddr3_ba                   ,
  output                 ddr3_ras_n                ,
  output                 ddr3_cas_n                ,
  output                 ddr3_we_n                 ,
  output                 ddr3_reset_n              ,
  output    [  0 : 0]    ddr3_ck_p                 ,
  output    [  0 : 0]    ddr3_ck_n                 ,
  output    [  0 : 0]    ddr3_cke                  ,
  output    [  0 : 0]    ddr3_cs_n                 ,
  output    [  0 : 0]    ddr3_odt                  ,
       
  // =============== 系统时钟与复�? (MIG�?�?) ===============
  input                  sys_rst                   ,
  input                  sys_clk_p                 ,
  input                  sys_clk_n                 ,

  // =============== HDMI (ps_ai_wrap_demo输出口，保留) ===============
  output                 HDMI_CLK_P                ,    
  output                 HDMI_CLK_N                , 
  output [2:0]           HDMI_TX_P                 , 
  output [2:0]           HDMI_TX_N                 ,      
 
  // =============== LED (ps_ai_wrap_demo输出口，保留) ===============
  output    [  1 : 0]    led                       ,

  // =============== PJTAG ===============
  input                  PJTAG_TCK                 ,
  input                  PJTAG_TMS                 ,
  input                  PJTAG_TDI                 ,
  output                 PJTAG_TDO                 

  // 已删除（Part2不需要，且GTX引脚无原语会DRC报错）：
  //   FMC_HPC_DP0_M2C_N/P         �? SDI GTX数据
  //   FMC_HPC_GBTCLK0_M2C_C_N/P   �? SDI GTX参�?�时�?
  //   rx_gtx_full_reset            �? SDI复位
);

  // ===================== User_Reg Bus (32-bit, GP) =====================
  wire   [ 31 : 0]  User_Reg_awwaddr  ;
  wire   [ 31 : 0]  User_Reg_awwdata  ;
  wire              User_Reg_awwvalid ;
  wire              User_Reg_awwready ;
  wire   [ 31 : 0]  User_Reg_araddr   ;
  wire              User_Reg_arvalid  ;
  wire              User_Reg_arready  ;
  wire   [ 31 : 0]  User_Reg_rdata    ;
  wire              User_Reg_rvalid   ;
  wire              User_Reg_rready   ;

  // ===================== User_Ddr Bus (512-bit, HP) =====================
  wire   [ 31 : 0]  User_Ddr_awwaddr  ;
  wire   [511 : 0]  User_Ddr_awwdata  ;
  wire   [ 71 : 0]  User_Ddr_awwinfo  ;
  wire              User_Ddr_awwvalid ;
  wire              User_Ddr_awwready ;
  wire   [ 31 : 0]  User_Ddr_araddr   ;
  wire              User_Ddr_arvalid  ;
  wire   [ 31 : 0]  User_Ddr_arinfo   ;
  wire              User_Ddr_arready  ;
  wire   [511 : 0]  User_Ddr_rdata    ;
  wire              User_Ddr_rvalid   ;
  wire   [ 31 : 0]  User_Ddr_rinfo    ;
  wire              User_Ddr_rready   ;

  // ===================== Clocks & Resets =====================
  wire              gp_clk, hp_clk, gp_clk_rst, hp_clk_rst;
  wire   [  1 : 0]  vendor_led_unused;
  wire   [  1 : 0]  alert_led;

  // =================================================================
  //  ps_ai_wrap_demo �? 整个PS/PL系统的黑盒wrapper�?.edf网表�?
  //  包含：PS7 ARM, MIG DDR3, AXI桥接, NPU, 帧处�?, HDMI
  //  cam_* 输入�?0 �? 我们不用摄像�?/视频通路，不影响User_Reg/User_Ddr
  // =================================================================
  ps_ai_wrap_demo U0_ps_ai_wrap_demo (
    // PS DDR
    .FIXED_IO_0_mio      ( FIXED_IO_0_mio      ),
    .DDR_0_addr          ( DDR_0_addr           ),
    .DDR_0_ba            ( DDR_0_ba             ),
    .DDR_0_cas_n         ( DDR_0_cas_n          ),
    .DDR_0_ck_n          ( DDR_0_ck_n           ),
    .DDR_0_ck_p          ( DDR_0_ck_p           ),
    .DDR_0_cke           ( DDR_0_cke            ),
    .DDR_0_cs_n          ( DDR_0_cs_n           ),
    .DDR_0_dm            ( DDR_0_dm             ),
    .DDR_0_dq            ( DDR_0_dq             ),
    .DDR_0_dqs_n         ( DDR_0_dqs_n          ),
    .DDR_0_dqs_p         ( DDR_0_dqs_p          ),
    .DDR_0_odt           ( DDR_0_odt            ),
    .DDR_0_ras_n         ( DDR_0_ras_n          ),
    .DDR_0_reset_n       ( DDR_0_reset_n        ),
    .DDR_0_we_n          ( DDR_0_we_n           ),
    .FIXED_IO_0_ddr_vrn  ( FIXED_IO_0_ddr_vrn   ),
    .FIXED_IO_0_ddr_vrp  ( FIXED_IO_0_ddr_vrp   ),
    .FIXED_IO_0_ps_clk   ( FIXED_IO_0_ps_clk    ),
    .FIXED_IO_0_ps_porb  ( FIXED_IO_0_ps_porb   ),
    .FIXED_IO_0_ps_srstb ( FIXED_IO_0_ps_srstb  ),
    // PL DDR3
    .ddr3_dq             ( ddr3_dq              ),
    .ddr3_dqs_n          ( ddr3_dqs_n           ),
    .ddr3_dqs_p          ( ddr3_dqs_p           ),
    .ddr3_dm             ( ddr3_dm              ),
    .ddr3_addr           ( ddr3_addr            ),
    .ddr3_ba             ( ddr3_ba              ),
    .ddr3_ras_n          ( ddr3_ras_n           ),
    .ddr3_cas_n          ( ddr3_cas_n           ),
    .ddr3_we_n           ( ddr3_we_n            ),
    .ddr3_reset_n        ( ddr3_reset_n         ),
    .ddr3_ck_p           ( ddr3_ck_p            ),
    .ddr3_ck_n           ( ddr3_ck_n            ),
    .ddr3_cke            ( ddr3_cke             ),
    .ddr3_cs_n           ( ddr3_cs_n            ),
    .ddr3_odt            ( ddr3_odt             ),
    // System
    .sys_rst             ( sys_rst              ),
    .sys_clk_p           ( sys_clk_p            ),
    .sys_clk_n           ( sys_clk_n            ),
    // Camera �? 全部�?0，Part2不使�?
    .cam_vs              ( 1'b0                 ),
    .cam_rgb             ( 24'd0                ),
    .cam_clk             ( gp_clk                 ),
    .cam_data_en         ( 1'b0                 ),
    // HDMI �? ps_ai_wrap_demo的输出口，保�?
    .HDMI_CLK_P          ( HDMI_CLK_P           ),
    .HDMI_CLK_N          ( HDMI_CLK_N           ),
    .HDMI_TX_P           ( HDMI_TX_P            ),
    .HDMI_TX_N           ( HDMI_TX_N            ),
    // LED / PJTAG
    .led                 ( vendor_led_unused    ),
    .led3                ( 1'b0                 ),
    .PJTAG_TCK           ( PJTAG_TCK            ),
    .PJTAG_TMS           ( PJTAG_TMS            ),
    .PJTAG_TDI           ( PJTAG_TDI            ),
    .PJTAG_TDO           ( PJTAG_TDO            ),
    // Clocks
    .gp_clk              ( gp_clk               ),
    .hp_clk              ( hp_clk               ),
    .gp_clk_rst          ( gp_clk_rst           ),
    .hp_clk_rst          ( hp_clk_rst           ),
    // Register bus �? bev_accel_top
    .User_Reg_awwaddr    ( User_Reg_awwaddr     ),
    .User_Reg_awwdata    ( User_Reg_awwdata     ),
    .User_Reg_awwvalid   ( User_Reg_awwvalid    ),
    .User_Reg_awwready   ( User_Reg_awwready    ),
    .User_Reg_araddr     ( User_Reg_araddr      ),
    .User_Reg_arvalid    ( User_Reg_arvalid     ),
    .User_Reg_arready    ( User_Reg_arready     ),
    .User_Reg_rdata      ( User_Reg_rdata       ),
    .User_Reg_rvalid     ( User_Reg_rvalid      ),
    .User_Reg_rready     ( User_Reg_rready      ),
    // DDR data bus (512-bit) �? bev_accel_top
    .User_Ddr_awwaddr    ( User_Ddr_awwaddr     ),
    .User_Ddr_awwdata    ( User_Ddr_awwdata     ),
    .User_Ddr_awwinfo    ( User_Ddr_awwinfo     ),
    .User_Ddr_awwvalid   ( User_Ddr_awwvalid    ),
    .User_Ddr_awwready   ( User_Ddr_awwready    ),
    .User_Ddr_araddr     ( User_Ddr_araddr      ),
    .User_Ddr_arvalid    ( User_Ddr_arvalid     ),
    .User_Ddr_arinfo     ( User_Ddr_arinfo      ),
    .User_Ddr_arready    ( User_Ddr_arready     ),
    .User_Ddr_rdata      ( User_Ddr_rdata       ),
    .User_Ddr_rvalid     ( User_Ddr_rvalid      ),
    .User_Ddr_rinfo      ( User_Ddr_rinfo       ),
    .User_Ddr_rready     ( User_Ddr_rready      )
  );

  // =================================================================
  //  rx_sdi 已删�? �? Part2不需要SDI摄像头接收器
  //  因此也不�?�? sdi/ 文件夹和 mgtclk/smpte_sdi/sys_clk IP�?
  //  约束文件中引�? rx_sdi_inst 的行会产�? Warning，不影响综合实现
  // =================================================================

  // ===================== DDR Info =====================
  // 写：64字节全使�? + awlast=1（单拍传输）
  // 读：arlast=1
  assign User_Ddr_awwinfo = {1'b0, {64{1'b1}}, 6'b0, 1'b1};
  assign User_Ddr_arinfo  = {25'h0, 6'b0, 1'b1};

  // ===================== BEV Accelerator（替换demo的adder_top�?=====================
  bev_accel_top U_bev_accel_top(
    .itf_ra_awaddr  ( User_Reg_awwaddr  ),
    .itf_ra_awdata  ( User_Reg_awwdata  ),
    .itf_ra_awvalid ( User_Reg_awwvalid ),
    .itf_ra_awready ( User_Reg_awwready ),
    .itf_ra_araddr  ( User_Reg_araddr   ),
    .itf_ra_arvalid ( User_Reg_arvalid  ),
    .itf_ra_arready ( User_Reg_arready  ),
    .itf_ra_rdata   ( User_Reg_rdata    ),
    .itf_ra_rvalid  ( User_Reg_rvalid   ),
    .itf_ra_rready  ( User_Reg_rready   ),
    .itf_awaddr     ( User_Ddr_awwaddr  ),
    .itf_awdata     ( User_Ddr_awwdata  ),
    .itf_awvalid    ( User_Ddr_awwvalid ),
    .itf_awready    ( User_Ddr_awwready ),
    .itf_araddr     ( User_Ddr_araddr   ),
    .itf_arvalid    ( User_Ddr_arvalid  ),
    .itf_arready    ( User_Ddr_arready  ),
    .itf_rdata      ( User_Ddr_rdata    ),
    .itf_rvalid     ( User_Ddr_rvalid   ),
    .itf_rready     ( User_Ddr_rready   ),
    .reset_reg      (                   ),
    .alert_led      ( alert_led         ),
    .clk            ( hp_clk            ),
    .ra_clk         ( gp_clk            ),
    .rst_n          ( ~hp_clk_rst       ),
    .ra_rst_n       ( ~gp_clk_rst       )
  );

  // The vendor EDIF LED output is intentionally disconnected from the board
  // pins. The alert controller is now the sole driver of J1/M6 via led[1:0].
  assign led = alert_led;

endmodule
