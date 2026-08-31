// Copyright 1986-2018 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2018.3 (win64) Build 2405991 Thu Dec  6 23:38:27 MST 2018
// Date        : Mon Mar 23 13:49:31 2026
// Host        : DESKTOP-7CRHBA3 running 64-bit major release  (build 9200)
// Command     : write_verilog -mode synth_stub ps_ai_wrap_demo.v
// Design      : ps_ai_wrap_demo
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7z030ffg676-2
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
module ps_ai_wrap_demo(FIXED_IO_0_mio, DDR_0_addr, DDR_0_ba, 
  DDR_0_cas_n, DDR_0_ck_n, DDR_0_ck_p, DDR_0_cke, DDR_0_cs_n, DDR_0_dm, DDR_0_dq, DDR_0_dqs_n, 
  DDR_0_dqs_p, DDR_0_odt, DDR_0_ras_n, DDR_0_reset_n, DDR_0_we_n, FIXED_IO_0_ddr_vrn, 
  FIXED_IO_0_ddr_vrp, FIXED_IO_0_ps_clk, FIXED_IO_0_ps_porb, FIXED_IO_0_ps_srstb, ddr3_dq, 
  ddr3_dqs_n, ddr3_dqs_p, ddr3_dm, ddr3_addr, ddr3_ba, ddr3_ras_n, ddr3_cas_n, ddr3_we_n, 
  ddr3_reset_n, ddr3_ck_p, ddr3_ck_n, ddr3_cke, ddr3_cs_n, ddr3_odt, sys_rst, sys_clk_p, sys_clk_n, 
  cam_vs, cam_rgb, cam_clk, cam_data_en, HDMI_CLK_P, HDMI_CLK_N, HDMI_TX_P, HDMI_TX_N, led, led3, 
  PJTAG_TCK, PJTAG_TMS, PJTAG_TDI, PJTAG_TDO, gp_clk, hp_clk, gp_clk_rst, hp_clk_rst, 
  User_Reg_awwaddr, User_Reg_awwdata, User_Reg_awwvalid, User_Reg_awwready, 
  User_Reg_araddr, User_Reg_arvalid, User_Reg_arready, User_Reg_rdata, User_Reg_rvalid, 
  User_Reg_rready, User_Ddr_awwaddr, User_Ddr_awwdata, User_Ddr_awwinfo, User_Ddr_awwvalid, 
  User_Ddr_awwready, User_Ddr_araddr, User_Ddr_arvalid, User_Ddr_arinfo, User_Ddr_arready, 
  User_Ddr_rdata, User_Ddr_rvalid, User_Ddr_rinfo, User_Ddr_rready)
/* synthesis syn_black_box black_box_pad_pin="FIXED_IO_0_mio[53:0],DDR_0_addr[14:0],DDR_0_ba[2:0],DDR_0_cas_n,DDR_0_ck_n,DDR_0_ck_p,DDR_0_cke,DDR_0_cs_n,DDR_0_dm[3:0],DDR_0_dq[31:0],DDR_0_dqs_n[3:0],DDR_0_dqs_p[3:0],DDR_0_odt,DDR_0_ras_n,DDR_0_reset_n,DDR_0_we_n,FIXED_IO_0_ddr_vrn,FIXED_IO_0_ddr_vrp,FIXED_IO_0_ps_clk,FIXED_IO_0_ps_porb,FIXED_IO_0_ps_srstb,ddr3_dq[63:0],ddr3_dqs_n[7:0],ddr3_dqs_p[7:0],ddr3_dm[7:0],ddr3_addr[14:0],ddr3_ba[2:0],ddr3_ras_n,ddr3_cas_n,ddr3_we_n,ddr3_reset_n,ddr3_ck_p[0:0],ddr3_ck_n[0:0],ddr3_cke[0:0],ddr3_cs_n[0:0],ddr3_odt[0:0],sys_rst,sys_clk_p,sys_clk_n,cam_vs,cam_rgb[23:0],cam_clk,cam_data_en,HDMI_CLK_P,HDMI_CLK_N,HDMI_TX_P[2:0],HDMI_TX_N[2:0],led[1:0],led3,PJTAG_TCK,PJTAG_TMS,PJTAG_TDI,PJTAG_TDO,gp_clk,hp_clk,gp_clk_rst,hp_clk_rst,User_Reg_awwaddr[31:0],User_Reg_awwdata[31:0],User_Reg_awwvalid,User_Reg_awwready,User_Reg_araddr[31:0],User_Reg_arvalid,User_Reg_arready,User_Reg_rdata[31:0],User_Reg_rvalid,User_Reg_rready,User_Ddr_awwaddr[31:0],User_Ddr_awwdata[511:0],User_Ddr_awwinfo[71:0],User_Ddr_awwvalid,User_Ddr_awwready,User_Ddr_araddr[31:0],User_Ddr_arvalid,User_Ddr_arinfo[31:0],User_Ddr_arready,User_Ddr_rdata[511:0],User_Ddr_rvalid,User_Ddr_rinfo[31:0],User_Ddr_rready" */;
  inout [53:0]FIXED_IO_0_mio;
  inout [14:0]DDR_0_addr;
  inout [2:0]DDR_0_ba;
  inout DDR_0_cas_n;
  inout DDR_0_ck_n;
  inout DDR_0_ck_p;
  inout DDR_0_cke;
  inout DDR_0_cs_n;
  inout [3:0]DDR_0_dm;
  inout [31:0]DDR_0_dq;
  inout [3:0]DDR_0_dqs_n;
  inout [3:0]DDR_0_dqs_p;
  inout DDR_0_odt;
  inout DDR_0_ras_n;
  inout DDR_0_reset_n;
  inout DDR_0_we_n;
  inout FIXED_IO_0_ddr_vrn;
  inout FIXED_IO_0_ddr_vrp;
  inout FIXED_IO_0_ps_clk;
  inout FIXED_IO_0_ps_porb;
  inout FIXED_IO_0_ps_srstb;
  inout [63:0]ddr3_dq;
  inout [7:0]ddr3_dqs_n;
  inout [7:0]ddr3_dqs_p;
  output [7:0]ddr3_dm;
  output [14:0]ddr3_addr;
  output [2:0]ddr3_ba;
  output ddr3_ras_n;
  output ddr3_cas_n;
  output ddr3_we_n;
  output ddr3_reset_n;
  output [0:0]ddr3_ck_p;
  output [0:0]ddr3_ck_n;
  output [0:0]ddr3_cke;
  output [0:0]ddr3_cs_n;
  output [0:0]ddr3_odt;
  input sys_rst;
  input sys_clk_p;
  input sys_clk_n;
  input cam_vs;
  input [23:0]cam_rgb;
  input cam_clk;
  input cam_data_en;
  output HDMI_CLK_P;
  output HDMI_CLK_N;
  output [2:0]HDMI_TX_P;
  output [2:0]HDMI_TX_N;
  output [1:0]led;
  input led3;
  input PJTAG_TCK;
  input PJTAG_TMS;
  input PJTAG_TDI;
  output PJTAG_TDO;
  output gp_clk;
  output hp_clk;
  output gp_clk_rst;
  output hp_clk_rst;
  output [31:0]User_Reg_awwaddr;
  output [31:0]User_Reg_awwdata;
  output User_Reg_awwvalid;
  input User_Reg_awwready;
  output [31:0]User_Reg_araddr;
  output User_Reg_arvalid;
  input User_Reg_arready;
  input [31:0]User_Reg_rdata;
  input User_Reg_rvalid;
  output User_Reg_rready;
  input [31:0]User_Ddr_awwaddr;
  input [511:0]User_Ddr_awwdata;
  input [71:0]User_Ddr_awwinfo;
  input User_Ddr_awwvalid;
  output User_Ddr_awwready;
  input [31:0]User_Ddr_araddr;
  input User_Ddr_arvalid;
  input [31:0]User_Ddr_arinfo;
  output User_Ddr_arready;
  output [511:0]User_Ddr_rdata;
  output User_Ddr_rvalid;
  output [31:0]User_Ddr_rinfo;
  input User_Ddr_rready;
endmodule
