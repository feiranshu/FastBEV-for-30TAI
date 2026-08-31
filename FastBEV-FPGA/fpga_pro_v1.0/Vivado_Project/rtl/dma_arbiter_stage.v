//==============================================================================
// File Name     : dma_arbiter_stage.v
// Project       : Fast-BEV Part2 FPGA Accelerator
// Description   : Stage-exclusive DMA arbiter for Quant/LUT/SA engines.
//
//   active_stage:
//     2'b00 = none
//     2'b01 = quant
//     2'b10 = lut
//     2'b11 = sa
//
//   This Phase E baseline intentionally does not overlap engines. The selected
//   stage owns DDR read response routing until the controller changes stage.
//==============================================================================

`timescale 1ns / 1ps

module dma_arbiter_stage(
    input         [    1 : 0]   active_stage       ,

    // === Quant Engine Read Interface ===
    input         [   31 : 0]   quant_rd_addr      ,
    input                       quant_rd_req       ,
    output                      quant_rd_grant     ,
    output        [  511 : 0]   quant_rd_data      ,
    output                      quant_rd_data_valid,
    input                       quant_rd_data_ready,

    // === Quant Engine Write Interface ===
    input         [   31 : 0]   quant_wr_addr      ,
    input         [  511 : 0]   quant_wr_data      ,
    input                       quant_wr_req       ,
    output                      quant_wr_grant     ,

    // === LUT Engine Read Interface ===
    input         [   31 : 0]   lut_rd_addr        ,
    input                       lut_rd_req         ,
    output                      lut_rd_grant       ,
    output        [  511 : 0]   lut_rd_data        ,
    output                      lut_rd_data_valid  ,
    input                       lut_rd_data_ready  ,

    // === LUT Engine Write Interface ===
    input         [   31 : 0]   lut_wr_addr        ,
    input         [  511 : 0]   lut_wr_data        ,
    input                       lut_wr_req         ,
    output                      lut_wr_grant       ,

    // === SA Engine Read Interface ===
    input         [   31 : 0]   sa_rd_addr         ,
    input                       sa_rd_req          ,
    output                      sa_rd_grant        ,
    output        [  511 : 0]   sa_rd_data         ,
    output                      sa_rd_data_valid   ,
    input                       sa_rd_data_ready   ,

    // === SA Engine Write Interface ===
    input         [   31 : 0]   sa_wr_addr         ,
    input         [  511 : 0]   sa_wr_data         ,
    input                       sa_wr_req          ,
    output                      sa_wr_grant        ,

    // === PLDDR Read Interface ===
    output reg    [   31 : 0]   ddr_araddr         ,
    output reg                  ddr_arvalid        ,
    input                       ddr_arready        ,
    input         [  511 : 0]   ddr_rdata          ,
    input                       ddr_rvalid         ,
    output                      ddr_rready         ,

    // === PLDDR Write Interface ===
    output reg    [   31 : 0]   ddr_awaddr         ,
    output reg    [  511 : 0]   ddr_awdata         ,
    output reg                  ddr_awvalid        ,
    input                       ddr_awready        ,

    input                       clk                ,
    input                       rst_n
);

    localparam STAGE_NONE  = 2'b00;
    localparam STAGE_QUANT = 2'b01;
    localparam STAGE_LUT   = 2'b10;
    localparam STAGE_SA    = 2'b11;

    wire sel_quant = (active_stage == STAGE_QUANT);
    wire sel_lut   = (active_stage == STAGE_LUT);
    wire sel_sa    = (active_stage == STAGE_SA);

    always @(*) begin
        if(sel_quant && quant_rd_req) begin
            ddr_araddr  = quant_rd_addr;
            ddr_arvalid = 1'b1;
        end else if(sel_lut && lut_rd_req) begin
            ddr_araddr  = lut_rd_addr;
            ddr_arvalid = 1'b1;
        end else if(sel_sa && sa_rd_req) begin
            ddr_araddr  = sa_rd_addr;
            ddr_arvalid = 1'b1;
        end else begin
            ddr_araddr  = 32'd0;
            ddr_arvalid = 1'b0;
        end
    end

    assign quant_rd_grant = sel_quant && quant_rd_req && ddr_arready;
    assign lut_rd_grant   = sel_lut   && lut_rd_req   && ddr_arready;
    assign sa_rd_grant    = sel_sa    && sa_rd_req    && ddr_arready;

    assign quant_rd_data       = ddr_rdata;
    assign lut_rd_data         = ddr_rdata;
    assign sa_rd_data          = ddr_rdata;
    assign quant_rd_data_valid = sel_quant && ddr_rvalid;
    assign lut_rd_data_valid   = sel_lut   && ddr_rvalid;
    assign sa_rd_data_valid    = sel_sa    && ddr_rvalid;

    assign ddr_rready = (sel_quant && quant_rd_data_ready) ||
                        (sel_lut   && lut_rd_data_ready)   ||
                        (sel_sa    && sa_rd_data_ready);

    always @(*) begin
        if(sel_quant && quant_wr_req) begin
            ddr_awaddr  = quant_wr_addr;
            ddr_awdata  = quant_wr_data;
            ddr_awvalid = 1'b1;
        end else if(sel_lut && lut_wr_req) begin
            ddr_awaddr  = lut_wr_addr;
            ddr_awdata  = lut_wr_data;
            ddr_awvalid = 1'b1;
        end else if(sel_sa && sa_wr_req) begin
            ddr_awaddr  = sa_wr_addr;
            ddr_awdata  = sa_wr_data;
            ddr_awvalid = 1'b1;
        end else begin
            ddr_awaddr  = 32'd0;
            ddr_awdata  = 512'd0;
            ddr_awvalid = 1'b0;
        end
    end

    assign quant_wr_grant = sel_quant && quant_wr_req && ddr_awready;
    assign lut_wr_grant   = sel_lut   && lut_wr_req   && ddr_awready;
    assign sa_wr_grant    = sel_sa    && sa_wr_req    && ddr_awready;

    wire unused_clk_rst;
    assign unused_clk_rst = clk | rst_n | (active_stage == STAGE_NONE);

endmodule
