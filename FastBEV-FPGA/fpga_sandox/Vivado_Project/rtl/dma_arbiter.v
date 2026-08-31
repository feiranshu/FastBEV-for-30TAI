`timescale 1ns/1ps
//==============================================================================
// File Name     : dma_arbiter.v
// Project       : Fast-BEV Part2 FPGA Accelerator
// Description   : DMA Arbiter for shared PLDDR bus access
//   - Multiplexes read/write requests from LUT engine and SA engine
//   - Interfaces with User_Ddr port (512-bit data, ready/valid handshake)
//   - Priority-based: active engine gets exclusive access
//   - Only one engine is active at a time (controlled by comp_mode)
//==============================================================================

module dma_arbiter(

    // --- Engine selection (from top control) ---
    input         [    1 : 0]   active_engine    ,  // 01=LUT, 10=SA

    // === LUT Engine Read Interface ===
    input         [   31 : 0]   lut_rd_addr      ,
    input                       lut_rd_req       ,
    output                      lut_rd_grant     ,
    output        [  511 : 0]   lut_rd_data      ,
    output                      lut_rd_data_valid,
    input                       lut_rd_data_ready,

    // === LUT Engine Write Interface ===
    input         [   31 : 0]   lut_wr_addr      ,
    input         [  511 : 0]   lut_wr_data      ,
    input                       lut_wr_req       ,
    output                      lut_wr_grant     ,

    // === SA Engine Read Interface ===
    input         [   31 : 0]   sa_rd_addr       ,
    input                       sa_rd_req        ,
    output                      sa_rd_grant      ,
    output        [  511 : 0]   sa_rd_data       ,
    output                      sa_rd_data_valid ,
    input                       sa_rd_data_ready ,

    // === SA Engine Write Interface ===
    input         [   31 : 0]   sa_wr_addr       ,
    input         [  511 : 0]   sa_wr_data       ,
    input                       sa_wr_req        ,
    output                      sa_wr_grant      ,

    // === PLDDR Read Interface (to top-level User_Ddr) ===
    output reg    [   31 : 0]   ddr_araddr       ,
    output reg                  ddr_arvalid      ,
    input                       ddr_arready      ,
    input         [  511 : 0]   ddr_rdata        ,
    input                       ddr_rvalid       ,
    output                      ddr_rready       ,

    // === PLDDR Write Interface (to top-level User_Ddr) ===
    output reg    [   31 : 0]   ddr_awaddr       ,
    output reg    [  511 : 0]   ddr_awdata       ,
    output reg                  ddr_awvalid      ,
    input                       ddr_awready      ,

    input                       clk              ,
    input                       rst_n
);

    // ===================== Selection Logic =====================
    wire sel_lut = (active_engine == 2'b01);
    wire sel_sa  = (active_engine == 2'b10);

    // ===================== Read Channel Mux =====================
    
    // Read address channel
    always @(*) begin
        if (sel_lut && lut_rd_req) begin
            ddr_araddr  = lut_rd_addr;
            ddr_arvalid = 1'b1;
        end
        else if (sel_sa && sa_rd_req) begin
            ddr_araddr  = sa_rd_addr;
            ddr_arvalid = 1'b1;
        end
        else begin
            ddr_araddr  = 32'd0;
            ddr_arvalid = 1'b0;
        end
    end

    // Read grant back to engines
    assign lut_rd_grant = sel_lut && lut_rd_req && ddr_arready;
    assign sa_rd_grant  = sel_sa  && sa_rd_req  && ddr_arready;

    // Read data channel - broadcast to both, valid gated by selection
    assign lut_rd_data       = ddr_rdata;
    assign lut_rd_data_valid = sel_lut && ddr_rvalid;
    assign sa_rd_data        = ddr_rdata;
    assign sa_rd_data_valid  = sel_sa  && ddr_rvalid;

    // Read data ready - from active engine
    assign ddr_rready = (sel_lut && lut_rd_data_ready) ||
                        (sel_sa  && sa_rd_data_ready);

    // ===================== Write Channel Mux =====================
    
    always @(*) begin
        if (sel_lut && lut_wr_req) begin
            ddr_awaddr  = lut_wr_addr;
            ddr_awdata  = lut_wr_data;
            ddr_awvalid = 1'b1;
        end
        else if (sel_sa && sa_wr_req) begin
            ddr_awaddr  = sa_wr_addr;
            ddr_awdata  = sa_wr_data;
            ddr_awvalid = 1'b1;
        end
        else begin
            ddr_awaddr  = 32'd0;
            ddr_awdata  = 512'd0;
            ddr_awvalid = 1'b0;
        end
    end

    // Write grant back to engines
    assign lut_wr_grant = sel_lut && lut_wr_req && ddr_awready;
    assign sa_wr_grant  = sel_sa  && sa_wr_req  && ddr_awready;

endmodule
