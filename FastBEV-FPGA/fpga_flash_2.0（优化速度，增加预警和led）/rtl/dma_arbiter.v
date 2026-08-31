//==============================================================================
// dma_arbiter.v
//------------------------------------------------------------------------------
// Single active-engine DDR mux. The current int8 Part2 package uses only the LUT
// engine path, but the SA path is kept at the interface for top-level stability.
//==============================================================================
`timescale 1ns/1ps

module dma_arbiter(
    input         [1:0]   active_engine,

    input        [31:0]   lut_rd_addr,
    input                 lut_rd_req,
    output                lut_rd_grant,
    output       [511:0]  lut_rd_data,
    output                lut_rd_data_valid,
    input                 lut_rd_data_ready,

    input        [31:0]   lut_wr_addr,
    input        [511:0]  lut_wr_data,
    input                 lut_wr_req,
    output                lut_wr_grant,

    input        [31:0]   sa_rd_addr,
    input                 sa_rd_req,
    output                sa_rd_grant,
    output       [511:0]  sa_rd_data,
    output                sa_rd_data_valid,
    input                 sa_rd_data_ready,

    input        [31:0]   sa_wr_addr,
    input        [511:0]  sa_wr_data,
    input                 sa_wr_req,
    output                sa_wr_grant,

    output reg   [31:0]   ddr_araddr,
    output reg            ddr_arvalid,
    input                 ddr_arready,
    input        [511:0]  ddr_rdata,
    input                 ddr_rvalid,
    output                ddr_rready,

    output reg   [31:0]   ddr_awaddr,
    output reg   [511:0]  ddr_awdata,
    output reg            ddr_awvalid,
    input                 ddr_awready,

    input                 clk,
    input                 rst_n
);

    wire sel_lut = (active_engine == 2'b01);
    wire sel_sa  = (active_engine == 2'b10);

    always @(*) begin
        if (sel_lut && lut_rd_req) begin
            ddr_araddr  = lut_rd_addr;
            ddr_arvalid = 1'b1;
        end else if (sel_sa && sa_rd_req) begin
            ddr_araddr  = sa_rd_addr;
            ddr_arvalid = 1'b1;
        end else begin
            ddr_araddr  = 32'd0;
            ddr_arvalid = 1'b0;
        end
    end

    assign lut_rd_grant = sel_lut && lut_rd_req && ddr_arready;
    assign sa_rd_grant  = sel_sa  && sa_rd_req  && ddr_arready;
    assign lut_rd_data  = ddr_rdata;
    assign sa_rd_data   = ddr_rdata;
    assign lut_rd_data_valid = sel_lut && ddr_rvalid;
    assign sa_rd_data_valid  = sel_sa  && ddr_rvalid;
    assign ddr_rready = (sel_lut && lut_rd_data_ready) ||
                        (sel_sa  && sa_rd_data_ready);

    always @(*) begin
        if (sel_lut && lut_wr_req) begin
            ddr_awaddr  = lut_wr_addr;
            ddr_awdata  = lut_wr_data;
            ddr_awvalid = 1'b1;
        end else if (sel_sa && sa_wr_req) begin
            ddr_awaddr  = sa_wr_addr;
            ddr_awdata  = sa_wr_data;
            ddr_awvalid = 1'b1;
        end else begin
            ddr_awaddr  = 32'd0;
            ddr_awdata  = 512'd0;
            ddr_awvalid = 1'b0;
        end
    end

    assign lut_wr_grant = sel_lut && lut_wr_req && ddr_awready;
    assign sa_wr_grant  = sel_sa  && sa_wr_req  && ddr_awready;

endmodule

