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

    // A two-entry registered read-request queue isolates engine request generation
    // from the external DDR ready path.  A single entry would insert a bubble
    // between requests; two entries retain one accepted request per cycle when
    // DDR is ready while also absorbing a short ready stall.
    reg [31:0] rd_head_addr;
    reg [31:0] rd_tail_addr;
    reg [1:0]  rd_fifo_count;

    reg [31:0] selected_rd_addr;
    reg        selected_rd_req;
    always @(*) begin
        selected_rd_addr = 32'd0;
        selected_rd_req  = 1'b0;
        if(sel_quant) begin
            selected_rd_addr = quant_rd_addr;
            selected_rd_req  = quant_rd_req;
        end else if(sel_lut) begin
            selected_rd_addr = lut_rd_addr;
            selected_rd_req  = lut_rd_req;
        end else if(sel_sa) begin
            selected_rd_addr = sa_rd_addr;
            selected_rd_req  = sa_rd_req;
        end
    end

    wire rd_fifo_ready = (rd_fifo_count != 2'd2);
    wire rd_fifo_push  = selected_rd_req && rd_fifo_ready;
    wire rd_fifo_pop   = (rd_fifo_count != 2'd0) && ddr_arready;

    always @(*) begin
        ddr_arvalid = (rd_fifo_count != 2'd0);
        // Address is don't-care while valid is low.  Driving the head register
        // directly avoids an output-wide zero-selection mux.
        ddr_araddr  = rd_head_addr;
    end

    // Internal grants retain ready semantics and are deliberately independent
    // of both the engine request and external DDR ready signals.
    assign quant_rd_grant = sel_quant && rd_fifo_ready;
    assign lut_rd_grant   = sel_lut   && rd_fifo_ready;
    assign sa_rd_grant    = sel_sa    && rd_fifo_ready;

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

    // Writes keep external-handshake grant semantics.  In particular, the
    // final SA engine_done/COMP_DONE cannot precede the last concat write
    // accepted by the CustomOp DDR interface.
    assign quant_wr_grant = sel_quant && ddr_awready;
    assign lut_wr_grant   = sel_lut   && ddr_awready;
    assign sa_wr_grant    = sel_sa    && ddr_awready;

    always @(posedge clk) begin
        if(!rst_n) begin
            rd_head_addr  <= 32'd0;
            rd_tail_addr  <= 32'd0;
            rd_fifo_count <= 2'd0;
        end else begin
            case({rd_fifo_push,rd_fifo_pop})
                2'b10: begin
                    if(rd_fifo_count==2'd0)
                        rd_head_addr <= selected_rd_addr;
                    else
                        rd_tail_addr <= selected_rd_addr;
                    rd_fifo_count <= rd_fifo_count + 1'b1;
                end
                2'b01: begin
                    if(rd_fifo_count==2'd2)
                        rd_head_addr <= rd_tail_addr;
                    rd_fifo_count <= rd_fifo_count - 1'b1;
                end
                2'b11: begin
                    // Only count==1 can push and pop together: directly
                    // replace the consumed head and keep occupancy at one.
                    rd_head_addr <= selected_rd_addr;
                end
                default: begin end
            endcase

        end
    end

endmodule
