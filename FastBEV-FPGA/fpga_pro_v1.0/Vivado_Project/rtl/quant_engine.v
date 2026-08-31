//==============================================================================
// File Name     : quant_engine.v
// Project       : Fast-BEV Part2 FPGA Accelerator
// Description   : FP32 2D feature to INT8 2D feature pack engine.
//
//   One 2D pixel is 64 FP32 channels in NHWC layout:
//     input  pixel = 64 * 4B = 256B = 4 DDR beats
//     output pixel = 64 * 1B =  64B = 1 DDR beat
//
//   This Phase A baseline keeps one read/quant operation outstanding at a time.
//   It is intentionally simple and deterministic for first bring-up; later
//   phases may overlap reads and quant pipeline flushes once the layout is
//   proven.
//==============================================================================
`timescale 1ns/1ps

module quant_engine #(
    parameter integer SHIFT_BASE = 156
)(
    input                       engine_start     ,
    output reg                  engine_done      ,

    input         [31 : 0]   fp32_base_addr   ,
    input         [31 : 0]   int8_base_addr   ,
    input         [31 : 0]   total_pixels     ,

    output reg    [31 : 0]   rd_addr          ,
    output reg                  rd_req           ,
    input                       rd_grant         ,
    input         [511 : 0]   rd_data          ,
    input                       rd_data_valid    ,
    output                      rd_data_ready    ,

    output reg    [31 : 0]   wr_addr          ,
    output reg    [511 : 0]   wr_data          ,
    output reg                  wr_req           ,
    input                       wr_grant         ,

    input                       clk              ,
    input                       rst_n
);

    localparam S_IDLE      = 3'd0;
    localparam S_ISSUE_RD  = 3'd1;
    localparam S_WAIT_RD   = 3'd2;
    localparam S_WAIT_Q    = 3'd3;
    localparam S_ISSUE_WR  = 3'd4;
    localparam S_DONE      = 3'd5;

    reg [2:0] state;
    reg [31:0] pixel_idx;
    reg [1:0] beat_idx;
    reg [511:0] pack_buf;
    reg [511:0] pack_buf_next;

    wire quant_fire;
    assign quant_fire = (state == S_WAIT_RD) && rd_data_valid && rd_data_ready;
    assign rd_data_ready = (state == S_WAIT_RD);

    wire [15:0] q_valid;
    wire [127:0] q_dst_bus;

    genvar lane;
    generate
        for (lane = 0; lane < 16; lane = lane + 1) begin : GEN_QUANT_LANE
            fp32_int8_quant #(
                .SHIFT_BASE(SHIFT_BASE)
            ) U_fp32_int8_quant (
                .clk       ( clk                       ),
                .rst_n     ( rst_n                     ),
                .in_valid  ( quant_fire                ),
                .src       ( rd_data[lane*32 +: 32]    ),
                .out_valid ( q_valid[lane]             ),
                .dst       ( q_dst_bus[lane*8 +: 8]    )
            );
        end
    endgenerate

    integer i;
    wire all_q_valid;
    assign all_q_valid = &q_valid;

    wire [31:0] rd_addr_next;
    wire [31:0] wr_addr_next;
    assign rd_addr_next = fp32_base_addr + (pixel_idx << 8) + ({30'd0, beat_idx} << 6);
    assign wr_addr_next = int8_base_addr + (pixel_idx << 6);

    always @(*) begin
        pack_buf_next = pack_buf;
        for (i = 0; i < 16; i = i + 1)
            pack_buf_next[(beat_idx*16 + i)*8 +: 8] = q_dst_bus[i*8 +: 8];
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            engine_done <= 1'b0;
            rd_addr     <= 32'd0;
            rd_req      <= 1'b0;
            wr_addr     <= 32'd0;
            wr_data     <= 512'd0;
            wr_req      <= 1'b0;
            pixel_idx   <= 32'd0;
            beat_idx    <= 2'd0;
            pack_buf    <= 512'd0;
        end else begin
            engine_done <= 1'b0;

            case (state)
                S_IDLE: begin
                    rd_req    <= 1'b0;
                    wr_req    <= 1'b0;
                    pixel_idx <= 32'd0;
                    beat_idx  <= 2'd0;
                    pack_buf  <= 512'd0;
                    if (engine_start) begin
                        if (total_pixels == 32'd0)
                            state <= S_DONE;
                        else
                            state <= S_ISSUE_RD;
                    end
                end

                S_ISSUE_RD: begin
                    rd_addr <= rd_addr_next;
                    rd_req  <= 1'b1;
                    if (rd_grant) begin
                        rd_req <= 1'b0;
                        state  <= S_WAIT_RD;
                    end
                end

                S_WAIT_RD: begin
                    if (quant_fire)
                        state <= S_WAIT_Q;
                end

                S_WAIT_Q: begin
                    if (all_q_valid) begin
                        if (beat_idx == 2'd3) begin
                            pack_buf <= pack_buf_next;
                            wr_data  <= pack_buf_next;
                            state <= S_ISSUE_WR;
                        end else begin
                            pack_buf <= pack_buf_next;
                            beat_idx <= beat_idx + 1'b1;
                            state    <= S_ISSUE_RD;
                        end
                    end
                end

                S_ISSUE_WR: begin
                    wr_addr <= wr_addr_next;
                    wr_req  <= 1'b1;
                    if (wr_grant) begin
                        wr_req <= 1'b0;
                        if (pixel_idx + 1'b1 >= total_pixels) begin
                            state <= S_DONE;
                        end else begin
                            pixel_idx <= pixel_idx + 1'b1;
                            beat_idx  <= 2'd0;
                            pack_buf  <= 512'd0;
                            state     <= S_ISSUE_RD;
                        end
                    end
                end

                S_DONE: begin
                    engine_done <= 1'b1;
                    state       <= S_IDLE;
                end

                default: begin
                    state  <= S_IDLE;
                    rd_req <= 1'b0;
                    wr_req <= 1'b0;
                end
            endcase
        end
    end

endmodule
