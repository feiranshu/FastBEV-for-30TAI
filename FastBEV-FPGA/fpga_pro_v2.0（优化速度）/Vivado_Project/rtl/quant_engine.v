//==============================================================================
// FP32 NHWC to INT8 pack engine. Four ordered FP32 beats per pixel may be in
// flight together. Two pixel pack contexts hide INT8 write backpressure.
//==============================================================================
`timescale 1ns/1ps

module quant_engine #(
    parameter integer SHIFT_BASE = 156,
    parameter integer MAX_OUTSTANDING = 4
)(
    input engine_start, output reg engine_done,
    input [31:0] fp32_base_addr, input [31:0] int8_base_addr,
    input [31:0] total_pixels,
    output reg [31:0] rd_addr, output reg rd_req, input rd_grant,
    input [511:0] rd_data, input rd_data_valid, output rd_data_ready,
    output reg [31:0] wr_addr, output reg [511:0] wr_data,
    output reg wr_req, input wr_grant, input clk, input rst_n
);
    localparam [1:0] S_IDLE=0,S_RUN=1,S_DONE=2;
    localparam [2:0] MAX_OUTSTANDING_C =
        (MAX_OUTSTANDING<1)?3'd1:(MAX_OUTSTANDING>4)?3'd4:MAX_OUTSTANDING;

    reg [1:0] state;
    reg [31:0] issue_pixel,completed_pixels;
    reg [1:0] issue_beat;
    reg issue_slot,write_slot;
    reg [1:0] slot_active,slot_full;
    reg [31:0] slot_pixel0,slot_pixel1;
    reg [511:0] pack_buf0,pack_buf1;

    // Accepted DDR reads are tagged with {pack slot, FP32 beat}.
    reg [2:0] req_tag_fifo[0:3];
    reg [1:0] req_wr_ptr,req_rd_ptr;
    reg [2:0] pending_count;

    // The lane quantizer has fixed latency. A deeper ordered tag FIFO allows
    // consecutive returns without coupling the engine to that latency value.
    reg [2:0] quant_tag_fifo[0:15];
    reg [3:0] q_wr_ptr,q_rd_ptr;
    reg [4:0] q_count;

    wire issue_slot_free=!slot_active[issue_slot];
    wire can_issue=(state==S_RUN)&&(issue_pixel<total_pixels)&&
                   ((issue_beat!=0)||issue_slot_free)&&
                   (pending_count<MAX_OUTSTANDING_C);
    wire read_issue_fire=can_issue&&rd_grant;
    assign rd_data_ready=(state==S_RUN)&&(pending_count!=0);
    wire quant_fire=rd_data_valid&&rd_data_ready;

    wire [15:0] q_valid;
    wire [127:0] q_dst_bus;
    genvar lane;
    generate for(lane=0;lane<16;lane=lane+1) begin:GEN_QUANT_LANE
        fp32_int8_quant #(.SHIFT_BASE(SHIFT_BASE)) U_fp32_int8_quant(
            .clk(clk),.rst_n(rst_n),.in_valid(quant_fire),
            .src(rd_data[lane*32+:32]),.out_valid(q_valid[lane]),
            .dst(q_dst_bus[lane*8+:8]));
    end endgenerate

    wire all_q_valid=&q_valid;
    wire q_output_fire=all_q_valid&&(q_count!=0);
    wire [2:0] output_tag=quant_tag_fifo[q_rd_ptr];
    wire output_slot=output_tag[2];
    wire [1:0] output_beat=output_tag[1:0];

    reg [511:0] pack0_with_q,pack1_with_q;
    integer qi;
    always @(*) begin
        pack0_with_q=pack_buf0;pack1_with_q=pack_buf1;
        for(qi=0;qi<16;qi=qi+1) begin
            if(output_slot)
                pack1_with_q[(output_beat*128)+(qi*8)+:8]=q_dst_bus[qi*8+:8];
            else
                pack0_with_q[(output_beat*128)+(qi*8)+:8]=q_dst_bus[qi*8+:8];
        end
    end

    always @(*) begin
        rd_addr=0;rd_req=0;wr_addr=0;wr_data=0;wr_req=0;
        if(can_issue) begin
            rd_addr=fp32_base_addr+(issue_pixel<<8)+({30'd0,issue_beat}<<6);
            rd_req=1;
        end
        if(state==S_RUN&&slot_full[write_slot]) begin
            wr_addr=int8_base_addr+((write_slot?slot_pixel1:slot_pixel0)<<6);
            wr_data=write_slot?pack_buf1:pack_buf0;
            wr_req=1;
        end
    end

    // Internal statistics remain simulation-visible without new PS registers.
    reg [63:0] perf_cycles,perf_read_requests,perf_write_requests;
    integer ri;
    always @(posedge clk) begin
        if(!rst_n) begin
            state<=S_IDLE;engine_done<=0;issue_pixel<=0;completed_pixels<=0;
            issue_beat<=0;issue_slot<=0;write_slot<=0;slot_active<=0;
            slot_full<=0;slot_pixel0<=0;slot_pixel1<=0;
            pack_buf0<=0;pack_buf1<=0;req_wr_ptr<=0;req_rd_ptr<=0;
            pending_count<=0;q_wr_ptr<=0;q_rd_ptr<=0;q_count<=0;
            perf_cycles<=0;perf_read_requests<=0;perf_write_requests<=0;
            for(ri=0;ri<4;ri=ri+1) req_tag_fifo[ri]<=0;
            for(ri=0;ri<16;ri=ri+1) quant_tag_fifo[ri]<=0;
        end else begin
            engine_done<=0;
            if(state!=S_IDLE&&state!=S_DONE) perf_cycles<=perf_cycles+1'b1;
            if(read_issue_fire) perf_read_requests<=perf_read_requests+1'b1;
            if(wr_req&&wr_grant) perf_write_requests<=perf_write_requests+1'b1;

            if(read_issue_fire) begin
                req_tag_fifo[req_wr_ptr]<={issue_slot,issue_beat};
                req_wr_ptr<=req_wr_ptr+1'b1;
                if(issue_beat==0) begin
                    slot_active[issue_slot]<=1'b1;
                    if(issue_slot) begin slot_pixel1<=issue_pixel;pack_buf1<=0;end
                    else begin slot_pixel0<=issue_pixel;pack_buf0<=0;end
                end
                if(issue_beat==3) begin
                    issue_beat<=0;issue_pixel<=issue_pixel+1'b1;
                    issue_slot<=~issue_slot;
                end else issue_beat<=issue_beat+1'b1;
            end

            if(quant_fire) begin
                quant_tag_fifo[q_wr_ptr]<=req_tag_fifo[req_rd_ptr];
                q_wr_ptr<=q_wr_ptr+1'b1;req_rd_ptr<=req_rd_ptr+1'b1;
            end
            if(q_output_fire) begin
                q_rd_ptr<=q_rd_ptr+1'b1;
                if(output_slot) pack_buf1<=pack1_with_q;
                else pack_buf0<=pack0_with_q;
                if(output_beat==3) slot_full[output_slot]<=1'b1;
            end

            case({read_issue_fire,quant_fire})
                2'b10:pending_count<=pending_count+1'b1;
                2'b01:pending_count<=pending_count-1'b1;
                default:begin end
            endcase
            case({quant_fire,q_output_fire})
                2'b10:q_count<=q_count+1'b1;
                2'b01:q_count<=q_count-1'b1;
                default:begin end
            endcase

            case(state)
                S_IDLE:if(engine_start) begin
                    issue_pixel<=0;completed_pixels<=0;issue_beat<=0;
                    issue_slot<=0;write_slot<=0;slot_active<=0;slot_full<=0;
                    pack_buf0<=0;pack_buf1<=0;req_wr_ptr<=0;req_rd_ptr<=0;
                    pending_count<=0;q_wr_ptr<=0;q_rd_ptr<=0;q_count<=0;
                    perf_cycles<=0;perf_read_requests<=0;perf_write_requests<=0;
                    state<=(total_pixels==0)?S_DONE:S_RUN;
                end
                S_RUN:if(wr_req&&wr_grant) begin
                    slot_full[write_slot]<=1'b0;slot_active[write_slot]<=1'b0;
                    write_slot<=~write_slot;
                    completed_pixels<=completed_pixels+1'b1;
                    if(completed_pixels+1'b1>=total_pixels) state<=S_DONE;
                end
                S_DONE:begin engine_done<=1;state<=S_IDLE;end
                default:state<=S_IDLE;
            endcase
        end
    end
endmodule
