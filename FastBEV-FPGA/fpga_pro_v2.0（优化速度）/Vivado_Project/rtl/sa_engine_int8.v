//==============================================================================
// Low-latency INT8 spatial-alignment engine for Fast-BEV Part2.
// External ports, register programming and Part3 memory layout are unchanged.
//==============================================================================
`timescale 1ns/1ps

module sa_engine_int8 #(
    parameter integer INTERP_FRAC_BITS = 6,
    parameter integer MAX_OUTSTANDING  = 4
)(
    input engine_start, output reg engine_done,
    input [31:0] sa_src_addr, input [31:0] concat_base_addr,
    input [31:0] sa_size, input [7:0] bev_x, input [7:0] bev_y,
    input [7:0] bev_z, input [1:0] temporal_idx,
    input [31:0] xform_a00, input [31:0] xform_a01, input [31:0] xform_a02,
    input [31:0] xform_a10, input [31:0] xform_a11, input [31:0] xform_a12,
    output reg [31:0] rd_addr, output reg rd_req, input rd_grant,
    input [511:0] rd_data, input rd_data_valid, output rd_data_ready,
    output reg [31:0] wr_addr, output reg [511:0] wr_data,
    output reg wr_req, input wr_grant, input clk, input rst_n
);

    localparam [4:0] S_IDLE=0, S_PREP=1, S_ADDR=2, S_FETCH_INIT=3,
        S_FETCH=4, S_FETCH_WAIT=5, S_FEED=6, S_PIPE_WAIT=7,
        S_CAPTURE=8, S_OUTPUT=9, S_PAIR_READ=10, S_PAIR_WR0=11,
        S_PAIR_WR1=12, S_RMW_ADDR=13, S_RMW_READ=14, S_RMW_WAIT=15,
        S_RMW_WRITE=16, S_NEXT=17, S_DONE=18, S_ADDR_SUM=19,
        S_ADDR_FINAL=20, S_RMW_INDEX=21, S_RMW_ALIGN=22;
    localparam [2:0] MAX_OUTSTANDING_C =
        (MAX_OUTSTANDING < 1) ? 3'd1 :
        (MAX_OUTSTANDING > 4) ? 3'd4 : MAX_OUTSTANDING;
    localparam [8:0] FRAC_ONE = (9'd1 << INTERP_FRAC_BITS);

    reg [4:0] state;
    reg [31:0] pix_cnt;
    reg [15:0] cur_x, cur_y, cur_z;
    reg signed [47:0] row_u_fp, row_v_fp, coord_u_fp, coord_v_fp;
    wire signed [47:0] a00_ext={{16{xform_a00[31]}},xform_a00};
    wire signed [47:0] a01_ext={{16{xform_a01[31]}},xform_a01};
    wire signed [47:0] a02_ext={{16{xform_a02[31]}},xform_a02};
    wire signed [47:0] a10_ext={{16{xform_a10[31]}},xform_a10};
    wire signed [47:0] a11_ext={{16{xform_a11[31]}},xform_a11};
    wire signed [47:0] a12_ext={{16{xform_a12[31]}},xform_a12};

    wire signed [15:0] coord_u_int_w=coord_u_fp[31:16];
    wire signed [15:0] coord_v_int_w=coord_v_fp[31:16];
    wire signed [15:0] bound_x_w=$signed({8'd0,bev_x})-16'sd1;
    wire signed [15:0] bound_y_w=$signed({8'd0,bev_y})-16'sd1;
    wire coord_valid_w=(bev_x>1)&&(bev_y>1)&&(coord_u_int_w>=0)&&
        (coord_v_int_w>=0)&&(coord_u_int_w<bound_x_w)&&
        (coord_v_int_w<bound_y_w);

    function [8:0] quantize_fraction;
        input [15:0] frac;
        integer rounded;
        begin
            if (INTERP_FRAC_BITS == 8)
                quantize_fraction={1'b0,frac[15:8]};
            else begin
                rounded=(frac+(1<<(15-INTERP_FRAC_BITS)))>>
                        (16-INTERP_FRAC_BITS);
                if (rounded>(1<<INTERP_FRAC_BITS))
                    quantize_fraction=(1<<INTERP_FRAC_BITS);
                else quantize_fraction=rounded[8:0];
            end
        end
    endfunction

    reg signed [15:0] u_int_r, v_int_r;
    reg [8:0] frac_x_q, frac_y_q;
    reg [15:0] plane_size_r;
    reg combine_mode;
    wire [15:0] plane_size_w=bev_x*bev_y;
    wire [23:0] full_count_w=plane_size_w*bev_z;
    wire [23:0] src_z_w=cur_z[7:0]*plane_size_r;
    wire [15:0] src_row_w=v_int_r[7:0]*bev_x;
    reg [23:0] src_z_r;
    reg [15:0] src_row_r;
    reg [31:0] src_index_r;

    reg [31:0] paddr0, paddr1, paddr2, paddr3;
    reg [3:0] required_mask, received_mask;
    reg [2:0] issue_corner;
    wire need_l=(frac_x_q!=FRAC_ONE), need_r=(frac_x_q!=0);
    wire need_t=(frac_y_q!=FRAC_ONE), need_b=(frac_y_q!=0);
    wire [3:0] required_mask_w={need_r&&need_b,need_l&&need_b,
                                need_r&&need_t,need_l&&need_t};
    reg [511:0] pbuf0, pbuf1, pbuf2, pbuf3, interp_result;

    reg [31:0] cache_tag[0:3];
    reg [511:0] cache_mem[0:3];
    reg [3:0] cache_valid;
    reg [1:0] cache_replace;
    // Two-stage, stallable cache lookup pipeline. Stage 0 holds the selected
    // corner address; stage 1 registers tag-compare/data-select results before
    // they drive the 512-bit pbuf registers or the DDR miss queue.
    reg lookup0_valid;
    reg [1:0] lookup0_corner;
    reg [31:0] lookup0_addr;
    reg lookup1_valid;
    reg lookup1_hit;
    reg [1:0] lookup1_corner;
    reg [31:0] lookup1_addr;
    reg [511:0] lookup1_data;
    // One registered target bit per pbuf avoids a shared hit/decode net with
    // fanout across all four 512-bit destination banks.
    (* max_fanout = 64 *) reg [3:0] lookup1_target;
    reg [1:0] req_corner_fifo[0:3];
    reg [31:0] req_addr_fifo[0:3];
    reg [1:0] req_wr_ptr, req_rd_ptr;
    reg [2:0] pending_count;

    reg [31:0] corner_addr_w;
    reg corner_needed_w;
    always @(*) begin
        corner_addr_w=0; corner_needed_w=0;
        case(issue_corner)
            0: begin corner_addr_w=paddr0; corner_needed_w=required_mask[0]; end
            1: begin corner_addr_w=paddr1; corner_needed_w=required_mask[1]; end
            2: begin corner_addr_w=paddr2; corner_needed_w=required_mask[2]; end
            3: begin corner_addr_w=paddr3; corner_needed_w=required_mask[3]; end
            default: begin corner_addr_w=0; corner_needed_w=0; end
        endcase
    end

    reg cache_hit_w;
    reg [511:0] cache_hit_data_w;
    integer ci;
    always @(*) begin
        cache_hit_w=0; cache_hit_data_w=0;
        for(ci=0;ci<4;ci=ci+1)
            if(cache_valid[ci]&&cache_tag[ci]==lookup0_addr) begin
                cache_hit_w=1; cache_hit_data_w=cache_mem[ci];
            end
    end

    wire fetch_can_issue=(state==S_FETCH)&&lookup1_valid&&!lookup1_hit&&
                         (pending_count<MAX_OUTSTANDING_C);
    wire fetch_issue_fire=fetch_can_issue&&rd_grant;
    wire lookup1_consume=lookup1_valid&&
                         (lookup1_hit||fetch_issue_fire);
    wire lookup1_ready=!lookup1_valid||lookup1_consume;
    wire lookup0_to_lookup1=lookup0_valid&&lookup1_ready;
    wire lookup0_ready=!lookup0_valid||lookup0_to_lookup1;
    wire fetch_response_state=(state==S_FETCH)||(state==S_FETCH_WAIT);
    wire fetch_response_fire=fetch_response_state&&(pending_count!=0)&&
                             rd_data_valid;

    reg [255:0] row_low[0:255], row_high[0:255];
    reg [255:0] pair_low_r, pair_high_r;
    reg [31:0] pair_addr0_r, pair_addr1_r;
    wire [10:0] temporal_group_w={8'd0,temporal_idx,1'b0};
    wire [10:0] out_group0_w={cur_z[7:0],3'b0}+temporal_group_w;
    wire [10:0] out_group1_w=out_group0_w+1'b1;
    wire [31:0] out_base0_w=out_group0_w*plane_size_r;
    wire [31:0] out_base1_w=out_group1_w*plane_size_r;
    wire [15:0] out_x_w=cur_x[7:0]*bev_y;
    reg [31:0] out_base0_r, out_base1_r;
    reg [15:0] out_x_r;
    wire [15:0] pair_y_w=cur_y-1'b1;
    wire [31:0] pair_idx0_w=out_base0_r+{16'd0,out_x_r}+{16'd0,pair_y_w};
    wire [31:0] pair_idx1_w=out_base1_r+{16'd0,out_x_r}+{16'd0,pair_y_w};
    wire [31:0] pair_addr0_w=concat_base_addr+(pair_idx0_w<<5);
    wire [31:0] pair_addr1_w=concat_base_addr+(pair_idx1_w<<5);

    reg rmw_half, rmw_upper_r;
    reg [31:0] rmw_addr_r;
    reg [31:0] rmw_base_r, rmw_half_addr_r;
    reg [255:0] rmw_payload_r;
    reg [511:0] rmw_merge_r;
    wire [31:0] rmw_idx_w=rmw_base_r+{16'd0,out_x_r}+{16'd0,cur_y};
    wire [31:0] rmw_half_addr_w=concat_base_addr+(rmw_idx_w<<5);
    wire [255:0] rmw_payload_w=rmw_half?interp_result[511:256]:
                                      interp_result[255:0];

    // Four channel groups enter a five-level, 16-lane pipeline consecutively.
    reg [1:0] feed_group;
    reg [127:0] feed0,feed1,feed2,feed3;
    always @(*) begin
        case(feed_group)
            0: begin feed0=pbuf0[127:0];feed1=pbuf1[127:0];
                     feed2=pbuf2[127:0];feed3=pbuf3[127:0];end
            1: begin feed0=pbuf0[255:128];feed1=pbuf1[255:128];
                     feed2=pbuf2[255:128];feed3=pbuf3[255:128];end
            2: begin feed0=pbuf0[383:256];feed1=pbuf1[383:256];
                     feed2=pbuf2[383:256];feed3=pbuf3[383:256];end
            default: begin feed0=pbuf0[511:384];feed1=pbuf1[511:384];
                     feed2=pbuf2[511:384];feed3=pbuf3[511:384];end
        endcase
    end
    reg pv0,pv1,pv2,pv3,pv4,pv5;
    reg [1:0] pg0,pg1,pg2,pg3,pg4,pg5;
    reg signed [8:0] dt0[0:15],db0[0:15],bt0[0:15],bb0[0:15];
    reg [8:0] fx0,fy0,fy1,fy2,fy3;
    reg signed [18:0] pt1[0:15],pb1[0:15];
    reg signed [8:0] bt1[0:15],bb1[0:15];
    reg signed [19:0] ht2[0:15],hb2[0:15],ht3[0:15],ht4[0:15];
    reg signed [20:0] diff3[0:15];
    reg signed [29:0] vp4[0:15],acc5[0:15];
    reg [511:0] pipe_result;

    function [7:0] round_sat;
        input signed [29:0] acc;
        reg [29:0] mag,rounded;
        begin
            mag=acc[29]?-acc:acc;
            rounded=(mag+(30'd1<<(2*INTERP_FRAC_BITS-1)))>>
                    (2*INTERP_FRAC_BITS);
            if(!acc[29]&&rounded>127) round_sat=8'h7f;
            else if(acc[29]&&rounded>128) round_sat=8'h80;
            else if(acc[29]) round_sat=(~rounded[7:0])+1'b1;
            else round_sat=rounded[7:0];
        end
    endfunction

    integer ln;
    always @(posedge clk) begin
        if(!rst_n) begin
            pv0<=0;pv1<=0;pv2<=0;pv3<=0;pv4<=0;pv5<=0;
            pg0<=0;pg1<=0;pg2<=0;pg3<=0;pg4<=0;pg5<=0;pipe_result<=0;
        end else begin
            pv0<=(state==S_FEED);pv1<=pv0;pv2<=pv1;pv3<=pv2;pv4<=pv3;
            pv5<=pv4;
            pg0<=feed_group;pg1<=pg0;pg2<=pg1;pg3<=pg2;pg4<=pg3;
            pg5<=pg4;
            if(state==S_FEED) begin
                fx0<=frac_x_q;fy0<=frac_y_q;
                for(ln=0;ln<16;ln=ln+1) begin
                    bt0[ln]<=$signed({feed0[ln*8+7],feed0[ln*8+:8]});
                    bb0[ln]<=$signed({feed2[ln*8+7],feed2[ln*8+:8]});
                    dt0[ln]<=$signed({feed1[ln*8+7],feed1[ln*8+:8]})-
                             $signed({feed0[ln*8+7],feed0[ln*8+:8]});
                    db0[ln]<=$signed({feed3[ln*8+7],feed3[ln*8+:8]})-
                             $signed({feed2[ln*8+7],feed2[ln*8+:8]});
                end
            end
            if(pv0) begin
                fy1<=fy0;
                for(ln=0;ln<16;ln=ln+1) begin
                    pt1[ln]<=dt0[ln]*$signed({1'b0,fx0});
                    pb1[ln]<=db0[ln]*$signed({1'b0,fx0});
                    bt1[ln]<=bt0[ln];bb1[ln]<=bb0[ln];
                end
            end
            if(pv1) begin
                fy2<=fy1;
                for(ln=0;ln<16;ln=ln+1) begin
                    ht2[ln]<=($signed({{11{bt1[ln][8]}},bt1[ln]})<<<
                              INTERP_FRAC_BITS)+pt1[ln];
                    hb2[ln]<=($signed({{11{bb1[ln][8]}},bb1[ln]})<<<
                              INTERP_FRAC_BITS)+pb1[ln];
                end
            end
            if(pv2) begin
                fy3<=fy2;
                for(ln=0;ln<16;ln=ln+1) begin
                    diff3[ln]<=$signed({hb2[ln][19],hb2[ln]})-
                               $signed({ht2[ln][19],ht2[ln]});
                    ht3[ln]<=ht2[ln];
                end
            end
            if(pv3) for(ln=0;ln<16;ln=ln+1) begin
                vp4[ln]<=diff3[ln]*$signed({1'b0,fy3});
                ht4[ln]<=ht3[ln];
            end
            if(pv4) for(ln=0;ln<16;ln=ln+1)
                acc5[ln]<=($signed({{10{ht4[ln][19]}},ht4[ln]})<<<
                           INTERP_FRAC_BITS)+vp4[ln];
            if(pv5) for(ln=0;ln<16;ln=ln+1)
                pipe_result[(pg5*128)+(ln*8)+:8]<=round_sat(acc5[ln]);
        end
    end
    wire interp_last_w=pv5&&(pg5==3);

    // Internal statistics are visible to simulation only through hierarchy.
    reg [63:0] perf_cycles,perf_read_requests,perf_write_requests,perf_cache_hits;

    always @(*) begin
        rd_addr=0;rd_req=0;
        if(fetch_can_issue) begin rd_addr=lookup1_addr;rd_req=1;end
        else if(state==S_RMW_READ) begin rd_addr=rmw_addr_r;rd_req=1;end
        wr_addr=0;wr_data=0;wr_req=0;
        if(state==S_PAIR_WR0) begin
            wr_addr=pair_addr0_r;wr_data={interp_result[255:0],pair_low_r};wr_req=1;
        end else if(state==S_PAIR_WR1) begin
            wr_addr=pair_addr1_r;wr_data={interp_result[511:256],pair_high_r};wr_req=1;
        end else if(state==S_RMW_WRITE) begin
            wr_addr=rmw_addr_r;wr_data=rmw_merge_r;wr_req=1;
        end
    end
    assign rd_data_ready=(fetch_response_state&&(pending_count!=0))||
                         (state==S_RMW_WAIT);

    integer ri;
    always @(posedge clk) begin
        if(!rst_n) begin
            state<=S_IDLE;engine_done<=0;pix_cnt<=0;cur_x<=0;cur_y<=0;cur_z<=0;
            row_u_fp<=0;row_v_fp<=0;coord_u_fp<=0;coord_v_fp<=0;
            u_int_r<=0;v_int_r<=0;frac_x_q<=0;frac_y_q<=0;plane_size_r<=0;
            src_z_r<=0;src_row_r<=0;src_index_r<=0;
            out_base0_r<=0;out_base1_r<=0;out_x_r<=0;
            combine_mode<=0;required_mask<=0;received_mask<=0;issue_corner<=0;
            pbuf0<=0;pbuf1<=0;pbuf2<=0;pbuf3<=0;interp_result<=0;
            cache_valid<=0;cache_replace<=0;req_wr_ptr<=0;req_rd_ptr<=0;
            pending_count<=0;feed_group<=0;rmw_half<=0;rmw_addr_r<=0;
            lookup0_valid<=0;lookup0_corner<=0;lookup0_addr<=0;
            lookup1_valid<=0;lookup1_hit<=0;lookup1_corner<=0;
            lookup1_addr<=0;lookup1_data<=0;lookup1_target<=0;
            rmw_base_r<=0;rmw_half_addr_r<=0;
            rmw_upper_r<=0;rmw_payload_r<=0;rmw_merge_r<=0;
            pair_low_r<=0;pair_high_r<=0;pair_addr0_r<=0;pair_addr1_r<=0;
            perf_cycles<=0;perf_read_requests<=0;perf_write_requests<=0;
            perf_cache_hits<=0;
            for(ri=0;ri<4;ri=ri+1) begin
                cache_tag[ri]<=0;cache_mem[ri]<=0;req_corner_fifo[ri]<=0;
                req_addr_fifo[ri]<=0;
            end
        end else begin
            engine_done<=0;
            if(state!=S_IDLE&&state!=S_DONE) perf_cycles<=perf_cycles+1'b1;
            if(fetch_issue_fire||(state==S_RMW_READ&&rd_grant))
                perf_read_requests<=perf_read_requests+1'b1;
            if(wr_req&&wr_grant) perf_write_requests<=perf_write_requests+1'b1;

            if(fetch_response_fire) begin
                case(req_corner_fifo[req_rd_ptr])
                    0: begin pbuf0<=rd_data;received_mask[0]<=1;end
                    1: begin pbuf1<=rd_data;received_mask[1]<=1;end
                    2: begin pbuf2<=rd_data;received_mask[2]<=1;end
                    default: begin pbuf3<=rd_data;received_mask[3]<=1;end
                endcase
                cache_tag[cache_replace]<=req_addr_fifo[req_rd_ptr];
                cache_mem[cache_replace]<=rd_data;cache_valid[cache_replace]<=1;
                cache_replace<=cache_replace+1'b1;req_rd_ptr<=req_rd_ptr+1'b1;
            end
            if(fetch_issue_fire) begin
                req_corner_fifo[req_wr_ptr]<=lookup1_corner;
                req_addr_fifo[req_wr_ptr]<=lookup1_addr;
                req_wr_ptr<=req_wr_ptr+1'b1;
            end
            case({fetch_issue_fire,fetch_response_fire})
                2'b10:pending_count<=pending_count+1'b1;
                2'b01:pending_count<=pending_count-1'b1;
                default:begin end
            endcase

            case(state)
                S_IDLE:if(engine_start) begin
                    pix_cnt<=0;cur_x<=0;cur_y<=0;cur_z<=0;
                    row_u_fp<=a02_ext;row_v_fp<=a12_ext;
                    coord_u_fp<=a02_ext;coord_v_fp<=a12_ext;
                    plane_size_r<=plane_size_w;
                    combine_mode<=!bev_y[0]&&(sa_size=={8'd0,full_count_w});
                    cache_valid<=0;cache_replace<=0;req_wr_ptr<=0;req_rd_ptr<=0;
                    lookup0_valid<=0;lookup1_valid<=0;
                    pending_count<=0;perf_cycles<=0;perf_read_requests<=0;
                    perf_write_requests<=0;perf_cache_hits<=0;
                    state<=(sa_size==0)?S_DONE:S_PREP;
                end
                S_PREP:begin
                    u_int_r<=coord_u_int_w;v_int_r<=coord_v_int_w;
                    frac_x_q<=quantize_fraction(coord_u_fp[15:0]);
                    frac_y_q<=quantize_fraction(coord_v_fp[15:0]);
                    out_base0_r<=out_base0_w;out_base1_r<=out_base1_w;
                    out_x_r<=out_x_w;
                    if(coord_valid_w) state<=S_ADDR;
                    else begin interp_result<=0;state<=S_OUTPUT;end
                end
                S_ADDR:begin
                    src_z_r<=src_z_w;src_row_r<=src_row_w;
                    state<=S_ADDR_SUM;
                end
                S_ADDR_SUM:begin
                    src_index_r<={8'd0,src_z_r}+{16'd0,src_row_r}+
                                 {24'd0,u_int_r[7:0]};
                    state<=S_ADDR_FINAL;
                end
                S_ADDR_FINAL:begin
                    paddr0<=sa_src_addr+(src_index_r<<6);
                    paddr1<=sa_src_addr+((src_index_r+1'b1)<<6);
                    paddr2<=sa_src_addr+((src_index_r+bev_x)<<6);
                    paddr3<=sa_src_addr+((src_index_r+bev_x+1'b1)<<6);
                    required_mask<=required_mask_w;state<=S_FETCH_INIT;
                end
                S_FETCH_INIT:begin
                    issue_corner<=0;received_mask<=~required_mask;
                    req_wr_ptr<=0;req_rd_ptr<=0;pending_count<=0;
                    lookup0_valid<=0;lookup1_valid<=0;state<=S_FETCH;
                end
                S_FETCH:begin
                    // Resolve only registered lookup results. The wide pbuf
                    // input no longer includes address mux, tag compare and
                    // cache-data selection in the same cycle.
                    if(lookup1_valid&&lookup1_hit) begin
                        if(lookup1_target[0]) begin
                            pbuf0<=lookup1_data;received_mask[0]<=1;
                        end
                        if(lookup1_target[1]) begin
                            pbuf1<=lookup1_data;received_mask[1]<=1;
                        end
                        if(lookup1_target[2]) begin
                            pbuf2<=lookup1_data;received_mask[2]<=1;
                        end
                        if(lookup1_target[3]) begin
                            pbuf3<=lookup1_data;received_mask[3]<=1;
                        end
                        perf_cache_hits<=perf_cache_hits+1'b1;
                    end

                    // Stage 1 advances only when its registered result is
                    // consumed. A stalled DDR miss backpressures both stages.
                    if(lookup1_ready) begin
                        if(lookup0_valid) begin
                            lookup1_valid<=1;
                            lookup1_hit<=cache_hit_w;
                            lookup1_corner<=lookup0_corner;
                            lookup1_addr<=lookup0_addr;
                            lookup1_data<=cache_hit_data_w;
                            lookup1_target<=cache_hit_w ?
                                (4'b0001<<lookup0_corner) : 4'b0000;
                        end else begin
                            lookup1_valid<=0;
                            lookup1_target<=0;
                        end
                    end

                    // Launch one required corner per cycle whenever stage 0
                    // can advance; unused endpoint corners are skipped.
                    if(lookup0_ready) begin
                        if(issue_corner<4) begin
                            if(corner_needed_w) begin
                                lookup0_valid<=1;
                                lookup0_corner<=issue_corner[1:0];
                                lookup0_addr<=corner_addr_w;
                            end else lookup0_valid<=0;
                            issue_corner<=issue_corner+1'b1;
                        end else lookup0_valid<=0;
                    end

                    if(issue_corner>=4&&!lookup0_valid&&!lookup1_valid)
                        state<=S_FETCH_WAIT;
                end
                S_FETCH_WAIT:if(pending_count==0&&(&received_mask)) begin
                    feed_group<=0;state<=S_FEED;
                end
                S_FEED:if(feed_group==3) state<=S_PIPE_WAIT;
                       else feed_group<=feed_group+1'b1;
                S_PIPE_WAIT:if(interp_last_w) state<=S_CAPTURE;
                S_CAPTURE:begin interp_result<=pipe_result;state<=S_OUTPUT;end
                S_OUTPUT:if(combine_mode) begin
                    if(!cur_y[0]) begin
                        row_low[cur_x[7:0]]<=interp_result[255:0];
                        row_high[cur_x[7:0]]<=interp_result[511:256];state<=S_NEXT;
                    end else state<=S_PAIR_READ;
                end else begin rmw_half<=0;state<=S_RMW_ADDR;end
                S_PAIR_READ:begin
                    pair_low_r<=row_low[cur_x[7:0]];
                    pair_high_r<=row_high[cur_x[7:0]];
                    pair_addr0_r<=pair_addr0_w;pair_addr1_r<=pair_addr1_w;
                    state<=S_PAIR_WR0;
                end
                S_PAIR_WR0:if(wr_grant) state<=S_PAIR_WR1;
                S_PAIR_WR1:if(wr_grant) state<=S_NEXT;
                S_RMW_ADDR:begin
                    rmw_base_r<=rmw_half?out_base1_r:out_base0_r;
                    rmw_payload_r<=rmw_payload_w;state<=S_RMW_INDEX;
                end
                S_RMW_INDEX:begin
                    rmw_half_addr_r<=rmw_half_addr_w;state<=S_RMW_ALIGN;
                end
                S_RMW_ALIGN:begin
                    rmw_addr_r<={rmw_half_addr_r[31:6],6'b0};
                    rmw_upper_r<=rmw_half_addr_r[5];state<=S_RMW_READ;
                end
                S_RMW_READ:if(rd_grant) state<=S_RMW_WAIT;
                S_RMW_WAIT:if(rd_data_valid) begin
                    if(rmw_upper_r) rmw_merge_r<={rmw_payload_r,rd_data[255:0]};
                    else rmw_merge_r<={rd_data[511:256],rmw_payload_r};
                    state<=S_RMW_WRITE;
                end
                S_RMW_WRITE:if(wr_grant) begin
                    if(!rmw_half) begin rmw_half<=1;state<=S_RMW_ADDR;end
                    else state<=S_NEXT;
                end
                S_NEXT:if(pix_cnt+1'b1>=sa_size) state<=S_DONE;
                else begin
                    pix_cnt<=pix_cnt+1'b1;
                    if(cur_x=={8'd0,bev_x}-1'b1) begin
                        cur_x<=0;
                        if(cur_y=={8'd0,bev_y}-1'b1) begin
                            cur_y<=0;cur_z<=cur_z+1'b1;
                            row_u_fp<=a02_ext;row_v_fp<=a12_ext;
                            coord_u_fp<=a02_ext;coord_v_fp<=a12_ext;
                        end else begin
                            cur_y<=cur_y+1'b1;
                            row_u_fp<=row_u_fp+a01_ext;row_v_fp<=row_v_fp+a11_ext;
                            coord_u_fp<=row_u_fp+a01_ext;
                            coord_v_fp<=row_v_fp+a11_ext;
                        end
                    end else begin
                        cur_x<=cur_x+1'b1;
                        coord_u_fp<=coord_u_fp+a00_ext;
                        coord_v_fp<=coord_v_fp+a10_ext;
                    end
                    state<=S_PREP;
                end
                S_DONE:begin engine_done<=1;state<=S_IDLE;end
                default:state<=S_IDLE;
            endcase
        end
    end
endmodule
