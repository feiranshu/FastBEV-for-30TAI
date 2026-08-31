//==============================================================================
// File Name     : lut_engine_int8.v
// Project       : Fast-BEV Part2 FPGA Accelerator
// Description   : INT8 LUT-based 2D-to-3D view transformation engine.
//
//   Data format:
//     2D feature: [cam][v][u][channel], 64 signed-int8 channels = 64B.
//     LUT entry : {pad[16], v[16], u[16], cam_id[16]} = 8B.
//     Output    : one 512-bit beat per voxel.
//
//   dst_mode:
//     0 = HISTORY_CONTIG : dst = dst_base_addr + voxel_idx * 64
//     1 = FUSED_CURRENT  : final Part3 layout
//
//   Final Part3 layout:
//     logical input   [N][X][Y][Z][C], C = 4 frames * 64 channels
//     physical output [N][Z][C/32][X][Y][C%32]
//     byte_addr = base + (((z * 8 + c/32) * X + x) * Y + y) * 32 + c%32.
//     temporal order  [current, prev1, prev3, prev5]
//   FUSED_CURRENT combines adjacent Y positions into complete 64B writes for
//   the production 200x200x4 shape. Other shapes retain the RMW fallback.
//==============================================================================
`timescale 1ns/1ps

module lut_engine_int8(

    input                       engine_start     ,
    output reg                  engine_done      ,

    input         [   31 : 0]   lut_base_addr    ,
    input         [   31 : 0]   lut_size         ,
    input         [   31 : 0]   feat2d_base_addr ,
    input         [   31 : 0]   dst_base_addr    ,
    input                       dst_mode         ,
    input         [   11 : 0]   img_w            ,
    input         [   11 : 0]   img_h            ,
    input         [    7 : 0]   bev_x            ,
    input         [    7 : 0]   bev_y            ,

    output reg    [   31 : 0]   rd_addr          ,
    output reg                  rd_req           ,
    input                       rd_grant         ,
    input         [  511 : 0]   rd_data          ,
    input                       rd_data_valid    ,
    output                      rd_data_ready    ,

    output reg    [   31 : 0]   wr_addr          ,
    output reg    [  511 : 0]   wr_data          ,
    output reg                  wr_req           ,
    input                       wr_grant         ,

    input                       clk              ,
    input                       rst_n
);

    localparam DST_HISTORY_CONTIG = 1'b0;
    localparam DST_FUSED_CURRENT  = 1'b1;

    localparam S_IDLE       = 5'd0;
    localparam S_REQ_LUT    = 5'd1;
    localparam S_GRANT_LUT  = 5'd2;
    localparam S_WAIT_LUT   = 5'd3;
    localparam S_PARSE_LUT  = 5'd4;
    localparam S_CALC_ADDR  = 5'd5;
    localparam S_CALC_ADDR2 = 5'd6;
    localparam S_CALC_ADDR3 = 5'd7;
    localparam S_REQ_FEAT   = 5'd8;
    localparam S_GRANT_FEAT = 5'd9;
    localparam S_WAIT_FEAT  = 5'd10;
    localparam S_WRITE_BEV  = 5'd11;
    localparam S_GRANT_WR   = 5'd12;
    localparam S_RMW_RD     = 5'd13;
    localparam S_RMW_WAIT   = 5'd14;
    localparam S_RMW_WR     = 5'd15;
    localparam S_NEXT       = 5'd16;
    localparam S_DONE       = 5'd17;
    localparam S_RMW_ADDR0  = 5'd18;
    localparam S_RMW_ADDR1  = 5'd19;
    localparam S_PAIR_READ  = 5'd20;
    localparam S_PAIR_WR0   = 5'd21;
    localparam S_PAIR_WR1   = 5'd22;
    localparam S_PAIR_ADDR  = 5'd23;

    reg  [4:0]   state;
    reg  [31:0]  voxel_cnt;
    reg  [2:0]   lut_sub_idx;
    reg  [511:0] lut_batch;
    reg  [511:0] feat_buffer;
    reg  [15:0]  cur_x;
    reg  [15:0]  cur_y;
    reg  [15:0]  cur_z;
    reg          rmw_half;
    reg  [31:0]  rmw_addr_r;
    reg          rmw_upper_r;
    reg  [255:0] rmw_payload_r;
    reg  [511:0] rmw_merge_data;
    reg  [18:0]  final_group_x_r;
    reg  [26:0]  final_xy_r;
    reg          combine_mode;
    reg  [255:0] row_buffer_low [0:255];
    reg  [255:0] row_buffer_high [0:255];
    reg  [255:0] pair_low_r;
    reg  [255:0] pair_high_r;
    reg  [31:0]  pair_addr0_r;
    reg  [31:0]  pair_addr1_r;
    reg  [31:0]  pair_group0_base_r;
    reg  [31:0]  pair_group1_base_r;
    reg  [15:0]  pair_x_base_r;
    reg  [15:0]  pair_y_even_r;

    reg  signed [15:0] cur_cam_id;
    reg         [15:0] cur_u;
    reg         [15:0] cur_v;

    reg  [20:0] pipe_cam_v;
    reg  [25:0] pixel_idx_r;
    reg  [31:0] feat_rd_addr_r;

    wire need_new_batch;
    assign need_new_batch = (lut_sub_idx == 3'd0);

    wire [31:0] lut_rd_addr;
    assign lut_rd_addr = lut_base_addr + {voxel_cnt[28:3], 6'b0};

    wire [63:0] cur_lut_raw;
    assign cur_lut_raw = lut_batch[lut_sub_idx*64 +: 64];

    wire signed [15:0] raw_cam_id;
    wire [15:0] raw_u;
    wire [15:0] raw_v;
    assign raw_cam_id = cur_lut_raw[15:0];
    assign raw_u      = cur_lut_raw[31:16];
    assign raw_v      = cur_lut_raw[47:32];

    wire signed [15:0] rd_cam_id;
    wire [15:0] rd_u;
    wire [15:0] rd_v;
    assign rd_cam_id = rd_data[15:0];
    assign rd_u      = rd_data[31:16];
    assign rd_v      = rd_data[47:32];

    wire cur_entry_valid;
    wire rd_entry_valid;
    assign cur_entry_valid = (raw_cam_id >= 0);
    assign rd_entry_valid  = (rd_cam_id  >= 0);

    wire [25:0] pixel_linear_idx_s2;
    assign pixel_linear_idx_s2 = pipe_cam_v * img_w + cur_u;

    wire [31:0] history_wr_addr;
    assign history_wr_addr       = dst_base_addr + {voxel_cnt[25:0], 6'b0};

    wire [3:0]   final_cgroup_w;
    wire [10:0]  final_group_w;
    wire [18:0]  final_group_x_product_w;
    wire [26:0]  final_xy_product_w;
    wire [31:0]  final_half_addr_w;
    wire [31:0]  rmw_aligned_addr_w;
    wire         rmw_upper_half_w;
    wire [255:0] rmw_payload_w;

    assign final_cgroup_w     = {3'd0, rmw_half};
    assign final_group_w      = {cur_z[7:0], 3'b0} + {7'd0, final_cgroup_w};
    assign final_group_x_product_w = final_group_w * bev_x;
    assign final_xy_product_w = final_group_x_r * bev_y;
    assign final_half_addr_w  = dst_base_addr + {final_xy_r, 5'b0};
    assign rmw_aligned_addr_w = {final_half_addr_w[31:6], 6'b0};
    assign rmw_upper_half_w   = final_half_addr_w[5];
    assign rmw_payload_w      = (rmw_half == 1'b0) ? feat_buffer[255:0] :
                                                     feat_buffer[511:256];

    wire [15:0] pair_plane_size_w = bev_x * bev_y;
    wire [31:0] pair_full_size_w = {14'd0, pair_plane_size_w, 2'b00};
    wire [10:0] pair_group0_w = {cur_z[7:0], 3'b000};
    wire [10:0] pair_group1_w = pair_group0_w + 1'b1;
    wire [31:0] pair_group0_base_w = pair_group0_w * pair_plane_size_w;
    wire [31:0] pair_group1_base_w = pair_group1_w * pair_plane_size_w;
    wire [15:0] pair_x_base_w = cur_x[7:0] * bev_y;
    wire [15:0] pair_y_even_w = cur_y - 1'b1;
    assign rd_data_ready = (state == S_WAIT_LUT) || (state == S_WAIT_FEAT) ||
                           (state == S_RMW_WAIT);

    always @(posedge clk) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            engine_done    <= 1'b0;
            voxel_cnt      <= 32'd0;
            lut_sub_idx    <= 3'd0;
            lut_batch      <= 512'd0;
            feat_buffer    <= 512'd0;
            cur_x          <= 16'd0;
            cur_y          <= 16'd0;
            cur_z          <= 16'd0;
            rmw_half       <= 1'b0;
            rmw_addr_r     <= 32'd0;
            rmw_upper_r    <= 1'b0;
            rmw_payload_r  <= 256'd0;
            rmw_merge_data <= 512'd0;
            final_group_x_r<= 19'd0;
            final_xy_r     <= 27'd0;
            combine_mode   <= 1'b0;
            pair_low_r     <= 256'd0;
            pair_high_r    <= 256'd0;
            pair_addr0_r   <= 32'd0;
            pair_addr1_r   <= 32'd0;
            pair_group0_base_r <= 32'd0;
            pair_group1_base_r <= 32'd0;
            pair_x_base_r  <= 16'd0;
            pair_y_even_r  <= 16'd0;
            cur_cam_id     <= -16'sd1;
            cur_u          <= 16'd0;
            cur_v          <= 16'd0;
            pipe_cam_v     <= 21'd0;
            pixel_idx_r    <= 26'd0;
            feat_rd_addr_r <= 32'd0;
            rd_addr        <= 32'd0;
            rd_req         <= 1'b0;
            wr_addr        <= 32'd0;
            wr_data        <= 512'd0;
            wr_req         <= 1'b0;
        end else begin
            engine_done <= 1'b0;

            case (state)
                S_IDLE: begin
                    rd_req      <= 1'b0;
                    wr_req      <= 1'b0;
                    voxel_cnt   <= 32'd0;
                    lut_sub_idx <= 3'd0;
                    feat_buffer <= 512'd0;
                    cur_x       <= 16'd0;
                    cur_y       <= 16'd0;
                    cur_z       <= 16'd0;
                    rmw_half    <= 1'b0;
                    if (engine_start) begin
                        combine_mode <= (dst_mode == DST_FUSED_CURRENT) &&
                                        !bev_y[0] &&
                                        (lut_size == pair_full_size_w);
                        if (lut_size == 32'd0)
                            state <= S_DONE;
                        else
                            state <= S_REQ_LUT;
                    end
                end

                S_REQ_LUT: begin
                    if (need_new_batch) begin
                        rd_addr <= lut_rd_addr;
                        rd_req  <= 1'b1;
                        state   <= S_GRANT_LUT;
                    end else begin
                        state <= S_PARSE_LUT;
                    end
                end

                S_GRANT_LUT: begin
                    rd_req <= 1'b1;
                    if (rd_grant) begin
                        rd_req <= 1'b0;
                        state  <= S_WAIT_LUT;
                    end
                end

                S_WAIT_LUT: begin
                    if (rd_data_valid) begin
                        lut_batch  <= rd_data;
                        cur_cam_id <= rd_cam_id;
                        cur_u      <= rd_u;
                        cur_v      <= rd_v;
                        if (rd_entry_valid)
                            state <= S_CALC_ADDR;
                        else begin
                            feat_buffer <= 512'd0;
                            state       <= S_WRITE_BEV;
                        end
                    end
                end

                S_PARSE_LUT: begin
                    cur_cam_id <= raw_cam_id;
                    cur_u      <= raw_u;
                    cur_v      <= raw_v;
                    if (cur_entry_valid)
                        state <= S_CALC_ADDR;
                    else begin
                        feat_buffer <= 512'd0;
                        state       <= S_WRITE_BEV;
                    end
                end

                S_CALC_ADDR: begin
                    pipe_cam_v <= cur_cam_id[7:0] * img_h + cur_v;
                    state      <= S_CALC_ADDR2;
                end

                S_CALC_ADDR2: begin
                    pixel_idx_r <= pixel_linear_idx_s2;
                    state       <= S_CALC_ADDR3;
                end

                S_CALC_ADDR3: begin
                    feat_rd_addr_r <= feat2d_base_addr + {pixel_idx_r[25:0], 6'b0};
                    state          <= S_REQ_FEAT;
                end

                S_REQ_FEAT: begin
                    rd_addr <= feat_rd_addr_r;
                    rd_req  <= 1'b1;
                    state   <= S_GRANT_FEAT;
                end

                S_GRANT_FEAT: begin
                    rd_req <= 1'b1;
                    if (rd_grant) begin
                        rd_req <= 1'b0;
                        state  <= S_WAIT_FEAT;
                    end
                end

                S_WAIT_FEAT: begin
                    if (rd_data_valid) begin
                        feat_buffer <= rd_data;
                        state       <= S_WRITE_BEV;
                    end
                end

                S_WRITE_BEV: begin
                    if (dst_mode == DST_FUSED_CURRENT) begin
                        if (combine_mode) begin
                            if (!cur_y[0]) begin
                                row_buffer_low[cur_x[7:0]] <= feat_buffer[255:0];
                                row_buffer_high[cur_x[7:0]] <= feat_buffer[511:256];
                                state <= S_NEXT;
                            end else begin
                                state <= S_PAIR_READ;
                            end
                        end else begin
                            rmw_half <= 1'b0;
                            state    <= S_RMW_ADDR0;
                        end
                    end else begin
                        wr_addr <= history_wr_addr;
                        wr_data <= feat_buffer;
                        wr_req  <= 1'b1;
                        state   <= S_GRANT_WR;
                    end
                end

                S_GRANT_WR: begin
                    wr_req <= 1'b1;
                    if (wr_grant) begin
                        wr_req <= 1'b0;
                        state  <= S_NEXT;
                    end
                end

                S_PAIR_READ: begin
                    pair_low_r   <= row_buffer_low[cur_x[7:0]];
                    pair_high_r  <= row_buffer_high[cur_x[7:0]];
                    pair_group0_base_r <= pair_group0_base_w;
                    pair_group1_base_r <= pair_group1_base_w;
                    pair_x_base_r <= pair_x_base_w;
                    pair_y_even_r <= pair_y_even_w;
                    state        <= S_PAIR_ADDR;
                end

                S_PAIR_ADDR: begin
                    pair_addr0_r <= dst_base_addr +
                        ((pair_group0_base_r + {16'd0, pair_x_base_r} +
                          {16'd0, pair_y_even_r}) << 5);
                    pair_addr1_r <= dst_base_addr +
                        ((pair_group1_base_r + {16'd0, pair_x_base_r} +
                          {16'd0, pair_y_even_r}) << 5);
                    state <= S_PAIR_WR0;
                end

                S_PAIR_WR0: begin
                    wr_addr <= pair_addr0_r;
                    wr_data <= {feat_buffer[255:0], pair_low_r};
                    wr_req  <= 1'b1;
                    if (wr_grant) begin
                        wr_req <= 1'b0;
                        state  <= S_PAIR_WR1;
                    end
                end

                S_PAIR_WR1: begin
                    wr_addr <= pair_addr1_r;
                    wr_data <= {feat_buffer[511:256], pair_high_r};
                    wr_req  <= 1'b1;
                    if (wr_grant) begin
                        wr_req <= 1'b0;
                        state  <= S_NEXT;
                    end
                end

                S_RMW_RD: begin
                    rd_addr       <= rmw_aligned_addr_w;
                    rmw_addr_r    <= rmw_aligned_addr_w;
                    rmw_upper_r   <= rmw_upper_half_w;
                    rmw_payload_r <= rmw_payload_w;
                    rd_req        <= 1'b1;
                    if (rd_grant) begin
                        rd_req <= 1'b0;
                        state  <= S_RMW_WAIT;
                    end
                end

                S_RMW_ADDR0: begin
                    final_group_x_r <= final_group_x_product_w +
                                       {11'd0, cur_x[7:0]};
                    state <= S_RMW_ADDR1;
                end

                S_RMW_ADDR1: begin
                    final_xy_r <= final_xy_product_w + {19'd0, cur_y[7:0]};
                    state      <= S_RMW_RD;
                end

                S_RMW_WAIT: begin
                    if (rd_data_valid) begin
                        if (rmw_upper_r)
                            rmw_merge_data <= {rmw_payload_r, rd_data[255:0]};
                        else
                            rmw_merge_data <= {rd_data[511:256], rmw_payload_r};
                        state <= S_RMW_WR;
                    end
                end

                S_RMW_WR: begin
                    wr_addr <= rmw_addr_r;
                    wr_data <= rmw_merge_data;
                    wr_req  <= 1'b1;
                    if (wr_grant) begin
                        wr_req <= 1'b0;
                        if (rmw_half == 1'b0) begin
                            rmw_half <= 1'b1;
                            state    <= S_RMW_ADDR0;
                        end else begin
                            state <= S_NEXT;
                        end
                    end
                end

                S_NEXT: begin
                    if (voxel_cnt + 1'b1 >= lut_size) begin
                        state <= S_DONE;
                    end else begin
                        voxel_cnt   <= voxel_cnt + 1'b1;
                        lut_sub_idx <= lut_sub_idx + 1'b1;
                        if (cur_x == {8'd0, bev_x} - 16'd1) begin
                            cur_x <= 16'd0;
                            if (cur_y == {8'd0, bev_y} - 16'd1) begin
                                cur_y <= 16'd0;
                                cur_z <= cur_z + 1'b1;
                            end else begin
                                cur_y <= cur_y + 1'b1;
                            end
                        end else begin
                            cur_x <= cur_x + 1'b1;
                        end
                        state       <= S_REQ_LUT;
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
