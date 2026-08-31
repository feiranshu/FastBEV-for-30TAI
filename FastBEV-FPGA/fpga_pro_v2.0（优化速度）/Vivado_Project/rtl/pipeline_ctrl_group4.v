//==============================================================================
// File Name     : pipeline_ctrl_group4.v
// Project       : Fast-BEV Part2 FPGA Accelerator
// Description   : Four-frame INT8 group controller.
//
//   Phase D scope:
//     - Maintain frame_phase and hist_valid_mask.
//     - Sequence Quant -> LUT for frames 1/2/3 into history slots.
//     - Sequence Quant -> LUT current -> SA hist0/1/2 for frame 4.
//     - Use network-required concat layout:
//         temporal 0 = current frame4, temporal 1/2/3 = aligned frame3/2/1.
//     - Clear only metadata after a group completes. DDR history slots are not
//       physically cleared.
//     - No partial/single-step run modes; group4 always runs as a full chain.
//==============================================================================

`timescale 1ns / 1ps

module pipeline_ctrl_group4(
    input                       clk,
    input                       rst_n,

    input                       group_start,
    input         [1 : 0]   comp_mode,
    input         [2 : 0]   ext_mode,
    input         [31 : 0]   group_ctrl,

    input         [   31 : 0]   feat2d_fp32_base,
    input         [   31 : 0]   feat2d_int8_base,
    input         [   31 : 0]   concat_out_base,
    input         [   31 : 0]   lut_base_addr,
    input         [   31 : 0]   lut_size,
    input         [   31 : 0]   frame0_addr,
    input         [   31 : 0]   frame1_addr,
    input         [   31 : 0]   frame2_addr,
    input         [   31 : 0]   frame_size,
    input         [   11 : 0]   img_w,
    input         [   11 : 0]   img_h,
    input         [    7 : 0]   cameras,

    input                       quant_done,
    input                       lut_done,
    input                       sa_done,

    input         [   31 : 0]   error_status_w1c,
    input                       error_status_w1c_valid,

    output reg                  quant_start,
    output reg                  lut_start,
    output reg                  sa_start,
    output reg                  group_done,

    output reg    [    1 : 0]   active_stage,
    output        [   31 : 0]   quant_fp32_base,
    output        [   31 : 0]   quant_int8_base,
    output        [   31 : 0]   quant_total_pixels,
    output        [   31 : 0]   lut_feat2d_base,
    output reg    [   31 : 0]   lut_dst_base,
    output reg                  lut_dst_mode,
    output reg    [   31 : 0]   sa_src_addr,
    output        [   31 : 0]   sa_concat_base,
    output reg    [    1 : 0]   sa_temporal_idx,

    output        [   31 : 0]   group_status,
    output reg    [   31 : 0]   error_status
);

    localparam ST_IDLE        = 4'd0;
    localparam ST_QUANT       = 4'd1;
    localparam ST_LUT_HIST    = 4'd2;
    localparam ST_LUT_CURRENT = 4'd3;
    localparam ST_SA          = 4'd4;
    localparam ST_DONE        = 4'd5;
    localparam ST_ERROR       = 4'd6;

    localparam STAGE_NONE  = 2'd0;
    localparam STAGE_QUANT = 2'd1;
    localparam STAGE_LUT   = 2'd2;
    localparam STAGE_SA    = 2'd3;

    localparam ERR_ALIGN       = 0;
    localparam ERR_PHASE       = 1;
    localparam ERR_HIST_VALID  = 2;
    localparam ERR_OVERLAP     = 3;
    localparam ERR_BAD_MODE    = 4;

    reg [3:0] state;
    reg [1:0] frame_phase;
    reg [2:0] hist_valid_mask;
    reg       output_ready;
    reg [1:0] fill_slot;
    reg [1:0] sa_slot;

    wire      soft_clear_history_valid = group_ctrl[0];
    wire      force_phase_reset        = group_ctrl[1];
    wire      start_group_mode         = group_start && (comp_mode == 2'b11);

    assign quant_fp32_base     = feat2d_fp32_base;
    assign quant_int8_base     = feat2d_int8_base;
    assign quant_total_pixels  = {24'd0, cameras} * {20'd0, img_h} * {20'd0, img_w};
    assign lut_feat2d_base     = feat2d_int8_base;
    assign sa_concat_base      = concat_out_base;

    assign group_status = {
        2'd0,
        16'd0,
        state[3:0],
        active_stage[1:0],
        hist_valid_mask[2:0],
        frame_phase[1:0],
        output_ready,
        (state != ST_IDLE) && (state != ST_DONE) && (state != ST_ERROR),
        (error_status != 32'd0)
    };

    function addr_aligned64;
        input [31:0] addr;
        begin
            addr_aligned64 = (addr == {addr[31:6], 6'd0});
        end
    endfunction

    function ranges_overlap;
        input [31:0] base_a;
        input [31:0] base_b;
        input [31:0] size_bytes;
        reg   [32:0] end_a;
        reg   [32:0] end_b;
        begin
            end_a = {1'b0, base_a} + {1'b0, size_bytes};
            end_b = {1'b0, base_b} + {1'b0, size_bytes};
            ranges_overlap = ({1'b0, base_a} < end_b) && ({1'b0, base_b} < end_a);
        end
    endfunction

    function [31:0] select_hist_base;
        input [1:0] slot;
        begin
            case(slot)
                2'd0: select_hist_base = frame0_addr;
                2'd1: select_hist_base = frame1_addr;
                2'd2: select_hist_base = frame2_addr;
                default: select_hist_base = frame0_addr;
            endcase
        end
    endfunction

    function [31:0] validate_config;
        input dummy;
        reg [31:0] err;
        begin
            err = 32'd0;
            if(dummy)
                err = err;
            if(!addr_aligned64(feat2d_fp32_base) ||
               !addr_aligned64(feat2d_int8_base) ||
               !addr_aligned64(frame0_addr) ||
               !addr_aligned64(frame1_addr) ||
               !addr_aligned64(frame2_addr) ||
               !addr_aligned64(concat_out_base))
                err[ERR_ALIGN] = 1'b1;

            if(ranges_overlap(frame0_addr, frame1_addr, frame_size) ||
               ranges_overlap(frame0_addr, frame2_addr, frame_size) ||
               ranges_overlap(frame1_addr, frame2_addr, frame_size) ||
               ranges_overlap(frame0_addr, concat_out_base, frame_size) ||
               ranges_overlap(frame1_addr, concat_out_base, frame_size) ||
               ranges_overlap(frame2_addr, concat_out_base, frame_size))
                err[ERR_OVERLAP] = 1'b1;

            validate_config = err;
        end
    endfunction

    always @(*) begin
        case(fill_slot)
            2'd0: lut_dst_base = frame0_addr;
            2'd1: lut_dst_base = frame1_addr;
            2'd2: lut_dst_base = frame2_addr;
            default: lut_dst_base = concat_out_base;
        endcase

        if(state == ST_LUT_CURRENT) begin
            lut_dst_base = concat_out_base;
            lut_dst_mode = 1'b1;
        end else begin
            lut_dst_mode = 1'b0;
        end

        sa_src_addr = select_hist_base(sa_slot);
        sa_temporal_idx = 2'd3 - sa_slot;
    end

    always @(posedge clk) begin
        if(!rst_n) begin
            state           <= ST_IDLE;
            frame_phase     <= 2'd0;
            hist_valid_mask <= 3'd0;
            output_ready    <= 1'b0;
            fill_slot       <= 2'd0;
            sa_slot         <= 2'd0;
            quant_start     <= 1'b0;
            lut_start       <= 1'b0;
            sa_start        <= 1'b0;
            group_done      <= 1'b0;
            active_stage    <= STAGE_NONE;
            error_status    <= 32'd0;
        end else begin
            quant_start <= 1'b0;
            lut_start   <= 1'b0;
            sa_start    <= 1'b0;
            group_done  <= 1'b0;

            if(error_status_w1c_valid)
                error_status <= error_status & ~error_status_w1c;

            if(soft_clear_history_valid) begin
                hist_valid_mask <= 3'd0;
                output_ready    <= 1'b0;
            end

            if(force_phase_reset) begin
                frame_phase     <= 2'd0;
                hist_valid_mask <= 3'd0;
                output_ready    <= 1'b0;
                state           <= ST_IDLE;
                active_stage    <= STAGE_NONE;
            end else begin
                case(state)
                    ST_IDLE: begin
                        active_stage <= STAGE_NONE;
                        if(start_group_mode) begin
                            output_ready <= 1'b0;
                            if(validate_config(1'b0) != 32'd0) begin
                                error_status <= error_status | validate_config(1'b0);
                                state <= ST_ERROR;
                            end else begin
                                fill_slot <= frame_phase;
                                if(frame_phase == 2'd3 && hist_valid_mask != 3'b111) begin
                                    error_status[ERR_HIST_VALID] <= 1'b1;
                                    state <= ST_ERROR;
                                end else begin
                                    quant_start  <= 1'b1;
                                    active_stage <= STAGE_QUANT;
                                    state <= ST_QUANT;
                                end
                            end
                        end
                    end

                    ST_QUANT: begin
                        active_stage <= STAGE_QUANT;
                        if(quant_done) begin
                            lut_start    <= 1'b1;
                            active_stage <= STAGE_LUT;
                            state <= (frame_phase == 2'd3) ? ST_LUT_CURRENT : ST_LUT_HIST;
                        end
                    end

                    ST_LUT_HIST: begin
                        active_stage <= STAGE_LUT;
                        if(lut_done) begin
                            case(fill_slot)
                                2'd0: hist_valid_mask[0] <= 1'b1;
                                2'd1: hist_valid_mask[1] <= 1'b1;
                                2'd2: hist_valid_mask[2] <= 1'b1;
                                default: error_status[ERR_PHASE] <= 1'b1;
                            endcase
                            if(fill_slot < 2'd2)
                                frame_phase <= fill_slot + 1'b1;
                            else
                                frame_phase <= 2'd3;
                            state <= ST_DONE;
                        end
                    end

                    ST_LUT_CURRENT: begin
                        active_stage <= STAGE_LUT;
                        if(lut_done) begin
                            sa_slot      <= 2'd0;
                            sa_start     <= 1'b1;
                            active_stage <= STAGE_SA;
                            state <= ST_SA;
                        end
                    end

                    ST_SA: begin
                        active_stage <= STAGE_SA;
                        if(sa_done) begin
                            if(frame_phase == 2'd3) begin
                                if(sa_slot < 2'd2) begin
                                    sa_slot  <= sa_slot + 1'b1;
                                    sa_start <= 1'b1;
                                end else begin
                                    output_ready    <= 1'b1;
                                    hist_valid_mask <= 3'd0;
                                    frame_phase     <= 2'd0;
                                    state <= ST_DONE;
                                end
                            end else begin
                                state <= ST_DONE;
                            end
                        end
                    end

                    ST_DONE: begin
                        active_stage <= STAGE_NONE;
                        group_done   <= 1'b1;
                        state        <= ST_IDLE;
                    end

                    ST_ERROR: begin
                        active_stage <= STAGE_NONE;
                        if(error_status == 32'd0)
                            state <= ST_IDLE;
                    end

                    default: begin
                        error_status[ERR_PHASE] <= 1'b1;
                        state <= ST_ERROR;
                    end
                endcase
            end
        end
    end

    wire unused_config_inputs;
    assign unused_config_inputs = |group_ctrl[31:3] | |lut_base_addr | |lut_size | |ext_mode;

endmodule
