//==============================================================================
// alert_led_ctrl.v
//------------------------------------------------------------------------------
// Register-triggered two-LED alert controller.
//
// level 0: off
// level 1: both LEDs continuously on
// level 2: LEDs alternate 01/10
// level 3: LEDs blink synchronously 11/00
//
// All configuration values are sampled together when trigger is asserted.
//==============================================================================
`timescale 1ns/1ps

module alert_led_ctrl #(
    parameter integer CLK_FREQ_HZ = 100_000_000
)(
    input               clk,
    input               rst_n,

    input       [1:0]   level,
    input               trigger,
    input               clear,
    input       [31:0]  duration_ms,
    input       [31:0]  danger_toggle_ms,
    input       [31:0]  emergency_toggle_ms,

    output reg          active,
    output reg  [1:0]   active_level,
    output reg  [1:0]   led
);

    localparam integer CYCLES_PER_MS = CLK_FREQ_HZ / 1000;

    reg [31:0] ms_div_count;
    reg [31:0] elapsed_ms;
    reg [31:0] toggle_elapsed_ms;
    reg [31:0] duration_ms_latched;
    reg [31:0] danger_toggle_ms_latched;
    reg [31:0] emergency_toggle_ms_latched;

    wire ms_tick = (ms_div_count == CYCLES_PER_MS - 1);

    // Restart the millisecond divider for every command so the configured
    // duration is measured from the trigger edge with less than one-cycle
    // uncertainty.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ms_div_count <= 32'd0;
        else if (clear || trigger)
            ms_div_count <= 32'd0;
        else if (ms_tick)
            ms_div_count <= 32'd0;
        else
            ms_div_count <= ms_div_count + 32'd1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active                      <= 1'b0;
            active_level                <= 2'b00;
            led                         <= 2'b00;
            elapsed_ms                  <= 32'd0;
            toggle_elapsed_ms           <= 32'd0;
            duration_ms_latched          <= 32'd0;
            danger_toggle_ms_latched     <= 32'd1;
            emergency_toggle_ms_latched  <= 32'd1;
        end else if (clear) begin
            active             <= 1'b0;
            active_level       <= 2'b00;
            led                <= 2'b00;
            elapsed_ms         <= 32'd0;
            toggle_elapsed_ms  <= 32'd0;
        end else if (trigger) begin
            elapsed_ms                 <= 32'd0;
            toggle_elapsed_ms          <= 32'd0;
            duration_ms_latched         <= duration_ms;
            danger_toggle_ms_latched    <= (danger_toggle_ms == 0) ? 32'd1 : danger_toggle_ms;
            emergency_toggle_ms_latched <= (emergency_toggle_ms == 0) ? 32'd1 : emergency_toggle_ms;

            if ((level == 2'b00) || (duration_ms == 0)) begin
                active       <= 1'b0;
                active_level <= 2'b00;
                led          <= 2'b00;
            end else begin
                active       <= 1'b1;
                active_level <= level;
                case (level)
                    2'b01: led <= 2'b11;
                    2'b10: led <= 2'b01;
                    2'b11: led <= 2'b11;
                    default: led <= 2'b00;
                endcase
            end
        end else if (active && ms_tick) begin
            if ((elapsed_ms + 32'd1) >= duration_ms_latched) begin
                active             <= 1'b0;
                active_level       <= 2'b00;
                led                <= 2'b00;
                elapsed_ms         <= 32'd0;
                toggle_elapsed_ms  <= 32'd0;
            end else begin
                elapsed_ms <= elapsed_ms + 32'd1;
                case (active_level)
                    2'b01: begin
                        led               <= 2'b11;
                        toggle_elapsed_ms <= 32'd0;
                    end
                    2'b10: begin
                        if ((toggle_elapsed_ms + 32'd1) >= danger_toggle_ms_latched) begin
                            toggle_elapsed_ms <= 32'd0;
                            led <= (led == 2'b01) ? 2'b10 : 2'b01;
                        end else begin
                            toggle_elapsed_ms <= toggle_elapsed_ms + 32'd1;
                        end
                    end
                    2'b11: begin
                        if ((toggle_elapsed_ms + 32'd1) >= emergency_toggle_ms_latched) begin
                            toggle_elapsed_ms <= 32'd0;
                            led <= (led == 2'b11) ? 2'b00 : 2'b11;
                        end else begin
                            toggle_elapsed_ms <= toggle_elapsed_ms + 32'd1;
                        end
                    end
                    default: begin
                        active             <= 1'b0;
                        active_level       <= 2'b00;
                        led                <= 2'b00;
                        toggle_elapsed_ms  <= 32'd0;
                    end
                endcase
            end
        end
    end

endmodule
