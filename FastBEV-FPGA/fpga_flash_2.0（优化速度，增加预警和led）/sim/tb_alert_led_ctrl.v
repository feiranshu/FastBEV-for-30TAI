`timescale 1ns/1ps

module tb_alert_led_ctrl;

    localparam integer CLK_FREQ_HZ = 4000;
    localparam integer CYCLES_PER_MS = CLK_FREQ_HZ / 1000;

    reg         clk;
    reg         rst_n;
    reg [1:0]   level;
    reg         trigger;
    reg         clear;
    reg [31:0]  duration_ms;
    reg [31:0]  danger_toggle_ms;
    reg [31:0]  emergency_toggle_ms;

    wire        active;
    wire [1:0]  active_level;
    wire [1:0]  led;

    alert_led_ctrl #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ)
    ) dut (
        .clk                 (clk),
        .rst_n               (rst_n),
        .level               (level),
        .trigger             (trigger),
        .clear               (clear),
        .duration_ms         (duration_ms),
        .danger_toggle_ms    (danger_toggle_ms),
        .emergency_toggle_ms (emergency_toggle_ms),
        .active              (active),
        .active_level        (active_level),
        .led                 (led)
    );

    always #5 clk = ~clk;

    task fail;
        input [8*100-1:0] message;
        begin
            $display("TEST_FAIL: %0s", message);
            $finish;
        end
    endtask

    task wait_cycles;
        input integer count;
        begin
            repeat (count) @(posedge clk);
            #1;
        end
    endtask

    task issue_trigger;
        input [1:0] trigger_level;
        input [31:0] trigger_duration_ms;
        input [31:0] trigger_danger_toggle_ms;
        input [31:0] trigger_emergency_toggle_ms;
        begin
            @(negedge clk);
            level               = trigger_level;
            duration_ms         = trigger_duration_ms;
            danger_toggle_ms    = trigger_danger_toggle_ms;
            emergency_toggle_ms = trigger_emergency_toggle_ms;
            trigger             = 1'b1;
            @(negedge clk);
            trigger             = 1'b0;
            #1;
        end
    endtask

    initial begin
        clk                 = 1'b0;
        rst_n               = 1'b0;
        level               = 2'b00;
        trigger             = 1'b0;
        clear               = 1'b0;
        duration_ms         = 32'd0;
        danger_toggle_ms    = 32'd0;
        emergency_toggle_ms = 32'd0;

        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);
        #1;
        if (active !== 1'b0 || active_level !== 2'b00 || led !== 2'b00)
            fail("reset must clear the alert");

        // Caution: both LEDs stay on until the duration expires.
        issue_trigger(2'b01, 32'd4, 32'd2, 32'd1);
        if (active !== 1'b1 || active_level !== 2'b01 || led !== 2'b11)
            fail("caution must start with both LEDs on");
        wait_cycles(CYCLES_PER_MS * 3);
        if (active !== 1'b1 || led !== 2'b11)
            fail("caution must remain active before timeout");
        wait_cycles(CYCLES_PER_MS);
        if (active !== 1'b0 || active_level !== 2'b00 || led !== 2'b00)
            fail("caution must turn off at timeout");

        // Danger: configuration is latched on trigger and LEDs alternate.
        issue_trigger(2'b10, 32'd8, 32'd2, 32'd1);
        danger_toggle_ms = 32'd1;
        if (led !== 2'b01)
            fail("danger must start at 01");
        wait_cycles(CYCLES_PER_MS);
        if (led !== 2'b01)
            fail("danger must use the latched two-ms period");
        wait_cycles(CYCLES_PER_MS);
        if (led !== 2'b10)
            fail("danger must alternate to 10");
        wait_cycles(CYCLES_PER_MS * 2);
        if (led !== 2'b01)
            fail("danger must alternate back to 01");

        // Clear stops an active alert immediately.
        @(negedge clk);
        clear = 1'b1;
        @(negedge clk);
        clear = 1'b0;
        #1;
        if (active !== 1'b0 || active_level !== 2'b00 || led !== 2'b00)
            fail("clear must stop danger immediately");

        // Emergency: synchronous 11/00 blinking.
        issue_trigger(2'b11, 32'd6, 32'd2, 32'd1);
        if (led !== 2'b11)
            fail("emergency must start at 11");
        wait_cycles(CYCLES_PER_MS);
        if (led !== 2'b00)
            fail("emergency must toggle to 00");
        wait_cycles(CYCLES_PER_MS);
        if (led !== 2'b11)
            fail("emergency must toggle back to 11");

        // Re-trigger restarts the complete duration.
        issue_trigger(2'b01, 32'd3, 32'd2, 32'd1);
        wait_cycles(CYCLES_PER_MS * 2);
        issue_trigger(2'b01, 32'd3, 32'd2, 32'd1);
        wait_cycles(CYCLES_PER_MS * 2);
        if (active !== 1'b1 || led !== 2'b11)
            fail("re-trigger must restart the duration");
        wait_cycles(CYCLES_PER_MS);
        if (active !== 1'b0 || led !== 2'b00)
            fail("re-triggered caution must eventually expire");

        // clear has priority when both command pulses arrive together.
        @(negedge clk);
        level       = 2'b11;
        duration_ms = 32'd5;
        trigger     = 1'b1;
        clear       = 1'b1;
        @(negedge clk);
        trigger     = 1'b0;
        clear       = 1'b0;
        #1;
        if (active !== 1'b0 || led !== 2'b00)
            fail("clear must have priority over trigger");

        // Level zero and zero duration are defensive no-op commands.
        issue_trigger(2'b00, 32'd5, 32'd2, 32'd1);
        if (active !== 1'b0 || led !== 2'b00)
            fail("level zero trigger must remain off");
        issue_trigger(2'b01, 32'd0, 32'd2, 32'd1);
        if (active !== 1'b0 || led !== 2'b00)
            fail("zero-duration trigger must remain off");

        $display("TEST_PASS: tb_alert_led_ctrl");
        $finish;
    end

endmodule
