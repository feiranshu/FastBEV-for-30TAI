//==============================================================================
// pulse_cross.v
//------------------------------------------------------------------------------
// Small pulse synchronizer compatible with the FPAI demo style.
//==============================================================================
`timescale 1ns/1ps

module pulse_cross (
    output reg a2,
    input      clk2,
    input      rst2,
    output     rdy1,
    input      a1,
    input      clk1,
    input      rst1
);
    reg req;
    reg ack_1, ack_2;
    reg req_1, req_2, req_3;

    assign rdy1 = (req == ack_2);

    always @(posedge clk1 or posedge rst1) begin
        if (rst1)
            req <= 1'b0;
        else if (a1 && rdy1)
            req <= ~req;
    end

    always @(posedge clk1 or posedge rst1) begin
        if (rst1) begin
            ack_1 <= 1'b0;
            ack_2 <= 1'b0;
        end else begin
            ack_1 <= req_2;
            ack_2 <= ack_1;
        end
    end

    always @(posedge clk2 or posedge rst2) begin
        if (rst2) begin
            req_1 <= 1'b0;
            req_2 <= 1'b0;
            req_3 <= 1'b0;
            a2    <= 1'b0;
        end else begin
            req_1 <= req;
            req_2 <= req_1;
            req_3 <= req_2;
            a2    <= req_2 ^ req_3;
        end
    end

endmodule

