//==============================================================================
// File Name     : pulse_cross.v
// Description   : Pulse signal cross clock domain from clk1 to clk2
//                 Reused from FPAI demo reference design
//==============================================================================

`timescale 1ns / 1ps

module pulse_cross (
  output      a2   ,
  input       clk2 ,
  input       rst2 ,

  output      rdy1 ,
  input       a1   ,
  input       clk1 ,
  input       rst1
);

  reg    req_clk1    ;
  reg    req_d1_clk2 ;
  reg    req_d2_clk2 ;

  reg    ack_clk2    ;
  reg    ack_d1_clk1 ;
  reg    ack_d2_clk1 ;
  reg    ack_d1_clk2 ;

  assign rdy1 = ~req_clk1;

always @(posedge clk1) begin
    if(rst1) begin
        ack_d1_clk1   <= 1'b0;
        ack_d2_clk1   <= 1'b0;
    end else begin
        ack_d1_clk1   <= ack_clk2   ;
        ack_d2_clk1   <= ack_d1_clk1;
    end
end

always @(posedge clk2) begin
    if(rst2) begin
        req_d1_clk2   <= 1'b0;
        req_d2_clk2   <= 1'b0;
    end else begin
        req_d1_clk2   <= req_clk1   ;
        req_d2_clk2   <= req_d1_clk2;
    end
end

always @(posedge clk1) begin
    if(rst1)
        req_clk1 <= 1'b0;
    else if(a1)
        req_clk1 <= 1'b1;
    else if(ack_d2_clk1)
        req_clk1 <= 1'b0;
end

always @(posedge clk2) begin
    if(rst2)
        ack_clk2 <= 1'b0;
    else
        ack_clk2 <= req_d2_clk2;
end

always @(posedge clk2) begin
    if(rst2)
        ack_d1_clk2 <= 1'b0;
    else
        ack_d1_clk2 <= ack_clk2;
end

assign a2 = ack_clk2 & ~ack_d1_clk2;

endmodule
