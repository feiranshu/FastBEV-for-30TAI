//==============================================================================
// tb_quant_engine.sv
//   Self-checking testbench for quant_engine.
//
// Coverage:
//   - 16-lane FP32 beat extraction.
//   - 4 FP32 beats packed into one 512-bit INT8 beat.
//   - ch0 is the lowest-address byte.
//   - deterministic read/write backpressure.
//   - final flush and single done pulse.
//==============================================================================
`timescale 1ns/1ps

module tb_quant_engine;

    localparam int TOTAL_PIXELS      = 9;
    localparam int MEM_WORDS         = 32768;
    localparam logic [31:0] FP32_BASE = 32'h0000_0000;
    localparam logic [31:0] INT8_BASE = 32'h0001_0000;
    localparam real SCALE            = 0.06905783;

    logic clk = 1'b0;
    always #5 clk = ~clk;

    logic rst_n = 1'b0;
    logic engine_start = 1'b0;
    logic engine_done;

    logic [31:0] rd_addr;
    logic        rd_req;
    logic        rd_grant;
    logic [511:0] rd_data;
    logic        rd_data_valid;
    logic        rd_data_ready;

    logic [31:0] wr_addr;
    logic [511:0] wr_data;
    logic        wr_req;
    logic        wr_grant;

    quant_engine #(
        .SHIFT_BASE(156)
    ) dut (
        .engine_start  ( engine_start       ),
        .engine_done   ( engine_done        ),
        .fp32_base_addr( FP32_BASE[31:0]    ),
        .int8_base_addr( INT8_BASE[31:0]    ),
        .total_pixels  ( TOTAL_PIXELS[31:0] ),
        .rd_addr       ( rd_addr            ),
        .rd_req        ( rd_req             ),
        .rd_grant      ( rd_grant           ),
        .rd_data       ( rd_data            ),
        .rd_data_valid ( rd_data_valid      ),
        .rd_data_ready ( rd_data_ready      ),
        .wr_addr       ( wr_addr            ),
        .wr_data       ( wr_data            ),
        .wr_req        ( wr_req             ),
        .wr_grant      ( wr_grant           ),
        .clk           ( clk                ),
        .rst_n         ( rst_n              )
    );

    logic [31:0] mem32 [0:MEM_WORDS-1];
    int gold [0:TOTAL_PIXELS*64-1];

    function automatic int abs_int(input int a);
        begin
            abs_int = (a < 0) ? -a : a;
        end
    endfunction

    function automatic int golden(input logic [31:0] b);
        logic [7:0] e;
        logic [22:0] m;
        shortreal sf;
        real v;
        real qf;
        int q;
        begin
            e = b[30:23];
            m = b[22:0];
            if (e == 8'hFF) begin
                if (m != 0)
                    golden = 0;
                else
                    golden = b[31] ? -128 : 127;
            end else begin
                sf = $bitstoshortreal(b);
                v  = sf;
                qf = v / SCALE;
                if (qf >= 0.0)
                    q = $rtoi(qf + 0.5);
                else
                    q = $rtoi(qf - 0.5);
                if (q > 127)
                    q = 127;
                if (q < -128)
                    q = -128;
                golden = q;
            end
        end
    endfunction

    function automatic logic [31:0] make_fp32_bits(input int pix, input int ch);
        int q_target;
        real val_real;
        shortreal val_short;
        begin
            q_target = ((pix * 37 + ch * 13) % 255) - 128;
            if ((ch % 29) == 0)
                q_target = 127;
            if ((ch % 31) == 0)
                q_target = -128;
            val_real  = q_target * SCALE;
            val_short = val_real;
            make_fp32_bits = $shortrealtobits(val_short);
        end
    endfunction

    integer p;
    integer c;
    integer word_idx;
    logic [31:0] bits;
    initial begin
        for (word_idx = 0; word_idx < MEM_WORDS; word_idx = word_idx + 1)
            mem32[word_idx] = 32'd0;

        for (p = 0; p < TOTAL_PIXELS; p = p + 1) begin
            for (c = 0; c < 64; c = c + 1) begin
                bits = make_fp32_bits(p, c);
                mem32[(FP32_BASE + p*256 + c*4) >> 2] = bits;
                gold[p*64 + c] = golden(bits);
            end
        end
    end

    int cycle_cnt = 0;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cycle_cnt <= 0;
        else
            cycle_cnt <= cycle_cnt + 1;
    end

    // Deterministic single-outstanding read model with address backpressure.
    typedef enum logic [1:0] {RD_IDLE, RD_DELAY, RD_RESP} rd_state_t;
    rd_state_t rd_state;
    logic [31:0] rd_addr_buf;
    int rd_delay_cnt;
    logic arready_model;

    assign rd_grant = rd_req && arready_model;

    integer ri;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_state      <= RD_IDLE;
            arready_model <= 1'b0;
            rd_data_valid <= 1'b0;
            rd_data       <= 512'd0;
            rd_addr_buf   <= 32'd0;
            rd_delay_cnt  <= 0;
        end else begin
            case (rd_state)
                RD_IDLE: begin
                    arready_model <= (cycle_cnt[1:0] != 2'b01);
                    rd_data_valid <= 1'b0;
                    if (rd_req && arready_model) begin
                        rd_addr_buf   <= rd_addr;
                        rd_delay_cnt  <= 1 + (cycle_cnt % 3);
                        arready_model <= 1'b0;
                        rd_state      <= RD_DELAY;
                    end
                end
                RD_DELAY: begin
                    if (rd_delay_cnt == 0) begin
                        for (ri = 0; ri < 16; ri = ri + 1)
                            rd_data[ri*32 +: 32] <= mem32[(rd_addr_buf >> 2) + ri];
                        rd_data_valid <= 1'b1;
                        rd_state      <= RD_RESP;
                    end else begin
                        rd_delay_cnt <= rd_delay_cnt - 1;
                    end
                end
                RD_RESP: begin
                    if (rd_data_valid && rd_data_ready) begin
                        rd_data_valid <= 1'b0;
                        rd_state      <= RD_IDLE;
                    end
                end
                default: rd_state <= RD_IDLE;
            endcase
        end
    end

    // Deterministic write backpressure.
    logic awready_model;
    assign wr_grant = wr_req && awready_model;

    integer wi;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            awready_model <= 1'b0;
        end else begin
            awready_model <= (cycle_cnt[2:0] != 3'd3) && (cycle_cnt[2:0] != 3'd4);
            if (wr_req && awready_model) begin
                for (wi = 0; wi < 16; wi = wi + 1)
                    mem32[(wr_addr >> 2) + wi] <= wr_data[wi*32 +: 32];
            end
        end
    end

    int timeout;
    int errors;
    int diff;
    byte signed actual;

    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1'b1;

        @(posedge clk);
        engine_start <= 1'b1;
        @(posedge clk);
        engine_start <= 1'b0;

        timeout = 0;
        while (!engine_done && timeout < 5000) begin
            @(posedge clk);
            timeout++;
        end

        if (!engine_done) begin
            $display("FAIL: timeout waiting for quant_engine done");
            $finish;
        end

        repeat (2) @(posedge clk);

        errors = 0;
        for (p = 0; p < TOTAL_PIXELS; p = p + 1) begin
            for (c = 0; c < 64; c = c + 1) begin
                actual = mem32[(INT8_BASE + p*64 + (c/4)*4) >> 2][(c%4)*8 +: 8];
                diff = abs_int(actual - gold[p*64 + c]);
                if (diff > 1) begin
                    if (errors < 16)
                        $display("Mismatch pixel=%0d ch=%0d actual=%0d golden=%0d diff=%0d",
                                 p, c, actual, gold[p*64 + c], diff);
                    errors++;
                end
            end
        end

        if (errors == 0)
            $display("*** PASS *** quant_engine lane order, pack, backpressure, flush");
        else
            $display("*** FAIL *** quant_engine errors=%0d", errors);

        $finish;
    end

endmodule
