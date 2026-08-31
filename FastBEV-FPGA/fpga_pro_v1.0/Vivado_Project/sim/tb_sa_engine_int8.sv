//==============================================================================
// tb_sa_engine_int8.sv
//   Self-checking testbench for sa_engine_int8.
//
// Coverage:
//   - signed INT8 identity sampling, including negative values.
//   - integer translation through Q16.16 C/F terms.
//   - Q0.8 fractional bilinear interpolation with signed round+saturate.
//   - signed fractional affine coefficients through the pipelined multiplier.
//   - out-of-bounds zero fill.
//   - Part3 concat destination layout: [N][Z][C/32][X][Y][C%32].
//   - deterministic read and write backpressure.
//==============================================================================
`timescale 1ns/1ps

module tb_sa_engine_int8;

    localparam int BEV_X     = 4;
    localparam int BEV_Y     = 4;
    localparam int BEV_Z     = 2;
    localparam int SA_SIZE   = BEV_X * BEV_Y * BEV_Z;
    localparam int CHANNELS  = 64;
    localparam int MEM_WORDS = 65536;

    localparam logic [31:0] SRC_BASE    = 32'h0000_1000;
    localparam logic [31:0] CONCAT_BASE = 32'h0000_8000;

    localparam logic [31:0] Q16_ONE   = 32'h0001_0000;
    localparam logic [31:0] Q16_ZERO  = 32'h0000_0000;
    localparam logic [31:0] Q16_HALF  = 32'h0000_8000;
    localparam logic [31:0] Q16_QUARTER = 32'h0000_4000;
    localparam logic [31:0] Q16_NEG_QUARTER = 32'hFFFF_C000;
    localparam logic [31:0] Q16_NEG1  = 32'hFFFF_0000;

    logic clk = 1'b0;
    always #5 clk = ~clk;

    logic rst_n = 1'b0;
    logic engine_start = 1'b0;
    logic engine_done;

    logic [31:0] xform_a00;
    logic [31:0] xform_a01;
    logic [31:0] xform_a02;
    logic [31:0] xform_a10;
    logic [31:0] xform_a11;
    logic [31:0] xform_a12;
    logic [1:0]  temporal_idx;

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

    sa_engine_int8 dut (
        .engine_start     ( engine_start       ),
        .engine_done      ( engine_done        ),
        .sa_src_addr      ( SRC_BASE           ),
        .concat_base_addr ( CONCAT_BASE        ),
        .sa_size          ( SA_SIZE[31:0]      ),
        .bev_x            ( BEV_X[7:0]         ),
        .bev_y            ( BEV_Y[7:0]         ),
        .bev_z            ( BEV_Z[7:0]         ),
        .temporal_idx     ( temporal_idx       ),
        .xform_a00        ( xform_a00          ),
        .xform_a01        ( xform_a01          ),
        .xform_a02        ( xform_a02          ),
        .xform_a10        ( xform_a10          ),
        .xform_a11        ( xform_a11          ),
        .xform_a12        ( xform_a12          ),
        .rd_addr          ( rd_addr            ),
        .rd_req           ( rd_req             ),
        .rd_grant         ( rd_grant           ),
        .rd_data          ( rd_data            ),
        .rd_data_valid    ( rd_data_valid      ),
        .rd_data_ready    ( rd_data_ready      ),
        .wr_addr          ( wr_addr            ),
        .wr_data          ( wr_data            ),
        .wr_req           ( wr_req             ),
        .wr_grant         ( wr_grant           ),
        .clk              ( clk                ),
        .rst_n            ( rst_n              )
    );

    logic [31:0] mem32 [0:MEM_WORDS-1];

    task automatic set_byte(input int byte_addr, input logic [7:0] value);
        begin
            mem32[byte_addr >> 2][(byte_addr & 3)*8 +: 8] = value;
        end
    endtask

    function automatic logic [7:0] get_byte(input int byte_addr);
        begin
            get_byte = mem32[byte_addr >> 2][(byte_addr & 3)*8 +: 8];
        end
    endfunction

    function automatic int signed src_value(input int z, input int y, input int x, input int ch);
        int v;
        begin
            v = (z * 73 + y * 37 + x * 19 + ch * 11) % 256;
            src_value = v - 128;
        end
    endfunction

    task automatic init_memory;
        int i;
        int z;
        int y;
        int x;
        int ch;
        int byte_addr;
        begin
            for (i = 0; i < MEM_WORDS; i = i + 1)
                mem32[i] = 32'd0;

            for (z = 0; z < BEV_Z; z = z + 1) begin
                for (y = 0; y < BEV_Y; y = y + 1) begin
                    for (x = 0; x < BEV_X; x = x + 1) begin
                        for (ch = 0; ch < CHANNELS; ch = ch + 1) begin
                            byte_addr = SRC_BASE + (((z * BEV_Y + y) * BEV_X + x) * 64) + ch;
                            set_byte(byte_addr, src_value(z, y, x, ch) & 8'hFF);
                        end
                    end
                end
            end
        end
    endtask

    function automatic int part3_addr(input int z, input int x, input int y, input int c);
        int cg;
        int lane;
        begin
            cg = c / 32;
            lane = c % 32;
            part3_addr = CONCAT_BASE + ((((z * 8 + cg) * BEV_X + x) * BEV_Y + y) * 32) + lane;
        end
    endfunction

    task automatic fill_concat(input logic [7:0] value);
        int z;
        int y;
        int x;
        int c;
        begin
            for (z = 0; z < BEV_Z; z = z + 1)
                for (x = 0; x < BEV_X; x = x + 1)
                    for (y = 0; y < BEV_Y; y = y + 1)
                        for (c = 0; c < 256; c = c + 1)
                            set_byte(part3_addr(z, x, y, c), value);
        end
    endtask

    int cycle_cnt;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cycle_cnt <= 0;
        else
            cycle_cnt <= cycle_cnt + 1;
    end

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
                        rd_delay_cnt  <= 1 + (cycle_cnt % 5);
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

    logic awready_model;
    assign wr_grant = wr_req && awready_model;

    integer wi;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            awready_model <= 1'b0;
        end else begin
            awready_model <= (cycle_cnt[2:0] != 3'd2) && (cycle_cnt[2:0] != 3'd6);
            if (wr_req && awready_model) begin
                for (wi = 0; wi < 16; wi = wi + 1)
                    mem32[(wr_addr >> 2) + wi] <= wr_data[wi*32 +: 32];
            end
        end
    end

    task automatic reset_dut;
        begin
            rst_n = 1'b0;
            engine_start = 1'b0;
            repeat (6) @(posedge clk);
            rst_n = 1'b1;
            repeat (4) @(posedge clk);
        end
    endtask

    task automatic run_case(
        input [31:0] a00,
        input [31:0] a01,
        input [31:0] a02,
        input [31:0] a10,
        input [31:0] a11,
        input [31:0] a12,
        input [1:0]  tidx
    );
        int timeout;
        begin
            xform_a00    = a00;
            xform_a01    = a01;
            xform_a02    = a02;
            xform_a10    = a10;
            xform_a11    = a11;
            xform_a12    = a12;
            temporal_idx = tidx;

            @(posedge clk);
            engine_start <= 1'b1;
            @(posedge clk);
            engine_start <= 1'b0;

            timeout = 0;
            while (!engine_done && timeout < 20000) begin
                @(posedge clk);
                timeout++;
            end
            if (!engine_done) begin
                $display("FAIL: timeout tidx=%0d", tidx);
                $finish;
            end
            repeat (3) @(posedge clk);
        end
    endtask

    function automatic int signed round_shift16(input int signed acc);
        begin
            if (acc >= 0)
                round_shift16 = (acc + 32768) >>> 16;
            else
                round_shift16 = -(((-acc) + 32768) >>> 16);
        end
    endfunction

    function automatic int signed sat_i8(input int signed v);
        begin
            if (v > 127)
                sat_i8 = 127;
            else if (v < -128)
                sat_i8 = -128;
            else
                sat_i8 = v;
        end
    endfunction

    function automatic int signed sample_expected(
        input int out_y,
        input int out_x,
        input int out_z,
        input int ch,
        input int signed u_q16,
        input int signed v_q16
    );
        int signed u_int;
        int signed v_int;
        int frac_x;
        int frac_y;
        int wx0;
        int wx1;
        int wy0;
        int wy1;
        int w00;
        int w01;
        int w10;
        int w11;
        int signed acc;
        int signed rounded;
        begin
            u_int = u_q16 >>> 16;
            v_int = v_q16 >>> 16;
            if (u_int < 0 || v_int < 0 || u_int >= (BEV_X - 1) || v_int >= (BEV_Y - 1)) begin
                sample_expected = 0;
            end else begin
                frac_x = (u_q16 >> 8) & 8'hFF;
                frac_y = (v_q16 >> 8) & 8'hFF;
                wx1 = frac_x;
                wx0 = 256 - frac_x;
                wy1 = frac_y;
                wy0 = 256 - frac_y;
                w00 = wx0 * wy0;
                w01 = wx1 * wy0;
                w10 = wx0 * wy1;
                w11 = wx1 * wy1;
                acc = src_value(out_z, v_int,     u_int,     ch) * w00 +
                      src_value(out_z, v_int,     u_int + 1, ch) * w01 +
                      src_value(out_z, v_int + 1, u_int,     ch) * w10 +
                      src_value(out_z, v_int + 1, u_int + 1, ch) * w11;
                rounded = round_shift16(acc);
                sample_expected = sat_i8(rounded);
            end
        end
    endfunction

    function automatic int signed expected_for_case(
        input int out_y,
        input int out_x,
        input int out_z,
        input int ch,
        input [31:0] a00,
        input [31:0] a01,
        input [31:0] a02,
        input [31:0] a10,
        input [31:0] a11,
        input [31:0] a12
    );
        int signed u_q16;
        int signed v_q16;
        begin
            u_q16 = $signed(a00) * out_x + $signed(a01) * out_y + $signed(a02);
            v_q16 = $signed(a10) * out_x + $signed(a11) * out_y + $signed(a12);
            expected_for_case = sample_expected(out_y, out_x, out_z, ch, u_q16, v_q16);
        end
    endfunction

    task automatic verify_case(
        input string name,
        input [31:0] a00,
        input [31:0] a01,
        input [31:0] a02,
        input [31:0] a10,
        input [31:0] a11,
        input [31:0] a12,
        input [1:0]  tidx,
        output int errors
    );
        int z;
        int y;
        int x;
        int c;
        int ch;
        int t;
        int signed exp;
        byte signed actual;
        begin
            errors = 0;
            for (z = 0; z < BEV_Z; z = z + 1) begin
                for (x = 0; x < BEV_X; x = x + 1) begin
                    for (y = 0; y < BEV_Y; y = y + 1) begin
                        for (c = 0; c < 256; c = c + 1) begin
                            t = c / 64;
                            ch = c % 64;
                            actual = get_byte(part3_addr(z, x, y, c));
                            if (t == tidx) begin
                                exp = expected_for_case(y, x, z, ch, a00, a01, a02, a10, a11, a12);
                                if (actual !== exp) begin
                                    if (errors < 16)
                                        $display("%s mismatch z=%0d y=%0d x=%0d c=%0d actual=%0d expected=%0d",
                                                 name, z, y, x, c, actual, exp);
                                    errors++;
                                end
                            end else begin
                                if (actual !== 8'hA5) begin
                                    if (errors < 16)
                                        $display("%s preserve mismatch z=%0d y=%0d x=%0d c=%0d actual=%02x",
                                                 name, z, y, x, c, actual);
                                    errors++;
                                end
                            end
                        end
                    end
                end
            end
        end
    endtask

    int err_identity;
    int err_translate;
    int err_frac;
    int err_affine_split;
    int err_oob;
    int total_errors;

    initial begin
        init_memory();

        fill_concat(8'hA5);
        reset_dut();
        run_case(Q16_ONE, Q16_ZERO, Q16_ZERO, Q16_ZERO, Q16_ONE, Q16_ZERO, 2'd0);
        verify_case("identity", Q16_ONE, Q16_ZERO, Q16_ZERO,
                    Q16_ZERO, Q16_ONE, Q16_ZERO, 2'd0, err_identity);
        if (err_identity == 0)
            $display("*** PASS *** sa_engine_int8 identity signed");
        else
            $display("*** FAIL *** sa_engine_int8 identity errors=%0d", err_identity);

        fill_concat(8'hA5);
        reset_dut();
        run_case(Q16_ONE, Q16_ZERO, Q16_ONE, Q16_ZERO, Q16_ONE, Q16_ZERO, 2'd1);
        verify_case("translate_x_plus1", Q16_ONE, Q16_ZERO, Q16_ONE,
                    Q16_ZERO, Q16_ONE, Q16_ZERO, 2'd1, err_translate);
        if (err_translate == 0)
            $display("*** PASS *** sa_engine_int8 integer translation");
        else
            $display("*** FAIL *** sa_engine_int8 translation errors=%0d", err_translate);

        fill_concat(8'hA5);
        reset_dut();
        run_case(Q16_ONE, Q16_ZERO, Q16_HALF, Q16_ZERO, Q16_ONE, Q16_HALF, 2'd2);
        verify_case("half_pixel", Q16_ONE, Q16_ZERO, Q16_HALF,
                    Q16_ZERO, Q16_ONE, Q16_HALF, 2'd2, err_frac);
        if (err_frac == 0)
            $display("*** PASS *** sa_engine_int8 fractional bilinear");
        else
            $display("*** FAIL *** sa_engine_int8 fractional errors=%0d", err_frac);

        fill_concat(8'hA5);
        reset_dut();
        run_case(Q16_HALF, Q16_QUARTER, Q16_ZERO,
                 Q16_NEG_QUARTER, Q16_ONE, Q16_HALF, 2'd2);
        verify_case("signed_fractional_affine",
                    Q16_HALF, Q16_QUARTER, Q16_ZERO,
                    Q16_NEG_QUARTER, Q16_ONE, Q16_HALF,
                    2'd2, err_affine_split);
        if (err_affine_split == 0)
            $display("*** PASS *** sa_engine_int8 signed fractional affine pipeline");
        else
            $display("*** FAIL *** sa_engine_int8 signed fractional affine errors=%0d",
                     err_affine_split);

        fill_concat(8'hA5);
        reset_dut();
        run_case(Q16_ONE, Q16_ZERO, Q16_NEG1, Q16_ZERO, Q16_ONE, Q16_ZERO, 2'd0);
        verify_case("out_of_bounds", Q16_ONE, Q16_ZERO, Q16_NEG1,
                    Q16_ZERO, Q16_ONE, Q16_ZERO, 2'd0, err_oob);
        if (err_oob == 0)
            $display("*** PASS *** sa_engine_int8 out-of-bounds zero");
        else
            $display("*** FAIL *** sa_engine_int8 oob errors=%0d", err_oob);

        total_errors = err_identity + err_translate + err_frac +
                       err_affine_split + err_oob;
        if (total_errors == 0)
            $display("*** PASS *** sa_engine_int8 all checks");
        else
            $display("*** FAIL *** sa_engine_int8 total_errors=%0d", total_errors);

        $finish;
    end

endmodule
