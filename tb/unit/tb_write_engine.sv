// ============================================================================
// tb_write_engine.sv  -- self-checking unit testbench for write_engine (rev5)
// ----------------------------------------------------------------------------
// Feeds a stream of beats (beat_data = running index) and checks each router
// write lands at:  bank = op%NUM_BANKS , offset = tile_base + op/NUM_BANKS + rc*SA_SIZE
// with op = k/n_rchunks (outer), rc = k%n_rchunks (inner),
// tile_base = sram_base * (SA_SIZE*KN_DEPTH*NUM_BANKS).
// Plain SV-2012. Design files: im2col_pkg.sv, write_engine.sv. INT8, SA_SIZE=4.
//
// FIXES vs prior rev: n_rchunks width now matches the DUT port (RCHUNK_W), and
// beat_data is a properly-sized (zero-extended) value, not an out-of-range slice.
// ============================================================================
`timescale 1ns/1ps
module tb_write_engine;

    localparam int SA_SIZE   = 4;
    localparam int DATA_PREC = 0;      // INT8
    localparam int NUM_BANKS = 4;
    localparam int ADDR_W    = 16;
    localparam int KN_DEPTH  = 4;
    localparam int RCHUNK_W  = 4;      // MUST match DUT port width
    localparam int LINE_W    = SA_SIZE*32;
    localparam int BANK_BITS = 2;
    localparam int OFFSET_BITS = ADDR_W - BANK_BITS;               // 14
    localparam int OUTPOS_PER_TILE = 16;                           // SA*4 (INT8)
    localparam int SRAM_TILE_STRIDE = SA_SIZE*KN_DEPTH*NUM_BANKS;  // 64

    logic                clk = 0, rst_n;
    logic                start;
    logic [RCHUNK_W-1:0] n_rchunks;     // <-- 4 bits now (was 3)
    logic [5:0]          sram_base_addr;
    logic                beat_valid;
    logic [LINE_W-1:0]   beat_data;
    logic                beat_ready;
    logic                en_a, we_a, done;
    logic [ADDR_W-1:0]   addr_a;
    logic [LINE_W-1:0]   din_a;

    integer errors = 0, tests = 0;
    integer beats_sent;        // == current write index k
    integer cur_nrc, cur_sb;

    // beat_data carries the running write index, zero-extended to LINE_W
    logic [LINE_W-1:0] beat_data_val;
    assign beat_data_val = beats_sent;   // 32-bit integer -> LINE_W (zero-extend)
    assign beat_data     = beat_data_val;

    write_engine #(
        .SA_SIZE(SA_SIZE), .DATA_PREC(DATA_PREC), .NUM_BANKS(NUM_BANKS),
        .ADDR_W(ADDR_W), .KN_DEPTH(KN_DEPTH), .RCHUNK_W(RCHUNK_W)
    ) dut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .n_rchunks(n_rchunks), .sram_base_addr(sram_base_addr),
        .beat_valid(beat_valid), .beat_data(beat_data), .beat_ready(beat_ready),
        .en_a(en_a), .we_a(we_a), .addr_a(addr_a), .din_a(din_a), .done(done)
    );

    always #5 clk = ~clk;

    function [ADDR_W-1:0] exp_addr(input integer k, input integer nrc, input integer sb);
        integer op, rc, bank, offset, tbase;
        begin
            op     = k / nrc;
            rc     = k % nrc;
            bank   = op % NUM_BANKS;
            tbase  = sb * SRAM_TILE_STRIDE;
            offset = tbase + (op / NUM_BANKS) + rc*SA_SIZE;
            exp_addr = (bank << OFFSET_BITS) + offset;
        end
    endfunction

    // check each write as it happens
    always @(posedge clk) begin
        if (en_a) begin
            if (addr_a !== exp_addr(beats_sent, cur_nrc, cur_sb) ||
                din_a  !== beat_data_val || we_a !== 1'b1) begin
                $display("  FAIL: write %0d -> addr=0x%0h din=%0d we=%b (exp addr=0x%0h din=%0d)",
                         beats_sent, addr_a, din_a, we_a,
                         exp_addr(beats_sent, cur_nrc, cur_sb), beats_sent);
                errors = errors + 1;
            end
            beats_sent = beats_sent + 1;
        end
    end

    task run_tile(input integer nrc, input integer sb);
        integer expected;
        reg     saw_done;    // 'done' is a 1-cycle pulse -> watch for it, don't sample one exact edge
        begin
            cur_nrc = nrc; cur_sb = sb;
            n_rchunks = nrc[RCHUNK_W-1:0]; sram_base_addr = sb[5:0];
            expected  = OUTPOS_PER_TILE * nrc;
            @(negedge clk); beats_sent = 0; start = 1; beat_valid = 1;
            @(negedge clk); start = 0;
            saw_done = 1'b0;
            while (beats_sent < expected) begin
                if (done) saw_done = 1'b1;
                @(negedge clk);
            end
            if (done) saw_done = 1'b1;                 // pulse at loop exit
            beat_valid = 0;
            repeat (3) begin @(negedge clk); if (done) saw_done = 1'b1; end
            tests = tests + 1;
            if (!saw_done) begin
                $display("  FAIL: tile(nrc=%0d,sb=%0d) done never asserted after %0d writes",
                         nrc, sb, expected);
                errors = errors + 1;
            end else begin
                $display("  ok  : tile(nrc=%0d,sb=%0d) wrote %0d beats, done ok", nrc, sb, expected);
            end
            repeat (2) @(negedge clk);
        end
    endtask

    initial begin
        rst_n = 0; start = 0; beat_valid = 0; n_rchunks = 1; sram_base_addr = 0;
        beats_sent = 0; cur_nrc = 1; cur_sb = 0;
        repeat (3) @(negedge clk);
        rst_n = 1;

        $display("[tb_write_engine] starting");
        run_tile(4, 0);   // full tile at base 0  (64 writes)
        run_tile(2, 1);   // 2 reduction chunks at tile slot 1
        run_tile(1, 0);   // single chunk

        if (errors == 0) $display("[tb_write_engine] PASS (%0d tiles)", tests);
        else             $display("[tb_write_engine] FAIL (%0d errors)", errors);
        $finish;
    end

    initial begin
        #500000; $display("[tb_write_engine] TIMEOUT"); $finish;
    end
endmodule
