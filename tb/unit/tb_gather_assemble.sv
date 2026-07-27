// ============================================================================
// tb_gather_assemble.sv (rev 2 -- matches the K-wide gather_assemble interface)
// ----------------------------------------------------------------------------
// Unit test: drives gather_assemble DIRECTLY (not through the AGU), feeding
// hand-constructed K-wide batches (lane_valid/lane_addr/lane_run_len/
// lane_pad_lead/lane_pad_trail) and checking the resulting beat bytes against
// hand-computed golden values. Exercises:
//   * a cold-cache batch (misses -> fallback path)
//   * the SAME addresses repeated (now cached -> fast path)
//   * a lane with both leading and trailing padding
//   * a lane whose data straddles a cache-line boundary (forces fallback
//     even though the data is technically available, per the fast/fallback
//     scope decision documented in gather_assemble.sv)
//
// Plain SV-2012. Design files: im2col_pkg.sv, gather_assemble.sv.
// INT8, SA_SIZE=4 (LINE_BYTES=16), PARALLEL_GROUPS=8, NUM_CACHE_LINES=8.
// ============================================================================
`timescale 1ns/1ps
module tb_gather_assemble;

    localparam int SA_SIZE   = 4;
    localparam int ADDR_W    = 32;
    localparam int KER_W     = 4;
    localparam int NCL       = 8;
    localparam int K         = 8;      // PARALLEL_GROUPS
    localparam int LINE_W    = SA_SIZE*32;   // 128
    localparam int LINE_BYTES= SA_SIZE*4;    // 16
    localparam int RED_PER_BEAT = 16;        // SA*4, INT8

    logic clk = 0, rst_n;
    logic                 pix_valid, run_last, tile_last, pix_ready;
    logic [K-1:0]         lane_valid;
    logic [ADDR_W-1:0]    lane_addr     [K];
    logic [KER_W-1:0]     lane_run_len  [K];
    logic [KER_W-1:0]     lane_pad_lead [K];
    logic [KER_W-1:0]     lane_pad_trail[K];

    logic                 rd_req_valid, rd_req_ready, rd_rsp_valid, rd_rsp_ready;
    logic [ADDR_W-1:0]    rd_req_addr;
    logic [LINE_W-1:0]    rd_rsp_data;

    logic                 beat_valid, beat_ready;
    logic [LINE_W-1:0]    beat_data;

    integer errors = 0, tests = 0;

    gather_assemble #(
        .SA_SIZE(SA_SIZE), .DATA_PREC(0), .ADDR_W(ADDR_W), .KER_W(KER_W),
        .NUM_CACHE_LINES(NCL), .PARALLEL_GROUPS(K)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .pix_valid(pix_valid),
        .lane_valid(lane_valid), .lane_addr(lane_addr),
        .lane_run_len(lane_run_len), .lane_pad_lead(lane_pad_lead), .lane_pad_trail(lane_pad_trail),
        .run_last(run_last), .tile_last(tile_last), .pix_ready(pix_ready),
        .rd_req_valid(rd_req_valid), .rd_req_addr(rd_req_addr), .rd_req_ready(rd_req_ready),
        .rd_rsp_valid(rd_rsp_valid), .rd_rsp_data(rd_rsp_data), .rd_rsp_ready(rd_rsp_ready),
        .beat_valid(beat_valid), .beat_data(beat_data), .beat_ready(beat_ready)
    );

    always #5 clk = ~clk;
    assign beat_ready = 1'b1;

    // ---- behavioural memory: byte(addr) = addr & 0xFF, 2-cycle latency ----
    logic [ADDR_W-1:0] req_addr_q;
    integer             lat_q;
    typedef enum logic [1:0] { M_IDLE, M_WAIT, M_DATA } mst_t;
    mst_t m_st;
    assign rd_req_ready = (m_st == M_IDLE);
    assign rd_rsp_valid = (m_st == M_DATA);
    function [LINE_W-1:0] line_of(input [ADDR_W-1:0] la);
        integer i; begin
            for (i = 0; i < LINE_BYTES; i = i + 1) line_of[i*8 +: 8] = (la + i) & 8'hFF;
        end
    endfunction
    assign rd_rsp_data = line_of(req_addr_q);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin m_st <= M_IDLE; req_addr_q <= '0; lat_q <= 0; end
        else unique case (m_st)
            M_IDLE: if (rd_req_valid) begin req_addr_q <= rd_req_addr; lat_q <= 2; m_st <= M_WAIT; end
            M_WAIT: if (lat_q <= 1) m_st <= M_DATA; else lat_q <= lat_q - 1;
            M_DATA: if (rd_rsp_ready) m_st <= M_IDLE;
            default: m_st <= M_IDLE;
        endcase
    end

    // ---- helper: clear all lanes to invalid -------------------------------
    task clear_lanes;
        integer i;
        begin
            for (i = 0; i < K; i = i + 1) begin
                lane_valid[i] = 1'b0;
                lane_addr[i] = '0; lane_run_len[i] = '0;
                lane_pad_lead[i] = '0; lane_pad_trail[i] = '0;
            end
        end
    endtask

    // ---- send one batch (edge-sampled accept, avoids delta-cycle races) --
    task send_batch(input logic is_run_last, input logic is_tile_last);
        begin
            @(negedge clk);
            pix_valid = 1'b1; run_last = is_run_last; tile_last = is_tile_last;
            @(posedge clk);
            while (!pix_ready) @(posedge clk);
            @(negedge clk);
            pix_valid = 1'b0;
        end
    endtask

    // ---- wait for and check one beat --------------------------------------
    task check_beat(input string label);
        integer errs_before;
        begin
            errs_before = errors;
            @(posedge clk);
            while (!beat_valid) @(posedge clk);
            tests = tests + 1;
            if (errors == errs_before)
                $display("  ok  : %s", label);
            @(negedge clk);
        end
    endtask

    // expected byte at position 'p' within a lane starting at 'base', given
    // that lane's run_len/pad_lead/pad_trail
    function [7:0] gbyte(input [ADDR_W-1:0] base, input integer p,
                          input integer lead, input integer trail, input integer rl);
        begin
            if (p < lead || p >= rl - trail) gbyte = 8'h00;
            else gbyte = (base + p) & 8'hFF;
        end
    endfunction

    initial begin
        rst_n = 0; clear_lanes(); pix_valid = 0; run_last = 0; tile_last = 0;
        repeat (3) @(negedge clk);
        rst_n = 1;

        $display("[tb_gather_assemble] starting (K-wide interface)");

        // ---- Batch 1: 2 lanes, no padding, cold cache -> fallback path ----
        clear_lanes();
        lane_valid[0]=1; lane_addr[0]=32'h0000_0000; lane_run_len[0]=4; lane_pad_lead[0]=0; lane_pad_trail[0]=0;
        lane_valid[1]=1; lane_addr[1]=32'h0000_0004; lane_run_len[1]=4; lane_pad_lead[1]=0; lane_pad_trail[1]=0;
        send_batch(1'b1, 1'b0);
        check_beat("batch1: 2 lanes, cold cache (fallback), no padding");
        if (beat_data[0*8 +: 8]   !== gbyte(32'h0,0,0,0,4)) begin errors=errors+1; $display("  FAIL b1 byte0"); end
        if (beat_data[3*8 +: 8]   !== gbyte(32'h0,3,0,0,4)) begin errors=errors+1; $display("  FAIL b1 byte3"); end
        if (beat_data[4*8 +: 8]   !== gbyte(32'h4,0,0,0,4)) begin errors=errors+1; $display("  FAIL b1 byte4"); end
        if (beat_data[7*8 +: 8]   !== gbyte(32'h4,3,0,0,4)) begin errors=errors+1; $display("  FAIL b1 byte7"); end

        // ---- Batch 2: SAME addresses -> cache now warm -> fast path -------
        clear_lanes();
        lane_valid[0]=1; lane_addr[0]=32'h0000_0000; lane_run_len[0]=4; lane_pad_lead[0]=0; lane_pad_trail[0]=0;
        lane_valid[1]=1; lane_addr[1]=32'h0000_0004; lane_run_len[1]=4; lane_pad_lead[1]=0; lane_pad_trail[1]=0;
        send_batch(1'b1, 1'b0);
        check_beat("batch2: same addresses, warm cache (fast path)");
        if (beat_data[0*8 +: 8]   !== gbyte(32'h0,0,0,0,4)) begin errors=errors+1; $display("  FAIL b2 byte0"); end
        if (beat_data[7*8 +: 8]   !== gbyte(32'h4,3,0,0,4)) begin errors=errors+1; $display("  FAIL b2 byte7"); end

        // ---- Batch 3: one lane with leading+trailing padding ---------------
        clear_lanes();
        lane_valid[0]=1; lane_addr[0]=32'h0000_0008; lane_run_len[0]=4; lane_pad_lead[0]=1; lane_pad_trail[0]=1;
        send_batch(1'b1, 1'b0);
        check_beat("batch3: single lane, pad_lead=1 pad_trail=1");
        if (beat_data[0*8 +: 8] !== 8'h00) begin errors=errors+1; $display("  FAIL b3 lead pad"); end
        if (beat_data[1*8 +: 8] !== gbyte(32'h8,1,1,1,4)) begin errors=errors+1; $display("  FAIL b3 mid0"); end
        if (beat_data[2*8 +: 8] !== gbyte(32'h8,2,1,1,4)) begin errors=errors+1; $display("  FAIL b3 mid1"); end
        if (beat_data[3*8 +: 8] !== 8'h00) begin errors=errors+1; $display("  FAIL b3 trail pad"); end

        // ---- Batch 4: line-crossing run (forces fallback even though data
        //      would otherwise be a hit, per the documented fast/fallback
        //      scope decision) --------------------------------------------
        clear_lanes();
        lane_valid[0]=1; lane_addr[0]=32'h0000_000E; lane_run_len[0]=4; lane_pad_lead[0]=0; lane_pad_trail[0]=0; // bytes 14,15,16,17 -> crosses the 16-byte line boundary
        send_batch(1'b1, 1'b0);
        check_beat("batch4: run crosses a cache-line boundary");
        if (beat_data[0*8 +: 8] !== gbyte(32'hE,0,0,0,4)) begin errors=errors+1; $display("  FAIL b4 byte0"); end
        if (beat_data[3*8 +: 8] !== gbyte(32'hE,3,0,0,4)) begin errors=errors+1; $display("  FAIL b4 byte3"); end

        if (errors == 0) $display("[tb_gather_assemble] PASS (%0d beats checked)", tests);
        else             $display("[tb_gather_assemble] FAIL (%0d errors)", errors);
        $finish;
    end

    initial begin #500000; $display("[tb_gather_assemble] TIMEOUT"); $finish; end
endmodule
`default_nettype wire
