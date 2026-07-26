// ============================================================================
// agu.sv  -- Address Generation Unit (rev 4: K-wide parallel batch output)
// ----------------------------------------------------------------------------
// MAJOR CHANGE vs rev 3: rev 3 emitted one RUN (up to S bytes) per cycle,
// which fixed the byte-level bottleneck but left cycles/beat still scaling
// with SA_SIZE, because groups-per-beat = RED_PER_BEAT/S still grows with
// SA_SIZE (RED_PER_BEAT = SA_SIZE*4). To make cycles/beat roughly
// SA_SIZE-independent, the AGU now computes up to PARALLEL_GROUPS runs
// PER CYCLE (a parameter, default 8) instead of one.
//
// MECHANISM: PARALLEL_GROUPS copies of rev 3's single-run-step formula are
// chained COMBINATIONALLY within one clock edge (lane i's starting state is
// lane i-1's resulting state, all computed in the same cycle, not across
// multiple cycles). If the beat completes partway through the chain (S does
// not have to divide RED_PER_BEAT evenly, same as rev 3), the remaining
// lanes in that cycle are simply NO-OPs (they propagate state unchanged) --
// so the committed state after PARALLEL_GROUPS lanes is always correct
// whether the beat finished at lane 0 or lane PARALLEL_GROUPS-1. If a beat
// needs MORE than PARALLEL_GROUPS runs, the AGU issues additional batches on
// subsequent cycles, continuing from where it left off (same idea as rev 3
// resuming a clipped run, just at batch granularity).
//
// This is a mechanical, unrolled repetition of rev 3's *already-verified*
// per-run formula -- not new address math. The chaining mechanism itself was
// verified separately: a Python model of this exact "always run K lanes,
// dead lanes are no-ops, commit lane K's state unconditionally" scheme was
// checked against the proven sequential (one-run-at-a-time) reference across
// 20,000 randomized configurations (S, R, SA_SIZE, K, arbitrary starting
// state) with zero mismatches, for both the emitted run sequence and the
// final state. See the design notes; the RTL below is a direct translation
// of that verified model.
//
// Interface change: pix_addr/run_len/pad_lead/pad_trail/run_last are now
// PARALLEL_GROUPS-wide arrays (one slot per lane) plus a per-lane 'lane_valid'
// (fewer than PARALLEL_GROUPS lanes are valid near a beat boundary). The
// handshake (pix_valid/pix_ready) is now per BATCH, not per run: gather must
// consume the WHOLE batch (all valid lanes) before pix_ready is reasserted
// for the next one. tile_last is asserted alongside the batch that completes
// the tile (same meaning as before, now at batch granularity).
//
// STATUS: substantial rewrite (3rd major AGU revision this project). The
// chaining ALGORITHM is verified in Python as described above; the RTL
// TRANSLATION itself has not been simulated (no simulator available here).
// The accompanying tb_agu.sv revalidates this exact RTL against the same
// per-element golden formula used throughout the project, now expanding
// EVERY lane of EVERY batch. Do not trust this file until that passes.
// ============================================================================
`default_nettype none

module agu import im2col_pkg::*; #(
    parameter int SA_SIZE   = 4,
    parameter int DATA_PREC = 0,
    parameter int IMG_W     = 12,
    parameter int KER_W     = 4,
    parameter int CH_W      = 10,
    parameter int PAD_W     = 4,
    parameter int ADDR_W    = 32,
    parameter int OUTGRP_W  = 10,
    parameter int REDGRP_W  = 8,
    parameter int NBLK_W    = 3,
    parameter int PARALLEL_GROUPS = 8,     // runs computed per cycle (K)
    // ---- derived (do not override) --------------------------------------
    parameter int OUTPOS_PER_TILE = SA_SIZE*simd_scale(DATA_PREC)*dt_16(DATA_PREC),
    parameter int RED_PER_BEAT    = SA_SIZE*simd_scale(DATA_PREC)
)(
    input  logic                 clk,
    input  logic                 rst_n,

    input  logic                 start,
    input  logic [OUTGRP_W-1:0]  out_grp,
    input  logic [REDGRP_W-1:0]  red_grp,
    input  logic [NBLK_W-1:0]    n_blks,

    input  logic [IMG_W-1:0]     H,
    input  logic [IMG_W-1:0]     W,
    input  logic [CH_W-1:0]      C,
    input  logic [KER_W-1:0]     R,
    input  logic [KER_W-1:0]     S,
    input  logic [KER_W-1:0]     stride,
    input  logic [PAD_W-1:0]     pad_left,
    input  logic [PAD_W-1:0]     pad_right,
    input  logic [PAD_W-1:0]     pad_top,
    input  logic [ADDR_W-1:0]    img_base,

    // ---- K-wide batch output (handshake) -----------------------------------
    input  logic                              pix_ready,
    output logic                              pix_valid,
    output logic [PARALLEL_GROUPS-1:0]        lane_valid,
    output logic [ADDR_W-1:0]                 lane_addr    [PARALLEL_GROUPS],
    output logic [KER_W-1:0]                  lane_run_len [PARALLEL_GROUPS],
    output logic [KER_W-1:0]                  lane_pad_lead[PARALLEL_GROUPS],
    output logic [KER_W-1:0]                  lane_pad_trail[PARALLEL_GROUPS],
    output logic                              run_last,      // this batch completes the beat
    output logic                              tile_last,     // this batch completes the tile

    output logic                 busy,
    output logic                 done
);
    localparam int HWIDX_W = 2*IMG_W;
    localparam int COORD_W = IMG_W + KER_W + 2;
    localparam int RS_W    = 2*KER_W;
    localparam int HWv_W   = 2*IMG_W;
    localparam int ACC_W   = ADDR_W + IMG_W + 4;
    localparam int OP_W    = (OUTPOS_PER_TILE<=1)?1:$clog2(OUTPOS_PER_TILE);
    localparam int RPB_W   = (RED_PER_BEAT<=1)?1:$clog2(RED_PER_BEAT);

    logic                 div_start, div_busy, div_done;
    logic [HWIDX_W-1:0]   div_dvnd, div_dsor, div_quot, div_rem;
    seq_divider #(.DVND_W(HWIDX_W), .DSOR_W(HWIDX_W)) u_div (
        .clk, .rst_n, .start(div_start),
        .dividend(div_dvnd), .divisor(div_dsor),
        .quotient(div_quot), .remainder(div_rem),
        .busy(div_busy), .done(div_done)
    );

    logic [HWv_W-1:0]     HW_q;
    logic [RS_W-1:0]      RS_q;
    logic [HWIDX_W-1:0]   hw_start_q, crs_start_q;
    logic [HWIDX_W-1:0]   crs_rem_q;
    logic [IMG_W:0]       W_OUT_q;
    logic [IMG_W-1:0]     out_h0_q, out_w0_q;
    logic [CH_W-1:0]      c0_q;
    logic [KER_W-1:0]     r0_q, s0_q;
    logic signed [COORD_W-1:0] outhb0_calc, outwb0_calc;
    logic [ACC_W-1:0]     chw0_q;

    typedef enum logic [2:0] {
        S_IDLE, S_SU_WOUT, S_SU_OUTHW, S_SU_CRS, S_SU_RS, S_SU_FIN, S_SWEEP, S_DONE
    } state_t;
    state_t state_q;
    logic   su_wait;

    // ---- per-beat-constant sweep state (unchanged shape from rev 3) --------
    logic [OP_W-1:0]           op_q;
    logic [NBLK_W-1:0]         rc_q;
    logic [IMG_W-1:0]          out_h_q, out_w_q;
    logic signed [COORD_W-1:0] outhb_q, outwb_q;

    // ---- per-group (within-beat) resume state -------------------------------
    logic [KER_W-1:0]  s_q;
    logic [KER_W-1:0]  r_q;
    logic [CH_W-1:0]   c_q;
    logic [ACC_W-1:0]  chw_q;
    logic [RPB_W-1:0]  be_q;

    logic [COORD_W-1:0]        outh_str, outw_str;
    assign outh_str = out_h0_q * stride;
    assign outw_str = out_w0_q * stride;

    // ---- K-wide combinational batch (direct translation of the verified
    //      Python 'batch_fill_beat_rtl_shaped' model) -----------------------
    int s_lane   [0:PARALLEL_GROUPS];
    int r_lane   [0:PARALLEL_GROUPS];
    int c_lane   [0:PARALLEL_GROUPS];
    longint chw_lane [0:PARALLEL_GROUPS];  // widened: c*H*W can exceed 32 bits for large CH_W/IMG_W
    int be_lane  [0:PARALLEL_GROUPS];

    logic [ADDR_W-1:0] b_addr    [PARALLEL_GROUPS];
    int                b_run_len [PARALLEL_GROUPS];
    int                b_pad_lead[PARALLEL_GROUPS];
    int                b_pad_trail[PARALLEL_GROUPS];
    logic              b_valid   [PARALLEL_GROUPS];
    logic              batch_beat_done;   // any lane completed the beat this cycle

    always_comb begin
        s_lane[0]   = int'(s_q);
        r_lane[0]   = int'(r_q);
        c_lane[0]   = int'(c_q);
        chw_lane[0] = longint'(chw_q);
        be_lane[0]  = int'(be_q);
        batch_beat_done = 1'b0;

        for (int i = 0; i < PARALLEL_GROUPS; i = i + 1) begin
            automatic logic active;
            active = !batch_beat_done;
            b_valid[i] = active;

            if (active) begin
                automatic int rem_grp, rem_beat, rl;
                automatic logic signed [COORD_W-1:0] in_h, in_w_first, in_w_last;
                automatic logic                      row_valid;
                automatic int lead_i, trail_i;
                automatic logic group_complete, beat_complete;
                automatic logic signed [ACC_W-1:0] addr_full_i;

                rem_grp  = int'(S) - s_lane[i];
                rem_beat = int'(RED_PER_BEAT) - be_lane[i];
                rl = (rem_grp <= rem_beat) ? rem_grp : rem_beat;
                if (rl < 1) rl = 1;

                in_h       = outhb_q + r_lane[i];
                in_w_first = outwb_q + s_lane[i];
                in_w_last  = in_w_first + (rl - 1);
                row_valid  = (in_h >= 0) && (in_h < $signed({1'b0, H}));

                if (!row_valid) begin
                    lead_i  = rl; trail_i = 0;
                end else begin
                    lead_i  = (in_w_first >= 0) ? 0 :
                              ((-in_w_first) >= rl ? rl : -in_w_first);
                    trail_i = (in_w_last < $signed({1'b0, W})) ? 0 :
                              ((in_w_last - $signed({1'b0, W}) + 1) >= rl ? rl :
                               (in_w_last - $signed({1'b0, W}) + 1));
                    if (lead_i + trail_i > rl) begin lead_i = rl; trail_i = 0; end
                end

                addr_full_i = $signed({1'b0, img_base}) + ACC_W'(chw_lane[i])
                            + in_h * $signed({1'b0, W}) + in_w_first;
                b_addr[i]      = ADDR_W'(addr_full_i);
                b_run_len[i]   = rl;
                b_pad_lead[i]  = lead_i;
                b_pad_trail[i] = trail_i;

                group_complete = (rl == rem_grp);
                beat_complete  = (rl == rem_beat);

                be_lane[i+1] = beat_complete ? 0 : be_lane[i] + rl;
                if (group_complete) begin
                    s_lane[i+1] = 0;
                    if (r_lane[i] == int'(R) - 1) begin
                        r_lane[i+1] = 0; c_lane[i+1] = c_lane[i] + 1; chw_lane[i+1] = chw_lane[i] + longint'(HW_q);
                    end else begin
                        r_lane[i+1] = r_lane[i] + 1; c_lane[i+1] = c_lane[i]; chw_lane[i+1] = chw_lane[i];
                    end
                end else begin
                    s_lane[i+1] = s_lane[i] + rl;
                    r_lane[i+1] = r_lane[i]; c_lane[i+1] = c_lane[i]; chw_lane[i+1] = chw_lane[i];
                end

                if (beat_complete) batch_beat_done = 1'b1;
            end else begin
                // dead lane: no-op, propagate state unchanged
                s_lane[i+1] = s_lane[i]; r_lane[i+1] = r_lane[i]; c_lane[i+1] = c_lane[i];
                chw_lane[i+1] = chw_lane[i]; be_lane[i+1] = be_lane[i];
                b_addr[i] = '0; b_run_len[i] = 0; b_pad_lead[i] = 0; b_pad_trail[i] = 0;
            end
        end
    end

    // ---- boundary conditions (op/tile -- unchanged shape from rev 3) -------
    wire accept        = pix_valid && pix_ready;
    wire last_rc       = (rc_q == n_blks-1);
    wire last_op       = (op_q == OUTPOS_PER_TILE-1);
    wire op_boundary   = batch_beat_done && last_rc;
    wire tile_boundary = op_boundary && last_op;

    // ---- outputs -------------------------------------------------------------
    assign pix_valid  = (state_q == S_SWEEP);
    assign run_last   = pix_valid && batch_beat_done;
    assign tile_last  = pix_valid && tile_boundary;
    assign busy       = (state_q != S_IDLE) && (state_q != S_DONE);
    assign done       = (state_q == S_DONE);

    genvar gi;
    generate
        for (gi = 0; gi < PARALLEL_GROUPS; gi = gi + 1) begin : g_out
            assign lane_valid[gi]      = b_valid[gi];
            assign lane_addr[gi]       = b_addr[gi];
            assign lane_run_len[gi]    = b_run_len[gi][KER_W-1:0];
            assign lane_pad_lead[gi]   = b_pad_lead[gi][KER_W-1:0];
            assign lane_pad_trail[gi]  = b_pad_trail[gi][KER_W-1:0];
        end
    endgenerate

    // ---- setup operand mux (unchanged from rev 2/3) -------------------------
    always_comb begin
        div_dvnd = '0; div_dsor = '0;
        unique case (state_q)
            S_SU_WOUT : begin div_dvnd = HWIDX_W'(W + pad_left + pad_right - S);
                              div_dsor = HWIDX_W'(stride);           end
            S_SU_OUTHW: begin div_dvnd = hw_start_q;
                              div_dsor = HWIDX_W'(W_OUT_q);          end
            S_SU_CRS  : begin div_dvnd = crs_start_q;
                              div_dsor = HWIDX_W'(RS_q);             end
            S_SU_RS   : begin div_dvnd = crs_rem_q;
                              div_dsor = HWIDX_W'(S);                end
            default   : ;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_q<=S_IDLE; su_wait<=1'b0; div_start<=1'b0;
            HW_q<='0; RS_q<='0; hw_start_q<='0; crs_start_q<='0; crs_rem_q<='0;
            W_OUT_q<='0; out_h0_q<='0; out_w0_q<='0; c0_q<='0; r0_q<='0; s0_q<='0;
            chw0_q<='0; outhb0_calc<='0; outwb0_calc<='0;
            op_q<='0; rc_q<='0; be_q<='0; out_h_q<='0; out_w_q<='0;
            outhb_q<='0; outwb_q<='0; c_q<='0; r_q<='0; s_q<='0; chw_q<='0;
        end else begin
            div_start <= 1'b0;
            unique case (state_q)
                S_IDLE: if (start) begin
                    HW_q        <= H * W;
                    RS_q        <= R * S;
                    hw_start_q  <= out_grp * OUTPOS_PER_TILE;
                    crs_start_q <= red_grp * RED_PER_BEAT;
                    su_wait     <= 1'b0;
                    state_q     <= S_SU_WOUT;
                end
                S_SU_WOUT: begin
                    if (!su_wait && !div_busy) begin div_start<=1'b1; su_wait<=1'b1; end
                    else if (div_done) begin
                        W_OUT_q <= (IMG_W+1)'(div_quot + 1);
                        su_wait <= 1'b0; state_q <= S_SU_OUTHW;
                    end
                end
                S_SU_OUTHW: begin
                    if (!su_wait && !div_busy) begin div_start<=1'b1; su_wait<=1'b1; end
                    else if (div_done) begin
                        out_h0_q <= div_quot[IMG_W-1:0];
                        out_w0_q <= div_rem [IMG_W-1:0];
                        su_wait  <= 1'b0; state_q <= S_SU_CRS;
                    end
                end
                S_SU_CRS: begin
                    if (!su_wait && !div_busy) begin div_start<=1'b1; su_wait<=1'b1; end
                    else if (div_done) begin
                        c0_q      <= div_quot[CH_W-1:0];
                        crs_rem_q <= div_rem;
                        su_wait   <= 1'b0; state_q <= S_SU_RS;
                    end
                end
                S_SU_RS: begin
                    if (!su_wait && !div_busy) begin div_start<=1'b1; su_wait<=1'b1; end
                    else if (div_done) begin
                        r0_q    <= div_quot[KER_W-1:0];
                        s0_q    <= div_rem [KER_W-1:0];
                        su_wait <= 1'b0; state_q <= S_SU_FIN;
                    end
                end
                S_SU_FIN: begin
                    chw0_q  <= c0_q * HW_q;
                    op_q <= '0; rc_q <= '0; be_q <= '0;
                    out_h_q <= out_h0_q; out_w_q <= out_w0_q;
                    outhb_q <= $signed(outh_str) - $signed({1'b0, pad_top});
                    outwb_q <= $signed(outw_str) - $signed({1'b0, pad_left});
                    c_q <= c0_q; r_q <= r0_q; s_q <= s0_q;
                    chw_q <= c0_q * HW_q;
                    state_q <= S_SWEEP;
                end
                // ---- K-wide batched sweep --------------------------------
                S_SWEEP: if (accept) begin
                    if (op_boundary) begin
                        if (tile_boundary) begin
                            state_q <= S_DONE;
                        end else begin
                            op_q <= op_q + 1'b1; rc_q <= '0; be_q <= '0;
                            c_q <= c0_q; r_q <= r0_q; s_q <= s0_q; chw_q <= chw0_q;
                            if (out_w_q == W_OUT_q-1) begin
                                out_w_q <= '0; outwb_q <= -$signed({1'b0, pad_left});
                                out_h_q <= out_h_q + 1'b1; outhb_q <= outhb_q + $signed({1'b0, stride});
                            end else begin
                                out_w_q <= out_w_q + 1'b1; outwb_q <= outwb_q + $signed({1'b0, stride});
                            end
                        end
                    end else begin
                        // commit the batch's final lane state (correct
                        // whether the beat finished early or all K lanes
                        // were used without finishing -- see design notes)
                        s_q   <= s_lane  [PARALLEL_GROUPS][KER_W-1:0];
                        r_q   <= r_lane  [PARALLEL_GROUPS][KER_W-1:0];
                        c_q   <= c_lane  [PARALLEL_GROUPS][CH_W-1:0];
                        chw_q <= chw_lane[PARALLEL_GROUPS][ACC_W-1:0];
                        be_q  <= be_lane [PARALLEL_GROUPS][RPB_W-1:0];
                        // BUG FIX: this batch may have completed a BEAT
                        // (reduction chunk) without completing the whole
                        // output position (i.e. more rc chunks remain for
                        // this op, n_blks>1). rc_q must advance in that
                        // case -- it was previously only ever reset (in the
                        // op_boundary branch above), never incremented,
                        // so multi-chunk tiles never terminated correctly.
                        if (batch_beat_done) rc_q <= rc_q + 1'b1;
                    end
                end
                default: state_q <= S_IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
