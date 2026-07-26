// ============================================================================
// tb_agu.sv (rev 3 -- validates the K-wide batched AGU, rev 4)
// ----------------------------------------------------------------------------
// Same golden model as always (independently recomputes the im2col address
// formula), now expanding EVERY VALID LANE of EVERY ACCEPTED BATCH back into
// its individual elements and checking each one -- so a batching bug (wrong
// lane count, wrong per-lane length, wrong padding split, wrong chaining
// between lanes) shows up as a wrong-element failure at a specific global
// index, same diagnostic granularity used throughout this project.
//
// Plain SV-2012. Design files: im2col_pkg.sv, seq_divider.sv, agu.sv.
// INT8, SA_SIZE=4, PARALLEL_GROUPS=8 (default).
// ============================================================================
`timescale 1ns/1ps
module tb_agu;

    localparam int SA_SIZE   = 4;
    localparam int IMG_W     = 12;
    localparam int KER_W     = 4;
    localparam int CH_W      = 10;
    localparam int PAD_W     = 4;
    localparam int ADDR_W    = 32;
    localparam int OUTGRP_W  = 10;
    localparam int REDGRP_W  = 8;
    localparam int NBLK_W    = 3;
    localparam int K         = 8;    // PARALLEL_GROUPS
    localparam int OUTPOS_PER_TILE = 16;
    localparam int RED_PER_BEAT    = 16;

    logic                clk = 0, rst_n;
    logic                start;
    logic [OUTGRP_W-1:0] out_grp;
    logic [REDGRP_W-1:0] red_grp;
    logic [NBLK_W-1:0]   n_blks;
    logic [IMG_W-1:0]    H, W;
    logic [CH_W-1:0]     C;
    logic [KER_W-1:0]    R, S, stride;
    logic [PAD_W-1:0]    pad_left, pad_right, pad_top;
    logic [ADDR_W-1:0]   img_base;
    logic                pix_ready, pix_valid, run_last, tile_last;
    logic [K-1:0]        lane_valid;
    logic [ADDR_W-1:0]   lane_addr     [K];
    logic [KER_W-1:0]    lane_run_len  [K];
    logic [KER_W-1:0]    lane_pad_lead [K];
    logic [KER_W-1:0]    lane_pad_trail[K];
    logic                busy, done;

    integer errors = 0, tiles = 0, checked = 0;

    agu #(
        .SA_SIZE(SA_SIZE), .DATA_PREC(0), .IMG_W(IMG_W), .KER_W(KER_W),
        .CH_W(CH_W), .PAD_W(PAD_W), .ADDR_W(ADDR_W),
        .OUTGRP_W(OUTGRP_W), .REDGRP_W(REDGRP_W), .NBLK_W(NBLK_W),
        .PARALLEL_GROUPS(K)
    ) dut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .out_grp(out_grp), .red_grp(red_grp), .n_blks(n_blks),
        .H(H), .W(W), .C(C), .R(R), .S(S), .stride(stride),
        .pad_left(pad_left), .pad_right(pad_right), .pad_top(pad_top),
        .img_base(img_base),
        .pix_ready(pix_ready), .pix_valid(pix_valid),
        .lane_valid(lane_valid), .lane_addr(lane_addr),
        .lane_run_len(lane_run_len), .lane_pad_lead(lane_pad_lead), .lane_pad_trail(lane_pad_trail),
        .run_last(run_last), .tile_last(tile_last),
        .busy(busy), .done(done)
    );

    always #5 clk = ~clk;

    task run_tile(
        input integer og, input integer rg, input integer nb,
        input integer h, input integer w, input integer c,
        input integer r, input integer s, input integer st,
        input integer pl, input integer pr, input integer pt, input integer base
    );
        integer w_out, hw_start, crs_start, total, gidx;
        integer op, rem, rc, e, hw, out_h, out_w, crs, ss, rr, cc;
        integer in_h, in_w, exp_pad, exp_addr;
        integer lane, rl, plead, ptrail, k2, elem_gidx, elem_addr;
        begin
            out_grp=og; red_grp=rg; n_blks=nb[2:0];
            H=h; W=w; C=c; R=r; S=s; stride=st;
            pad_left=pl; pad_right=pr; pad_top=pt; img_base=base;

            w_out     = (w + pl + pr - s) / st + 1;
            hw_start  = og * OUTPOS_PER_TILE;
            crs_start = rg * RED_PER_BEAT;
            total     = OUTPOS_PER_TILE * nb * RED_PER_BEAT;

            @(negedge clk); start = 1; pix_ready = 1;
            @(negedge clk); start = 0;

            gidx = 0;
            while (gidx < total) begin
                @(negedge clk);
                if (pix_valid) begin
                    for (lane = 0; lane < K; lane = lane + 1) begin
                        if (lane_valid[lane]) begin
                            rl     = lane_run_len[lane];
                            plead  = lane_pad_lead[lane];
                            ptrail = lane_pad_trail[lane];
                            if (rl <= 0) begin
                                $display("  FAIL[%0d] lane %0d: run_len<=0 (%0d)", gidx, lane, rl);
                                errors = errors + 1;
                            end else begin
                                for (k2 = 0; k2 < rl; k2 = k2 + 1) begin
                                    elem_gidx = gidx + k2;
                                    op  = elem_gidx / (nb * RED_PER_BEAT);
                                    rem = elem_gidx % (nb * RED_PER_BEAT);
                                    rc  = rem / RED_PER_BEAT;
                                    e   = rem % RED_PER_BEAT;
                                    hw    = hw_start + op;
                                    out_h = hw / w_out;
                                    out_w = hw % w_out;
                                    crs = crs_start + rc*RED_PER_BEAT + e;
                                    ss  = crs % s;
                                    rr  = (crs / s) % r;
                                    cc  = crs / (r * s);
                                    in_h = out_h*st - pt + rr;
                                    in_w = out_w*st - pl + ss;
                                    exp_pad  = (in_h < 0) || (in_h >= h) || (in_w < 0) || (in_w >= w);
                                    exp_addr = base + cc*(h*w) + in_h*w + in_w;

                                    checked = checked + 1;
                                    if ((k2 < plead || k2 >= rl-ptrail) !== (exp_pad!=0)) begin
                                        $display("  FAIL[%0d] lane %0d elem %0d: pad mismatch (k=%0d plead=%0d ptrail=%0d rl=%0d) exp_pad=%0d",
                                                  elem_gidx, lane, k2, k2, plead, ptrail, rl, exp_pad);
                                        errors = errors + 1;
                                    end else if (!exp_pad) begin
                                        elem_addr = lane_addr[lane] + k2;
                                        if (elem_addr !== exp_addr) begin
                                            $display("  FAIL[%0d] lane %0d elem %0d: addr 0x%0h (base 0x%0h +%0d) exp 0x%0h (c=%0d in_h=%0d in_w=%0d)",
                                                      elem_gidx, lane, k2, elem_addr, lane_addr[lane], k2, exp_addr, cc, in_h, in_w);
                                            errors = errors + 1;
                                        end
                                    end
                                end
                                gidx = gidx + rl;
                            end
                        end
                    end
                end
            end
            tiles = tiles + 1;
            $display("  tile done: og=%0d rg=%0d nb=%0d %0dx%0d C=%0d k=%0dx%0d st=%0d pad(L%0d T%0d) -> %0d elements (via K=%0d-wide batches)",
                      og, rg, nb, h, w, c, r, s, st, pl, pt, total, K);
            repeat (4) @(negedge clk);
        end
    endtask

    initial begin
        rst_n=0; start=0; pix_ready=0;
        out_grp=0; red_grp=0; n_blks=1;
        H=8; W=8; C=2; R=3; S=3; stride=1; pad_left=0; pad_right=0; pad_top=0; img_base=0;
        repeat (3) @(negedge clk);
        rst_n = 1;

        $display("[tb_agu] starting (rev 4, K-wide batched, K=%0d)", K);
        // cfg1: no padding, stride 1 (S=3 does not divide RED_PER_BEAT=16 evenly
        // -- exercises both within-lane-chain group clipping AND K-wide batching)
        run_tile(0, 0, 1,  8, 8, 2, 3, 3, 1, 0, 0, 0, 0);
        // cfg2: padding 1, stride 1
        run_tile(0, 0, 1,  8, 8, 2, 3, 3, 1, 1, 1, 1, 0);
        // cfg3: stride 2, padding 1
        run_tile(0, 0, 1,  8, 8, 2, 3, 3, 2, 1, 1, 1, 0);
        // cfg4: nonzero out_grp and img_base
        run_tile(1, 0, 1,  8, 8, 2, 3, 3, 1, 0, 0, 0, 32'h1000);
        // cfg5: S=4 exactly divides RED_PER_BEAT=16 (clean case)
        run_tile(0, 0, 1,  8, 8, 1, 4, 4, 1, 0, 0, 0, 0);
        // cfg6: multiple reduction chunks (n_blks=2)
        run_tile(0, 0, 2,  8, 8, 2, 3, 3, 1, 1, 1, 1, 0);
        // cfg7: large C so groups-per-beat exceeds K=8 (forces MULTIPLE
        // batch cycles per beat, exercising cross-batch resume)
        run_tile(0, 0, 1,  16, 16, 8, 4, 4, 1, 0, 0, 0, 0);

        if (errors == 0) $display("[tb_agu] PASS (%0d tiles, %0d elements checked)", tiles, checked);
        else             $display("[tb_agu] FAIL (%0d errors, %0d checked)", errors, checked);
        $finish;
    end

    initial begin
        #4000000; $display("[tb_agu] TIMEOUT (check setup-divide / handshake)"); $finish;
    end
endmodule
