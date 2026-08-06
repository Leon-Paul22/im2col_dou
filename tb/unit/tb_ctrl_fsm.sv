// ============================================================================
// tb_ctrl_fsm.sv  -- self-checking unit testbench for ctrl_fsm
// ----------------------------------------------------------------------------
// Verifies the sequence:
//   start_conv_load -> {cfg_latch, bd_load} pulse
//                   -> (wait bd_ready) -> {agu_start, wr_start} pulse
//                   -> (wait wr_done)  -> load_conv_done pulse
// The pulse outputs are COMBINATIONAL, so after driving an input we settle
// (#1) before sampling the dependent output. Plain SV-2012. Design: ctrl_fsm.sv.
// ============================================================================
`timescale 1ns/1ps
module tb_ctrl_fsm;

    logic clk=0, rst_n;
    logic start_conv_load, bd_ready, wr_done;
    logic cfg_latch, bd_load, agu_start, wr_start, load_conv_done;

    integer errors=0;

    ctrl_fsm dut (
        .clk(clk), .rst_n(rst_n),
        .start_conv_load(start_conv_load), .bd_ready(bd_ready), .wr_done(wr_done),
        .cfg_latch(cfg_latch), .bd_load(bd_load),
        .agu_start(agu_start), .wr_start(wr_start), .load_conv_done(load_conv_done)
    );

    always #5 clk = ~clk;

    task expect_eq(input logic got, input logic exp, input string name);
        begin
            if (got !== exp) begin
                $display("  FAIL: %s = %b (exp %b)", name, got, exp);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        rst_n=0; start_conv_load=0; bd_ready=0; wr_done=0;
        repeat(3) @(negedge clk); rst_n=1;
        @(negedge clk);

        $display("[tb_ctrl_fsm] starting");

        // ---- idle: nothing asserted
        expect_eq(cfg_latch,0,"cfg_latch@idle"); expect_eq(load_conv_done,0,"done@idle");

        // ---- pulse start_conv_load in IDLE -> cfg_latch & bd_load high this cycle
        @(negedge clk); start_conv_load=1; #1;
        expect_eq(cfg_latch,1,"cfg_latch"); expect_eq(bd_load,1,"bd_load");
        expect_eq(agu_start,0,"agu_start@idle");
        @(negedge clk); start_conv_load=0;              // -> C_WAIT_BD
        expect_eq(cfg_latch,0,"cfg_latch after"); expect_eq(bd_load,0,"bd_load after");

        // ---- in WAIT_BD: bd_ready 0 -> agu_start 0, then assert bd_ready
        expect_eq(agu_start,0,"agu_start noready");
        bd_ready=1; #1;
        expect_eq(agu_start,1,"agu_start"); expect_eq(wr_start,1,"wr_start");
        @(negedge clk); bd_ready=0;                     // -> C_RUN
        expect_eq(agu_start,0,"agu_start after"); expect_eq(wr_start,0,"wr_start after");

        // ---- in RUN: wr_done 0 -> done 0, then assert wr_done
        expect_eq(load_conv_done,0,"done@run");
        wr_done=1;
        @(negedge clk); wr_done=0;                      // -> C_DONE (settled at edge)
        expect_eq(load_conv_done,1,"load_conv_done");
        @(negedge clk);                                 // -> C_IDLE
        expect_eq(load_conv_done,0,"done after");

        // ---- second run: confirm it re-arms
        @(negedge clk); start_conv_load=1; #1;
        expect_eq(cfg_latch,1,"cfg_latch run2");
        @(negedge clk); start_conv_load=0;
        bd_ready=1; #1; expect_eq(agu_start,1,"agu_start run2");
        @(negedge clk); bd_ready=0;
        wr_done=1; @(negedge clk); wr_done=0;
        expect_eq(load_conv_done,1,"done run2");
        @(negedge clk);

        if (errors==0) $display("[tb_ctrl_fsm] PASS");
        else           $display("[tb_ctrl_fsm] FAIL (%0d errors)", errors);
        $finish;
    end

    initial begin #100000; $display("[tb_ctrl_fsm] TIMEOUT"); $finish; end
endmodule
