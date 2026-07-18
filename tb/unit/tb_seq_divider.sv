// ============================================================================
// tb_seq_divider.sv  -- self-checking unit testbench for seq_divider
// ----------------------------------------------------------------------------
// Plain SV-2012 (no classes/covergroups/randomize). Runs on Icarus Verilog 12,
// EDA Playground (Aldec/Synopsys), etc. Add seq_divider.sv to the design.
// ============================================================================
`timescale 1ns/1ps
module tb_seq_divider;

    localparam int DVND_W = 20;
    localparam int DSOR_W = 12;

    logic                clk = 0;
    logic                rst_n;
    logic                start;
    logic [DVND_W-1:0]   dividend;
    logic [DSOR_W-1:0]   divisor;
    logic [DVND_W-1:0]   quotient;
    logic [DSOR_W-1:0]   remainder;
    logic                busy, done;

    integer errors = 0;
    integer tests  = 0;

    seq_divider #(.DVND_W(DVND_W), .DSOR_W(DSOR_W)) dut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .dividend(dividend), .divisor(divisor),
        .quotient(quotient), .remainder(remainder),
        .busy(busy), .done(done)
    );

    always #5 clk = ~clk;

    // one division, then check q and rem
    task run_div(input [DVND_W-1:0] a, input [DSOR_W-1:0] b);
        begin
            @(negedge clk); start = 1; dividend = a; divisor = b;
            @(negedge clk); start = 0;
            // wait for done pulse
            while (!done) @(negedge clk);
            tests = tests + 1;
            if (quotient !== (a / b) || remainder !== (a % b)) begin
                $display("  FAIL: %0d / %0d -> q=%0d r=%0d  (exp q=%0d r=%0d)",
                          a, b, quotient, remainder, a / b, a % b);
                errors = errors + 1;
            end else begin
                $display("  ok  : %0d / %0d -> q=%0d r=%0d", a, b, quotient, remainder);
            end
            @(negedge clk);
        end
    endtask

    initial begin
        rst_n = 0; start = 0; dividend = 0; divisor = 1;
        repeat (3) @(negedge clk);
        rst_n = 1;

        $display("[tb_seq_divider] starting");
        run_div(100, 7);
        run_div(255, 16);
        run_div(1023, 33);
        run_div(0, 5);
        run_div(65535, 256);
        run_div(12345, 1);
        run_div(4095, 64);
        run_div(999999, 1000);
        run_div(7, 8);          // a < b -> q=0, r=a

        if (errors == 0) $display("[tb_seq_divider] PASS (%0d tests)", tests);
        else             $display("[tb_seq_divider] FAIL (%0d/%0d failed)", errors, tests);
        $finish;
    end

    // safety timeout
    initial begin
        #100000;
        $display("[tb_seq_divider] TIMEOUT");
        $finish;
    end
endmodule
