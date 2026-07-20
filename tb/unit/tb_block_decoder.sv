// ============================================================================
// tb_block_decoder.sv  -- self-checking unit testbench for block_decoder
// ----------------------------------------------------------------------------
// Checks out_grp = blk_id % grid_width and red_grp = blk_id / grid_width after
// 'ready'. Plain SV-2012. Add block_decoder.sv to the design.
// ============================================================================
`timescale 1ns/1ps
module tb_block_decoder;

    localparam int BLK_ID_W = 8;
    localparam int GRID_W_W = 10;

    logic                 clk = 0;
    logic                 rst_n;
    logic                 load;
    logic [BLK_ID_W-1:0]  blk_id_start;
    logic [GRID_W_W-1:0]  grid_width;
    logic [GRID_W_W-1:0]  out_grp;
    logic [BLK_ID_W-1:0]  red_grp;
    logic                 ready;

    integer errors = 0;
    integer tests  = 0;

    block_decoder #(.BLK_ID_W(BLK_ID_W), .GRID_W_W(GRID_W_W)) dut (
        .clk(clk), .rst_n(rst_n), .load(load),
        .blk_id_start(blk_id_start), .grid_width(grid_width),
        .out_grp(out_grp), .red_grp(red_grp), .ready(ready)
    );

    always #5 clk = ~clk;

    task run_decode(input [BLK_ID_W-1:0] bid, input [GRID_W_W-1:0] gw);
        begin
            @(negedge clk); load = 1; blk_id_start = bid; grid_width = gw;
            @(negedge clk); load = 0;
            while (!ready) @(negedge clk);
            tests = tests + 1;
            if (out_grp !== (bid % gw) || red_grp !== (bid / gw)) begin
                $display("  FAIL: blk=%0d gw=%0d -> out_grp=%0d red_grp=%0d (exp %0d / %0d)",
                          bid, gw, out_grp, red_grp, bid % gw, bid / gw);
                errors = errors + 1;
            end else begin
                $display("  ok  : blk=%0d gw=%0d -> out_grp=%0d red_grp=%0d",
                          bid, gw, out_grp, red_grp);
            end
            @(negedge clk);
        end
    endtask

    initial begin
        rst_n = 0; load = 0; blk_id_start = 0; grid_width = 1;
        repeat (3) @(negedge clk);
        rst_n = 1;

        $display("[tb_block_decoder] starting");
        run_decode(0,   4);
        run_decode(5,   4);   // 5%4=1, 5/4=1
        run_decode(7,   3);   // 1, 2
        run_decode(255, 16);  // 15, 15
        run_decode(100, 10);  // 0, 10
        run_decode(23,  8);   // 7, 2
        run_decode(1,   1);   // 0, 1
        run_decode(200, 7);   // 4, 28

        if (errors == 0) $display("[tb_block_decoder] PASS (%0d tests)", tests);
        else             $display("[tb_block_decoder] FAIL (%0d/%0d failed)", errors, tests);
        $finish;
    end

    initial begin
        #100000; $display("[tb_block_decoder] TIMEOUT"); $finish;
    end
endmodule
