// ============================================================================
// tb_cfg_regs.sv  -- self-checking unit testbench for cfg_regs
// ----------------------------------------------------------------------------
// Checks that fields are latched on 'start', held stable afterwards, and that
// inputs changing WITHOUT 'start' do not disturb the registered outputs.
// Plain SV-2012. Design file: cfg_regs.sv.
// ============================================================================
`timescale 1ns/1ps
module tb_cfg_regs;

    localparam int IMG_W=12, KER_W=4, CH_W=10, NBLK_W=3, SCRATCH_W=32;
    localparam int SRAMBASE_W=6, PAD_W=4, BLKID_W=8, GRIDW_W=10;

    logic clk=0, rst_n, start;
    logic [IMG_W-1:0]     i_x_size, i_y_size;
    logic [KER_W-1:0]     i_ker_x, i_ker_y, i_stride;
    logic [CH_W-1:0]      i_channel;
    logic [NBLK_W-1:0]    i_n_blks;
    logic [SCRATCH_W-1:0] i_scratch_base;
    logic [SRAMBASE_W-1:0]i_sram_base;
    logic [PAD_W-1:0]     i_pad_left, i_pad_right, i_pad_top, i_pad_bottom;
    logic [BLKID_W-1:0]   i_blk_id_start;
    logic [GRIDW_W-1:0]   i_grid_width;

    logic                 cfg_valid;
    logic [IMG_W-1:0]     W, H;
    logic [KER_W-1:0]     S, R, stride;
    logic [CH_W-1:0]      C;
    logic [NBLK_W-1:0]    n_blks;
    logic [SCRATCH_W-1:0] img_base;
    logic [SRAMBASE_W-1:0]sram_base;
    logic [PAD_W-1:0]     pad_left, pad_right, pad_top, pad_bottom;
    logic [BLKID_W-1:0]   blk_id_start;
    logic [GRIDW_W-1:0]   grid_width;

    integer errors=0, tests=0;

    cfg_regs dut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .i_x_size(i_x_size), .i_y_size(i_y_size), .i_ker_x(i_ker_x), .i_ker_y(i_ker_y),
        .i_channel(i_channel), .i_stride(i_stride), .i_n_blks(i_n_blks),
        .i_scratch_base(i_scratch_base), .i_sram_base(i_sram_base),
        .i_pad_left(i_pad_left), .i_pad_right(i_pad_right),
        .i_pad_top(i_pad_top), .i_pad_bottom(i_pad_bottom),
        .i_blk_id_start(i_blk_id_start), .i_grid_width(i_grid_width),
        .cfg_valid(cfg_valid),
        .W(W), .H(H), .S(S), .R(R), .C(C), .stride(stride), .n_blks(n_blks),
        .img_base(img_base), .sram_base(sram_base),
        .pad_left(pad_left), .pad_right(pad_right), .pad_top(pad_top), .pad_bottom(pad_bottom),
        .blk_id_start(blk_id_start), .grid_width(grid_width)
    );

    always #5 clk = ~clk;

    task drive(input [11:0] xw, input [11:0] yh, input [3:0] kx, input [3:0] ky,
               input [9:0] ch, input [3:0] st, input [2:0] nb, input [31:0] sc,
               input [5:0] sb, input [3:0] pl, input [3:0] pr, input [3:0] pt,
               input [3:0] pb, input [7:0] bid, input [9:0] gw);
        begin
            i_x_size=xw; i_y_size=yh; i_ker_x=kx; i_ker_y=ky; i_channel=ch;
            i_stride=st; i_n_blks=nb; i_scratch_base=sc; i_sram_base=sb;
            i_pad_left=pl; i_pad_right=pr; i_pad_top=pt; i_pad_bottom=pb;
            i_blk_id_start=bid; i_grid_width=gw;
        end
    endtask

    task check_outputs(input [11:0] xw, input [11:0] yh, input [3:0] kx, input [3:0] ky,
                       input [9:0] ch, input [3:0] st, input [2:0] nb, input [31:0] sc,
                       input [5:0] sb, input [3:0] pl, input [3:0] pr, input [3:0] pt,
                       input [3:0] pb, input [7:0] bid, input [9:0] gw);
        begin
            tests = tests + 1;
            if (W!==xw || H!==yh || S!==kx || R!==ky || C!==ch || stride!==st ||
                n_blks!==nb || img_base!==sc || sram_base!==sb ||
                pad_left!==pl || pad_right!==pr || pad_top!==pt || pad_bottom!==pb ||
                blk_id_start!==bid || grid_width!==gw) begin
                $display("  FAIL: latched fields mismatch");
                $display("    W=%0d/%0d H=%0d/%0d S=%0d/%0d R=%0d/%0d C=%0d/%0d st=%0d/%0d",
                         W,xw,H,yh,S,kx,R,ky,C,ch,stride,st);
                $display("    nblk=%0d/%0d img=0x%0h/0x%0h sram=%0d/%0d bid=%0d/%0d gw=%0d/%0d",
                         n_blks,nb,img_base,sc,sram_base,sb,blk_id_start,bid,grid_width,gw);
                errors = errors + 1;
            end else
                $display("  ok  : fields latched & held correctly");
        end
    endtask

    initial begin
        rst_n=0; start=0;
        drive(0,0,0,0,0,1,0,0,0,0,0,0,0,0,1);
        repeat(3) @(negedge clk); rst_n=1;

        $display("[tb_cfg_regs] starting");

        // ---- load set A, then check
        drive(8,8,3,3,64,1,4,32'hDEAD_0000,6'h2,1,1,1,1,8'h05,10'd7);
        @(negedge clk); start=1;
        @(negedge clk); start=0;
        @(negedge clk);
        check_outputs(8,8,3,3,64,1,4,32'hDEAD_0000,6'h2,1,1,1,1,8'h05,10'd7);

        // ---- change inputs WITHOUT start: outputs must HOLD set A
        drive(99,99,7,7,1023,2,7,32'hBEEF_FFFF,6'h3F,2,2,2,2,8'hAA,10'd511);
        repeat(3) @(negedge clk);
        check_outputs(8,8,3,3,64,1,4,32'hDEAD_0000,6'h2,1,1,1,1,8'h05,10'd7);

        // ---- now pulse start: outputs update to set B
        @(negedge clk); start=1;
        @(negedge clk); start=0;
        @(negedge clk);
        check_outputs(99,99,7,7,1023,2,7,32'hBEEF_FFFF,6'h3F,2,2,2,2,8'hAA,10'd511);

        if (errors==0) $display("[tb_cfg_regs] PASS (%0d tests)", tests);
        else           $display("[tb_cfg_regs] FAIL (%0d errors)", errors);
        $finish;
    end

    initial begin #100000; $display("[tb_cfg_regs] TIMEOUT"); $finish; end
endmodule
