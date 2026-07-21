// ============================================================================
// cfg_regs.sv
// ----------------------------------------------------------------------------
// Latches one CONV-LOAD instruction (AIACC_CONVLOADInsn fields) plus the
// externally-supplied grid_width, and holds them stable for the whole
// instruction. Leaf module: takes individual fields (not the packed struct) so
// it stays dependency-free and unit-testable; the top wrapper unpacks the
// struct and drives these ports.
//
// Field widths default to the ISA (ISADefines.sv):
//   X_Size=Y_Size=12, Kernel_*=Stride=4, Channel=10, n_blks=3,
//   Scratch_Base=32, SRAM_Base=6, Pad_*=4, Blk_ID_Start=8, blocks_per_row=10.
//
// On 'start', every field is registered and 'cfg_valid' is asserted; the
// registered outputs then remain constant until the next 'start'.
// ============================================================================

module cfg_regs #(
    parameter int IMG_W      = 12,   // X_Size / Y_Size
    parameter int KER_W      = 4,    // Kernel_X/Y_Size, Kernel_Stride
    parameter int CH_W       = 10,   // Channel_Dim
    parameter int NBLK_W     = 3,    // n_blks_to_load
    parameter int SCRATCH_W  = 32,   // Scratch_Base_Addr (image base)
    parameter int SRAMBASE_W = 6,    // SRAM_Base_Addr
    parameter int PAD_W       = 4,   // Pad_Left/Right/Top/Bottom
    parameter int BLKID_W     = 8,   // Blk_ID_Start
    parameter int GRIDW_W     = 10   // blocks_per_row (grid_width)
)(
    input   logic                    clk,
    input   logic                    rst_n,

    input   logic                    start,        // latch pulse (= start_conv_load)

    // ---- raw instruction fields (from unpacked AIACC_CONVLOADInsn) --------
    input   logic [IMG_W-1:0]        i_x_size,     // W
    input   logic [IMG_W-1:0]        i_y_size,     // H
    input   logic [KER_W-1:0]        i_ker_x,      // S
    input   logic [KER_W-1:0]        i_ker_y,      // R
    input   logic [CH_W-1:0]         i_channel,    // C
    input   logic [KER_W-1:0]        i_stride,
    input   logic [NBLK_W-1:0]       i_n_blks,
    input   logic [SCRATCH_W-1:0]    i_scratch_base, // image base addr
    input   logic [SRAMBASE_W-1:0]   i_sram_base,    // staging tile slot
    input   logic [PAD_W-1:0]        i_pad_left,
    input   logic [PAD_W-1:0]        i_pad_right,
    input   logic [PAD_W-1:0]        i_pad_top,
    input   logic [PAD_W-1:0]        i_pad_bottom,
    input   logic [BLKID_W-1:0]      i_blk_id_start,
    input   logic [GRIDW_W-1:0]      i_grid_width,   // blocks_per_row (given)

    // ---- registered, held-stable outputs --------------------------------
    output  logic                    cfg_valid,
    output  logic [IMG_W-1:0]        W,
    output  logic [IMG_W-1:0]        H,
    output  logic [KER_W-1:0]        S,
    output  logic [KER_W-1:0]        R,
    output  logic [CH_W-1:0]         C,
    output  logic [KER_W-1:0]        stride,
    output  logic [NBLK_W-1:0]       n_blks,
    output  logic [SCRATCH_W-1:0]    img_base,
    output  logic [SRAMBASE_W-1:0]   sram_base,
    output  logic [PAD_W-1:0]        pad_left,
    output  logic [PAD_W-1:0]        pad_right,
    output  logic [PAD_W-1:0]        pad_top,
    output  logic [PAD_W-1:0]        pad_bottom,
    output  logic [BLKID_W-1:0]      blk_id_start,
    output  logic [GRIDW_W-1:0]      grid_width
);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cfg_valid    <= 1'b0;
            W <= '0; H <= '0; S <= '0; R <= '0; C <= '0; stride <= '0;
            n_blks <= '0; img_base <= '0; sram_base <= '0;
            pad_left <= '0; pad_right <= '0; pad_top <= '0; pad_bottom <= '0;
            blk_id_start <= '0; grid_width <= '0;
        end else if (start) begin
            cfg_valid    <= 1'b1;
            W            <= i_x_size;
            H            <= i_y_size;
            S            <= i_ker_x;
            R            <= i_ker_y;
            C            <= i_channel;
            stride       <= i_stride;
            n_blks       <= i_n_blks;
            img_base     <= i_scratch_base;
            sram_base    <= i_sram_base;
            pad_left     <= i_pad_left;
            pad_right    <= i_pad_right;
            pad_top      <= i_pad_top;
            pad_bottom   <= i_pad_bottom;
            blk_id_start <= i_blk_id_start;
            grid_width   <= i_grid_width;
        end
        // else: hold (registers retain their values)
    end

endmodule
