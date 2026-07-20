// ============================================================================
// block_decoder.sv
// ----------------------------------------------------------------------------
// Decodes a flat block id into its 2-D block-grid coordinates:
//     out_grp = blk_id_start % grid_width   -> block-grid COLUMN (output group)
//     red_grp = blk_id_start / grid_width   -> block-grid ROW    (reduction group)
// These are GRID coordinates (positions in the lowered matrix), NOT image/IFM
// pixel coordinates -- the pixel math lives in the AGU.
//
// The divide is done by the shared seq_divider (single divider in the whole
// design). One decode per 'load'; 'ready' rises when the quotient/remainder are
// valid and stays high until the next 'load'. Reduction panning is handled
// inside the AGU, so there is no step/pan port here.
//
// Precision-independent: block-id decoding does not depend on the data type.
// ============================================================================

module block_decoder #(
    parameter int BLK_ID_W  = 8,               // Blk_ID_Start (ISA: 8 bits)
    parameter int GRID_W_W  = 10,              // blocks_per_row (DUT: 10 bits)
    // derived output widths (do NOT override)
    parameter int OUT_GRP_W = GRID_W_W,        // remainder < grid_width
    parameter int RED_GRP_W = BLK_ID_W         // quotient  <= blk_id_start
)(
    input  logic                  clk,
    input  logic                  rst_n,

    input  logic                  load,          // 1-cycle pulse: start a decode
    input  logic [BLK_ID_W-1:0]   blk_id_start,
    input  logic [GRID_W_W-1:0]   grid_width,    // MUST be > 0

    output logic [OUT_GRP_W-1:0]  out_grp,       // valid when 'ready'
    output logic [RED_GRP_W-1:0]  red_grp,       // valid when 'ready'
    output logic                  ready
);
    // ---- shared sequential divider --------------------------------------
    logic [BLK_ID_W-1:0]  quot;
    logic [GRID_W_W-1:0]  rem;
    logic                 div_done;

    seq_divider #(.DVND_W(BLK_ID_W), .DSOR_W(GRID_W_W)) u_div (
        .clk(clk), .rst_n(rst_n), .start(load),
        .dividend(blk_id_start), .divisor(grid_width),
        .quotient(quot), .remainder(rem),
        .busy(), .done(div_done)
    );

    // ---- capture results, manage 'ready' --------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ready <= 1'b0; out_grp <= '0; red_grp <= '0;
        end else if (load) begin
            ready <= 1'b0;                 // new decode in flight
        end else if (div_done) begin
            red_grp <= quot;    // BLK_ID_W == RED_GRP_W (default): full-width copy
            out_grp <= rem;     // GRID_W_W == OUT_GRP_W (default): full-width copy
            ready   <= 1'b1;
        end
    end

    // ---- assertion (ignored by synthesis) -------------------------------
`ifndef SYNTHESIS
    always_ff @(posedge clk)
        if (rst_n && load && (grid_width == 0))
            $error("block_decoder: grid_width must be > 0 at load");
`endif
endmodule
