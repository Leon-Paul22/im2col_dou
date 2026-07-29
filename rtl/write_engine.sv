// ============================================================================
// write_engine.sv  (rev 5 -- precision math sourced from im2col_pkg)
// ----------------------------------------------------------------------------
// Drives the staging write port (router_* of input BRAM inp_mem_simd).
// One LINE_W-bit beat per clock, self-paced single-port write.
//
//   en_a->router_en  we_a->router_wen  addr_a->router_addr  din_a->router_wdata
//
// ADDRESS SCHEME (tb verify_tile, INT8/INT16 branch):
//     A beat = one OUTPUT POSITION's reduction chunk.
//     bank   = output_pos % NUM_BANKS
//     offset = (output_pos / NUM_BANKS) + rchunk*SA_SIZE
//     addr   = SRAM_Base_Addr*(SA_SIZE*KN_DEPTH*NUM_BANKS) + { bank , offset }
//
// Tile shape now comes from im2col_pkg (single source of truth, shared with
// agu / gather_assemble): outpos_per_tile(), sram_tile_stride(), bytes_per_elem().
//
// SCOPE: INT8 & INT16 (shared datapath). INT32 forks (paired-bank interleave).
// ============================================================================

module write_engine import im2col_pkg::*; #(
    parameter int SA_SIZE   = 4,     // AIACC_SA_SIZE
    parameter int DATA_PREC = 0,     // 0=INT8 1=FP16 2=INT32 3=INT16 4=BF16 5=FP32
    parameter int NUM_BANKS = 4,
    parameter int ADDR_W    = 16,    // AIACC_INP_BRAM_ADDR_WIDTH
    parameter int KN_DEPTH  = 4,
    parameter int RCHUNK_W  = 4,     // width of the reduction-chunk count input
    parameter int SRAMBASE_W= 6,     // AIACC_MEMOP_SRAM_ADDR_BIT_WIDTH_CONV
    // ---- derived widths (for ports; do NOT override) ---------------------
    parameter int LINE_W    = SA_SIZE*32,
    parameter int BANK_BITS = (NUM_BANKS<=1)?1:$clog2(NUM_BANKS),
    parameter int OFFSET_BITS = ADDR_W - BANK_BITS
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // ---- control ---------------------------------------------------------
    input  logic                    start,           // = start_conv_load
    input  logic [RCHUNK_W-1:0]     n_rchunks,       // reduction chunks this tile (<=KN_DEPTH)
    input  logic [SRAMBASE_W-1:0]   sram_base_addr,  // ISA SRAM_Base_Addr (tile slot)

    // ---- beat input (from assembly buffer) -------------------------------
    input  logic                    beat_valid,
    input  logic [LINE_W-1:0]       beat_data,       // one window's reduction chunk
    output logic                    beat_ready,

    // ---- staging write port (router_*) -----------------------------------
    output logic                    en_a,
    output logic                    we_a,
    output logic [ADDR_W-1:0]       addr_a,
    output logic [LINE_W-1:0]       din_a,

    // ---- status ----------------------------------------------------------
    output logic                    done
);
    // ---- tile shape from the shared package ------------------------------
    localparam int OUTPOS_PER_TILE  = outpos_per_tile(SA_SIZE, DATA_PREC);
    localparam int SRAM_TILE_STRIDE = sram_tile_stride(SA_SIZE, KN_DEPTH, NUM_BANKS);
    localparam int OP_W             = (OUTPOS_PER_TILE<=1)?1:$clog2(OUTPOS_PER_TILE);

    typedef enum logic [1:0] { S_IDLE, S_WRITE, S_DONE } state_t;
    state_t                  state_q;

    logic [OP_W-1:0]         op_q;      // output position within tile (-> bank)
    logic [RCHUNK_W-1:0]     rc_q;      // reduction chunk (-> +SA_SIZE step)
    logic [RCHUNK_W-1:0]     n_rc_q;
    logic [OFFSET_BITS-1:0]  tile_base_q;

    wire xfer      = (state_q==S_WRITE) && beat_valid;
    wire last_rc   = (rc_q == n_rc_q-1);
    wire last_op   = (op_q == OUTPOS_PER_TILE-1);
    wire last_beat = xfer && last_rc && last_op;

    // ---- address (combinational) -----------------------------------------
    logic [BANK_BITS-1:0]   bank;
    logic [OFFSET_BITS-1:0] offset;
    always_comb begin
        bank   = op_q[BANK_BITS-1:0];                     // output_pos % NUM_BANKS
        offset = tile_base_q
               + (op_q >> BANK_BITS)                      // output_pos / NUM_BANKS
               + (rc_q * SA_SIZE);                        // reduction chunk step
        addr_a = {bank, offset};
    end

    assign din_a      = beat_data;
    assign en_a       = xfer;
    assign we_a       = xfer;
    assign beat_ready = (state_q==S_WRITE);
    assign done       = (state_q==S_DONE);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_q<=S_IDLE; op_q<='0; rc_q<='0; n_rc_q<='0; tile_base_q<='0;
        end else begin
            unique case (state_q)
                S_IDLE:
                    if (start) begin
                        op_q<='0; rc_q<='0; n_rc_q<=n_rchunks;
                        tile_base_q <= OFFSET_BITS'(sram_base_addr * SRAM_TILE_STRIDE);
                        state_q<=S_WRITE;
                    end
                S_WRITE:
                    if (xfer) begin
                        if      (last_beat) state_q<=S_DONE;
                        else if (last_rc)   begin rc_q<='0; op_q<=op_q+1'b1; end // next window
                        else                rc_q<=rc_q+1'b1;                     // next chunk
                    end
                default: state_q<=S_IDLE;   // S_DONE : 1-cycle pulse
            endcase
        end
    end

`ifndef SYNTHESIS
    initial begin
        if (OUTPOS_PER_TILE % NUM_BANKS != 0)
            $fatal(1,"write_engine: OUTPOS_PER_TILE(%0d) must be divisible by NUM_BANKS(%0d)",
                     OUTPOS_PER_TILE, NUM_BANKS);
        if (bytes_per_elem(DATA_PREC)==4)
            $fatal(1,"write_engine: 32-bit path uses the paired-bank interleave variant, not this module");
    end
`endif
endmodule
