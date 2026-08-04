// ============================================================================
// read_engine.sv (rev 2 -- streamlined protocol, 2 fewer cycles per miss)
// ----------------------------------------------------------------------------
// Turns a simple line-fetch request/response into transactions on the Ld_*
// scratchpad read port. Functionally identical to rev 1; only the internal
// FSM changed, to remove pure protocol overhead:
//
//   rev 1: R_IDLE -> R_REQ -> R_DATA -> R_RSP -> R_IDLE   (4 states)
//     - accept (R_IDLE) and address-issue (R_REQ) were separate cycles even
//       when the memory was immediately ready.
//     - data was latched into a register and PRESENTED a cycle later (R_RSP)
//       even though the consumer (gather_assemble) was already waiting with
//       rd_rsp_ready asserted the whole time.
//
//   rev 2: R_IDLE -> [R_ADDR] -> R_WAIT -> R_IDLE
//     - address issue happens COMBINATIONALLY the same cycle a request is
//       accepted; if the memory's address channel is ready that same cycle
//       (the common case), R_IDLE jumps straight to R_WAIT -- R_ADDR only
//       activates if the memory back-pressures the address phase.
//     - data is forwarded to the consumer COMBINATIONALLY the cycle it
//       arrives (rd_rsp_valid = ld_dat_valid while in R_WAIT), no extra
//       latch-and-present cycle.
//     Net effect: 2 fewer cycles of pure bookkeeping per request, on top of
//     whatever the real memory latency is. Traced cycle-by-cycle against
//     ld_scratchpad_model (RD_LATENCY=2): rev 1 = 6 cycles/miss end-to-end,
//     rev 2 = 4 cycles/miss end-to-end.
//
// Still one outstanding request at a time -- this is a protocol-overhead
// fix, not multi-outstanding pipelining. True request-level pipelining would
// need a lookahead FIFO between the AGU and gather_assemble (the AGU produces
// one pixel at a time with no lookahead today) -- a larger, separate change,
// not built here; flagged as a further option if still needed after this fix
// and the gather_assemble cache upgrade are measured.
// ============================================================================
`default_nettype none

module read_engine #(
    parameter int SA_SIZE       = 4,
    parameter int LD_ADDR_W     = 32,   // Ld_Base-addr (28 on the photo, 32 in ISA/AXI)
    parameter int WCOUNT_W      = 6,    // Ld_Word count
    parameter bit LD_WCOUNT_ENC = 1'b0, // 0 = count-1 (arlen style), 1 = literal count
    parameter int LINE_W        = SA_SIZE*32
)(
    input  logic                 clk,
    input  logic                 rst_n,

    // ---- upstream: line fetch request / response -------------------------
    input  logic                 rd_req_valid,
    input  logic [LD_ADDR_W-1:0] rd_req_addr,   // line-aligned byte address
    output logic                 rd_req_ready,
    output logic                 rd_rsp_valid,
    output logic [LINE_W-1:0]    rd_rsp_data,
    input  logic                 rd_rsp_ready,

    // ---- downstream: Ld_* port ---------------------------------------------
    output logic                 ld_req_valid,  // address-channel request
    input  logic                 ld_req_ready,
    output logic [LD_ADDR_W-1:0] ld_addr,
    output logic [WCOUNT_W-1:0]  ld_wcount,
    output logic                 ld_rnw,        // 1 = read
    input  logic                 ld_dat_valid,  // data-channel beat
    output logic                 ld_dat_ready,
    input  logic [LINE_W-1:0]    ld_dat
);
    typedef enum logic [1:0] { R_IDLE, R_ADDR, R_WAIT } state_t;
    state_t                 state_q;
    logic [LD_ADDR_W-1:0]   addr_q;

    // single-word read
    localparam logic [WCOUNT_W-1:0] WC_ONE = LD_WCOUNT_ENC ? WCOUNT_W'(1) : WCOUNT_W'(0);

    // ---- combinational protocol -------------------------------------------
    assign rd_req_ready = (state_q == R_IDLE);
    assign ld_req_valid = (state_q == R_ADDR) ||
                           ((state_q == R_IDLE) && rd_req_valid);   // fast-path issue
    assign ld_addr      = (state_q == R_IDLE) ? rd_req_addr : addr_q;
    assign ld_wcount    = WC_ONE;
    assign ld_rnw       = 1'b1;
    assign ld_dat_ready = (state_q == R_WAIT);
    assign rd_rsp_valid = (state_q == R_WAIT) && ld_dat_valid;      // combinational passthrough
    assign rd_rsp_data  = ld_dat;                                    // combinational passthrough

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_q <= R_IDLE; addr_q <= '0;
        end else begin
            unique case (state_q)
                R_IDLE: if (rd_req_valid) begin
                    addr_q  <= rd_req_addr;
                    state_q <= ld_req_ready ? R_WAIT : R_ADDR;   // fast path if mem ready now
                end
                R_ADDR: if (ld_req_ready) state_q <= R_WAIT;
                R_WAIT: if (ld_dat_valid && rd_rsp_ready) state_q <= R_IDLE;
                default: state_q <= R_IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
