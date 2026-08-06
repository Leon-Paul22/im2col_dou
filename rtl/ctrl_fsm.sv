// ============================================================================
// ctrl_fsm.sv  -- top-level orchestration
// ----------------------------------------------------------------------------
// Sequences one CONV-LOAD:
//   start_conv_load -> latch config + kick block decode
//                   -> wait block-decode ready
//                   -> start AGU and write_engine together
//                   -> wait write_engine done
//                   -> pulse load_conv_done
//
// The read/gather stages are self-timed (they react to the AGU pixel stream and
// to memory responses), so they need no explicit start.
// ============================================================================

module ctrl_fsm (
    input   logic clk,
    input   logic rst_n,

    input   logic start_conv_load,   // external start pulse
    input   logic bd_ready,           // block_decoder decode complete
    input   logic wr_done,            // write_engine finished the tile

    output  logic cfg_latch,          // -> cfg_regs.start
    output  logic bd_load,            // -> block_decoder.load
    output  logic agu_start,          // -> agu.start
    output  logic wr_start,           // -> write_engine.start
    output  logic load_conv_done      // external done pulse
);
    typedef enum logic [1:0] { C_IDLE, C_WAIT_BD, C_RUN, C_DONE } state_t;
    state_t state_q;

    // default de-asserted; pulsed by state transitions
    assign cfg_latch      = (state_q == C_IDLE) && start_conv_load;
    assign bd_load        = (state_q == C_IDLE) && start_conv_load;
    assign agu_start      = (state_q == C_WAIT_BD) && bd_ready;
    assign wr_start       = (state_q == C_WAIT_BD) && bd_ready;
    assign load_conv_done = (state_q == C_DONE);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) state_q <= C_IDLE;
        else unique case (state_q)
            C_IDLE   : if (start_conv_load) state_q <= C_WAIT_BD;
            C_WAIT_BD: if (bd_ready)        state_q <= C_RUN;
            C_RUN    : if (wr_done)         state_q <= C_DONE;
            default  : state_q <= C_IDLE;   // C_DONE : 1-cycle pulse
        endcase
    end
endmodule
