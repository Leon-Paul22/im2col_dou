// ============================================================================
// seq_divider.sv
// ----------------------------------------------------------------------------
// Reusable unsigned sequential (restoring) divider. quotient = dividend/divisor,
// remainder = dividend%divisor, over DVND_W cycles. Handshake: pulse 'start'
// with operands valid; 'done' pulses one cycle when ready; 'busy' high while
// dividing. Caller must guarantee divisor > 0.
//
// NOTE: written select-free inside always blocks (shifts/OR instead of
// [msb] / [hi:lo]) so it runs on older Icarus Verilog builds that reject
// constant part-selects in processes.
// ============================================================================

module seq_divider #(
    parameter int DVND_W = 32,
    parameter int DSOR_W = 16
)(
    input  logic                 clk,
    input  logic                 rst_n,
    input  logic                 start,
    input  logic [DVND_W-1:0]    dividend,
    input  logic [DSOR_W-1:0]    divisor,
    output logic [DVND_W-1:0]    quotient,
    output logic [DSOR_W-1:0]    remainder,
    output logic                 busy,
    output logic                 done
);
    localparam int REM_W = DSOR_W + 1;
    localparam int CNT_W = (DVND_W <= 1) ? 1 : $clog2(DVND_W + 1);

    logic [DVND_W-1:0] dvnd_q, quot_q;
    logic [DSOR_W-1:0] dsor_q;
    logic [REM_W-1:0]  rem_q;
    logic [CNT_W-1:0]  cnt_q;
    logic              running_q;

    // one restoring step -- shifts only, no bit/part-selects in the process
    logic [REM_W-1:0]  rem_shift;
    logic              ge;
    logic [REM_W-1:0]  rem_step;
    always_comb begin
        rem_shift = (rem_q << 1) | (dvnd_q >> (DVND_W-1)); // (rem<<1) | MSB(dividend)
        ge        = (rem_shift >= dsor_q);
        rem_step  = ge ? (rem_shift - dsor_q) : rem_shift;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            running_q <= 1'b0; done <= 1'b0;
            dvnd_q <= '0; dsor_q <= '0; quot_q <= '0; rem_q <= '0; cnt_q <= '0;
        end else begin
            done <= 1'b0;
            if (start && !running_q) begin
                dvnd_q    <= dividend;
                dsor_q    <= divisor;
                quot_q    <= '0;
                rem_q     <= '0;
                cnt_q     <= DVND_W;              // plain assign (truncates to CNT_W)
                running_q <= 1'b1;
            end else if (running_q) begin
                rem_q  <= rem_step;
                quot_q <= (quot_q << 1) | ge;     // shift in quotient bit
                dvnd_q <= dvnd_q << 1;            // drop consumed MSB
                cnt_q  <= cnt_q - 1'b1;
                if (cnt_q == 1) begin
                    running_q <= 1'b0;
                    done      <= 1'b1;
                end
            end
        end
    end

    assign busy      = running_q;
    assign quotient  = quot_q;
    assign remainder = rem_q[DSOR_W-1:0];   // continuous assign (allowed)
endmodule
