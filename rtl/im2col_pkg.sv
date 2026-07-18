// ============================================================================
// im2col_pkg.sv
// ----------------------------------------------------------------------------
// Single source of truth for the precision -> dimension math used across the
// im2col DOU. All functions are elaboration-time (usable in localparam / port
// width contexts). Mirrors the exact formulas in tb_conv_top.sv so RTL and
// golden model agree by construction.
//
// Precision codes (match tb DATA_PRECISION / CONFIG Data_type):
//     0=INT8  1=FP16  2=INT32  3=INT16  4=BF16  5=FP32
// ============================================================================
`ifndef IM2COL_PKG_SV
`define IM2COL_PKG_SV

package im2col_pkg;

    // ---- precision code names -------------------------------------------
    localparam int PREC_INT8  = 0;
    localparam int PREC_FP16  = 1;
    localparam int PREC_INT32 = 2;
    localparam int PREC_INT16 = 3;
    localparam int PREC_BF16  = 4;
    localparam int PREC_FP32  = 5;

    // ---- bytes per element ----------------------------------------------
    function automatic int bytes_per_elem(input int prec);
        if (prec == PREC_FP32 || prec == PREC_INT32)                    bytes_per_elem = 4;
        else if (prec == PREC_FP16 || prec == PREC_INT16 || prec == PREC_BF16) bytes_per_elem = 2;
        else                                                            bytes_per_elem = 1;
    endfunction

    // ---- column packing factor (tb SIMD_SCALE) --------------------------
    function automatic int simd_scale(input int prec);
        simd_scale = (bytes_per_elem(prec) == 1) ? 4 : 2;
    endfunction

    // ---- row scaling factor (tb DT_16) ----------------------------------
    function automatic int dt_16(input int prec);
        dt_16 = (prec == PREC_FP16 || prec == PREC_INT16 || prec == PREC_BF16) ? 2 : 1;
    endfunction

    // ---- beats per row of a block (1 for INT8/16, 2 for the 32-bit fork) -
    function automatic int beats_per_row(input int prec);
        beats_per_row = bytes_per_elem(prec) / 2 == 2 ? 2 : 1;  // 4B->2, else 1
    endfunction

    // ---- tile dimensions -------------------------------------------------
    // output positions covered by one tile (tb y_size)
    function automatic int outpos_per_tile(input int sa, input int prec);
        outpos_per_tile = sa * simd_scale(prec) * dt_16(prec);
    endfunction
    // reduction (crs) values packed into one beat
    function automatic int red_per_beat(input int sa, input int prec);
        red_per_beat = sa * simd_scale(prec);
    endfunction

    // ---- staging tile stride (tb sram_tile_stride) ----------------------
    function automatic int sram_tile_stride(input int sa, input int kn_depth, input int num_banks);
        sram_tile_stride = sa * kn_depth * num_banks;
    endfunction

    // ---- output-map dimension: W_OUT / H_OUT ----------------------------
    // (caller supplies stride > 0; standard conv output-size formula)
    function automatic int out_dim(input int in_dim, input int pad_lo,
                                   input int pad_hi, input int ker, input int stride);
        out_dim = (in_dim + pad_lo + pad_hi - ker) / stride + 1;
    endfunction

endpackage

`endif
