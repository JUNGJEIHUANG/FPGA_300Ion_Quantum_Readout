#include "dwnet_layers.h"

// Standard Conv2D (used for Stage 0 where IN_C=1, OUT_C=16, K=3)
template<int H, int W, int IN_C, int OUT_C, int K, int S, int P, bool ACT, typename W_T>
void stream_conv2d(
    hls::stream<data_t>& in_stream,
    hls::stream<data_t>& out_stream,
    const weight_raw_t weights[OUT_C][IN_C][K][K],
    const bias_raw_t bias[OUT_C]
) {
    #pragma HLS INLINE off

    static const int W_FRAC = W_T::width - W_T::iwidth;
    static const int ROUND  = (1 << (W_FRAC - 1));
    static const int SIX_SC = (1536 << W_FRAC);

    // Line buffers to store K-1 previous rows
    raw18_t line_buf[K-1][W + 2*P][IN_C];
    #pragma HLS array_reshape variable=line_buf complete dim=1
    #pragma HLS array_reshape variable=line_buf complete dim=3
    #pragma HLS bind_storage variable=line_buf type=ram_2p impl=lutram

    // Sliding window to store KxK patch
    raw18_t window[IN_C][K][K];
    #pragma HLS array_partition variable=window complete dim=0

    #pragma HLS array_reshape variable=weights complete dim=2
    #pragma HLS array_reshape variable=weights complete dim=3
    #pragma HLS array_reshape variable=weights complete dim=4
    #pragma HLS bind_storage variable=weights type=rom_1p impl=lutram
    #pragma HLS bind_storage variable=bias type=rom_1p impl=lutram

    L_ROW: for (int r = 0; r < H + 2*P; r++) {
        L_COL: for (int c = 0; c < W + 2*P; c++) {
            L_IN: for (int ch_in = 0; ch_in < IN_C; ch_in++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS dependence variable=line_buf inter false
                raw18_t val = 0;
                if (r >= P && r < H + P && c >= P && c < W + P) {
                    val = in_stream.read().V;
                }

                for (int i = 0; i < K; i++) {
                    #pragma HLS UNROLL
                    for (int j = 0; j < K-1; j++) {
                        #pragma HLS UNROLL
                        window[ch_in][i][j] = window[ch_in][i][j+1];
                    }
                }

                for (int i = 0; i < K-1; i++) {
                    #pragma HLS UNROLL
                    window[ch_in][i][K-1] = line_buf[i][c][ch_in];
                }
                window[ch_in][K-1][K-1] = val;

                for (int i = 0; i < K-2; i++) {
                    #pragma HLS UNROLL
                    line_buf[i][c][ch_in] = line_buf[i+1][c][ch_in];
                }
                if (K > 1) {
                    line_buf[K-2][c][ch_in] = val;
                }
            }

            if (r >= K-1 && c >= K-1) {
                if ((r - K + 1) % S == 0 && (c - K + 1) % S == 0) {
                    L_CH_OUT: for (int ch_out = 0; ch_out < OUT_C; ch_out++) {
                        #pragma HLS PIPELINE II=1
                        int sum = ((int)bias[ch_out]) << W_FRAC;
                        for (int ch_in = 0; ch_in < IN_C; ch_in++) {
                            #pragma HLS UNROLL
                            for (int i = 0; i < K; i++) {
                                #pragma HLS UNROLL
                                for (int j = 0; j < K; j++) {
                                    #pragma HLS UNROLL
                                    sum += (int)window[ch_in][i][j] * (int)weights[ch_out][ch_in][i][j];
                                }
                            }
                        }
                        int rounded = (sum + ROUND) >> W_FRAC;
                        data_t activated;
                        if (ACT) {
                            if (sum < 0) activated.V = 0;
                            else if (sum > SIX_SC) activated.V = 1536;
                            else activated.V = (raw18_t)rounded;
                        } else {
                            activated.V = (raw18_t)rounded;
                        }

                        out_stream.write(activated);
                    }
                }
            }
        }
    }
}

// Depthwise Conv2D with 2-row 7-bit packed 14-bit line buffer, 8-bit window, and power-of-two BRAM/LUTRAM hybrid split
template<int H, int W, int C, int K, int S, int P, bool ACT, typename W_T>
void stream_depthwise_conv2d(
    hls::stream<data_t>& in_stream,
    hls::stream<data_t>& out_stream,
    const weight_raw_t weights[C][K][K],
    const bias_raw_t bias[C]
) {
    #pragma HLS INLINE off

    static const int W_FRAC   = W_T::width - W_T::iwidth;
    static const int ROUND    = (1 << (W_FRAC - 1));
    static const int SIX_SC   = (1536 << W_FRAC);
    static const int TOTAL_SZ = W * C;
    static const int MAIN_SZ  = (TOTAL_SZ >= 32768) ? 32768 :
                                (TOTAL_SZ >= 16384) ? 16384 : 1;
    static const int TAIL_SZ  = (TOTAL_SZ >= 16384) ? (TOTAL_SZ - MAIN_SZ) : TOTAL_SZ;

    // Pack 2 rows of 8-bit ReLU6 activations [0,6] (UQ3.5) into a single 16-bit primitive integer word
    unsigned short line_buf_main[MAIN_SZ];
    unsigned short line_buf_tail[TAIL_SZ];
    #pragma HLS bind_storage variable=line_buf_main type=ram_s2p impl=bram
    #pragma HLS bind_storage variable=line_buf_tail type=ram_s2p impl=lutram

    unsigned char window[C][K][K];
    #pragma HLS array_reshape variable=window complete dim=2
    #pragma HLS array_reshape variable=window complete dim=3
    #pragma HLS bind_storage variable=window type=ram_2p impl=lutram

    #pragma HLS array_reshape variable=weights complete dim=2
    #pragma HLS array_reshape variable=weights complete dim=3
    #pragma HLS bind_storage variable=weights type=rom_1p impl=lutram
    #pragma HLS bind_storage variable=bias type=rom_1p impl=lutram

    L_ROW: for (int r = 0; r < H + 2*P; r++) {
        L_COL: for (int c = 0; c < W + 2*P; c++) {
            L_CH: for (int ch = 0; ch < C; ch++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS dependence variable=line_buf_main inter false
                #pragma HLS dependence variable=line_buf_tail inter false
                #pragma HLS dependence variable=window inter false

                for (int i = 0; i < K; i++) {
                    #pragma HLS UNROLL
                    for (int j = 0; j < K-1; j++) {
                        #pragma HLS UNROLL
                        window[ch][i][j] = window[ch][i][j+1];
                    }
                }

                if (c >= P && c < W + P) {
                    raw18_t val = 0;
                    if (r >= P && r < H + P) {
                        val = in_stream.read().V;
                    }

                    int addr = (c - P) * C + ch;
                    unsigned int packed_rd;
                    if (TOTAL_SZ >= 16384) {
                        packed_rd = (unsigned int)((addr < MAIN_SZ) ? line_buf_main[addr] : line_buf_tail[addr - MAIN_SZ]);
                    } else {
                        packed_rd = (unsigned int)line_buf_tail[addr];
                    }

                    unsigned int lb0 = packed_rd & 0xFFU;
                    unsigned int lb1 = (packed_rd >> 8) & 0xFFU;

                    int v_rnd = ((int)val + 4) >> 3;
                    unsigned int val_8b = (v_rnd <= 0) ? 0U : (v_rnd >= 255) ? 255U : (unsigned int)v_rnd;

                    window[ch][0][K-1] = (r >= 2) ? (unsigned char)lb0 : (unsigned char)0;
                    window[ch][1][K-1] = (r >= 1) ? (unsigned char)lb1 : (unsigned char)0;
                    window[ch][2][K-1] = (unsigned char)val_8b;

                    unsigned int new_lo = (r >= 1) ? lb1 : 0U;
                    unsigned short packed_wr = (unsigned short)(new_lo | (val_8b << 8));

                    if (TOTAL_SZ >= 16384) {
                        if (addr < MAIN_SZ) line_buf_main[addr] = packed_wr;
                        else line_buf_tail[addr - MAIN_SZ] = packed_wr;
                    } else {
                        line_buf_tail[addr] = packed_wr;
                    }
                } else {
                    window[ch][0][K-1] = 0;
                    window[ch][1][K-1] = 0;
                    window[ch][2][K-1] = 0;
                }

                if (r >= K-1 && c >= K-1 && (r - K + 1) % S == 0 && (c - K + 1) % S == 0) {
                    int prod_sum = 0;
                    for (int i = 0; i < K; i++) {
                        #pragma HLS UNROLL
                        for (int j = 0; j < K; j++) {
                            #pragma HLS UNROLL
                            prod_sum += (int)window[ch][i][j] * (int)weights[ch][i][j];
                        }
                    }
                    int sum = (((int)bias[ch]) << W_FRAC) + (prod_sum << 3);
                    int rounded = (sum + ROUND) >> W_FRAC;
                    data_t activated;
                    if (ACT) {
                        if (sum < 0) activated.V = 0;
                        else if (sum > SIX_SC) activated.V = 1536;
                        else activated.V = (raw18_t)rounded;
                    } else {
                        activated.V = (raw18_t)rounded;
                    }

                    out_stream.write(activated);
                }
            }
        }
    }
}

// Helper struct for Pointwise Conv2D:
// MODE 0: Small weight matrices (IN_C * OUT_C <= 4096) with UF > 2 -> Reshaped LUTRAM ROM
// MODE 1: Any weight matrix with UF == 2 -> Unpartitioned Dual-Port BRAM ROM (2 native read ports)
// MODE 2: Large weight matrices (IN_C * OUT_C > 4096) with UF > 2 -> Reshaped BRAM ROM
template<int H, int W, int IN_C, int OUT_C, bool ACT, typename W_T, int UF, int MODE>
struct PointwiseRunner;

// MODE 0: Small weight matrices (IN_C * OUT_C <= 4096, UF > 2) -> Reshaped LUTRAM ROM
template<int H, int W, int IN_C, int OUT_C, bool ACT, typename W_T, int UF>
struct PointwiseRunner<H, W, IN_C, OUT_C, ACT, W_T, UF, 0> {
    static void run(
        hls::stream<data_t>& in_stream,
        hls::stream<data_t>& out_stream,
        const weight_raw_t weights[OUT_C][IN_C],
        const bias_raw_t bias[OUT_C]
    ) {
        #pragma HLS INLINE

        static const int W_FRAC = W_T::width - W_T::iwidth;
        static const int ROUND  = (1 << (W_FRAC - 1));
        static const int SIX_SC = (1536 << W_FRAC);
        static const int STEPS  = IN_C / UF;

        raw18_t in_buf[IN_C];
        #pragma HLS array_reshape variable=in_buf cyclic factor=UF dim=1
        #pragma HLS bind_storage variable=in_buf type=ram_2p impl=lutram

        raw32_t acc_buf[OUT_C];
        #pragma HLS bind_storage variable=acc_buf type=ram_2p impl=lutram

        #pragma HLS array_reshape variable=weights cyclic factor=UF dim=2
        #pragma HLS bind_storage variable=weights type=rom_1p impl=lutram
        #pragma HLS bind_storage variable=bias type=rom_1p impl=lutram

        L_PIXEL: for (int p = 0; p < H * W; p++) {
            L_LOAD: for (int s = 0; s < STEPS; s++) {
                #pragma HLS PIPELINE II=UF
                for (int u = 0; u < UF; u++) {
                    #pragma HLS UNROLL
                    in_buf[s * UF + u] = in_stream.read().V;
                }
            }

            L_STEP: for (int s = 0; s < STEPS; s++) {
                L_CH_OUT: for (int ch_out = 0; ch_out < OUT_C; ch_out++) {
                    #pragma HLS PIPELINE II=1
                    #pragma HLS dependence variable=acc_buf inter false
                    int prev = (s == 0) ? (((int)bias[ch_out]) << W_FRAC) : (int)acc_buf[ch_out];
                    int partial = 0;
                    L_UNROLL: for (int u = 0; u < UF; u++) {
                        #pragma HLS UNROLL
                        int ch_in = s * UF + u;
                        partial += (int)in_buf[ch_in] * (int)weights[ch_out][ch_in];
                    }
                    int sum = prev + partial;
                    acc_buf[ch_out] = (raw32_t)sum;
                    if (s == STEPS - 1) {
                        int rounded = (sum + ROUND) >> W_FRAC;
                        data_t activated;
                        if (ACT) {
                            if (sum < 0) activated.V = 0;
                            else if (sum > SIX_SC) activated.V = 1536;
                            else activated.V = (raw18_t)rounded;
                        } else {
                            activated.V = (raw18_t)rounded;
                        }
                        out_stream.write(activated);
                    }
                }
            }
        }
    }
};

// MODE 1: UF == 2 -> Unpartitioned Dual-Port ROM/RAM (uses 2 native ports per cycle with ZERO array_partition/reshape)
template<int H, int W, int IN_C, int OUT_C, bool ACT, typename W_T, int UF>
struct PointwiseRunner<H, W, IN_C, OUT_C, ACT, W_T, UF, 1> {
    static void run(
        hls::stream<data_t>& in_stream,
        hls::stream<data_t>& out_stream,
        const weight_raw_t weights[OUT_C][IN_C],
        const bias_raw_t bias[OUT_C]
    ) {
        #pragma HLS INLINE

        static const int W_FRAC = W_T::width - W_T::iwidth;
        static const int ROUND  = (1 << (W_FRAC - 1));
        static const int SIX_SC = (1536 << W_FRAC);
        static const int STEPS  = IN_C / 2;

        raw18_t in_buf[IN_C];
        #pragma HLS bind_storage variable=in_buf type=ram_2p impl=lutram

        raw32_t acc_buf[OUT_C];
        #pragma HLS bind_storage variable=acc_buf type=ram_2p impl=lutram

        #pragma HLS bind_storage variable=weights type=rom_2p impl=bram
        #pragma HLS bind_storage variable=bias type=rom_1p impl=lutram

        L_PIXEL: for (int p = 0; p < H * W; p++) {
            L_LOAD: for (int ch_in = 0; ch_in < IN_C; ch_in++) {
                #pragma HLS PIPELINE II=1
                in_buf[ch_in] = in_stream.read().V;
            }

            L_STEP: for (int s = 0; s < STEPS; s++) {
                L_CH_OUT: for (int ch_out = 0; ch_out < OUT_C; ch_out++) {
                    #pragma HLS PIPELINE II=1
                    #pragma HLS dependence variable=acc_buf inter false
                    int prev = (s == 0) ? (((int)bias[ch_out]) << W_FRAC) : (int)acc_buf[ch_out];
                    int c0 = s * 2;
                    int c1 = s * 2 + 1;
                    int sum = prev + (int)in_buf[c0] * (int)weights[ch_out][c0] + (int)in_buf[c1] * (int)weights[ch_out][c1];
                    acc_buf[ch_out] = (raw32_t)sum;
                    if (s == STEPS - 1) {
                        int rounded = (sum + ROUND) >> W_FRAC;
                        data_t activated;
                        if (ACT) {
                            if (sum < 0) activated.V = 0;
                            else if (sum > SIX_SC) activated.V = 1536;
                            else activated.V = (raw18_t)rounded;
                        } else {
                            activated.V = (raw18_t)rounded;
                        }
                        out_stream.write(activated);
                    }
                }
            }
        }
    }
};

// MODE 2: Large weight matrices (IN_C * OUT_C > 4096) with UF > 2 -> Reshaped Wide BRAM ROM (1 single array per layer)
template<int H, int W, int IN_C, int OUT_C, bool ACT, typename W_T, int UF>
struct PointwiseRunner<H, W, IN_C, OUT_C, ACT, W_T, UF, 2> {
    static void run(
        hls::stream<data_t>& in_stream,
        hls::stream<data_t>& out_stream,
        const weight_raw_t weights[OUT_C][IN_C],
        const bias_raw_t bias[OUT_C]
    ) {
        #pragma HLS INLINE

        static const int W_FRAC = W_T::width - W_T::iwidth;
        static const int ROUND  = (1 << (W_FRAC - 1));
        static const int SIX_SC = (1536 << W_FRAC);
        static const int STEPS  = IN_C / UF;

        raw18_t in_buf[IN_C];
        #pragma HLS array_reshape variable=in_buf cyclic factor=UF dim=1
        #pragma HLS bind_storage variable=in_buf type=ram_2p impl=lutram

        raw32_t acc_buf[OUT_C];
        #pragma HLS bind_storage variable=acc_buf type=ram_2p impl=lutram

        #pragma HLS array_reshape variable=weights cyclic factor=UF dim=2
        #pragma HLS bind_storage variable=weights type=rom_1p impl=bram
        #pragma HLS bind_storage variable=bias type=rom_1p impl=lutram

        L_PIXEL: for (int p = 0; p < H * W; p++) {
            L_LOAD: for (int s = 0; s < STEPS; s++) {
                #pragma HLS PIPELINE II=UF
                for (int u = 0; u < UF; u++) {
                    #pragma HLS UNROLL
                    in_buf[s * UF + u] = in_stream.read().V;
                }
            }

            L_STEP: for (int s = 0; s < STEPS; s++) {
                L_CH_OUT: for (int ch_out = 0; ch_out < OUT_C; ch_out++) {
                    #pragma HLS PIPELINE II=1
                    #pragma HLS dependence variable=acc_buf inter false
                    int prev = (s == 0) ? (((int)bias[ch_out]) << W_FRAC) : (int)acc_buf[ch_out];
                    int partial = 0;
                    L_UNROLL: for (int u = 0; u < UF; u++) {
                        #pragma HLS UNROLL
                        int ch_in = s * UF + u;
                        partial += (int)in_buf[ch_in] * (int)weights[ch_out][ch_in];
                    }
                    int sum = prev + partial;
                    acc_buf[ch_out] = (raw32_t)sum;
                    if (s == STEPS - 1) {
                        int rounded = (sum + ROUND) >> W_FRAC;
                        data_t activated;
                        if (ACT) {
                            if (sum < 0) activated.V = 0;
                            else if (sum > SIX_SC) activated.V = 1536;
                            else activated.V = (raw18_t)rounded;
                        } else {
                            activated.V = (raw18_t)rounded;
                        }
                        out_stream.write(activated);
                    }
                }
            }
        }
    }
};

template<int H, int W, int IN_C, int OUT_C, bool ACT, typename W_T, int UF>
void stream_pointwise_conv2d(
    hls::stream<data_t>& in_stream,
    hls::stream<data_t>& out_stream,
    const weight_raw_t weights[OUT_C][IN_C],
    const bias_raw_t bias[OUT_C]
) {
    #pragma HLS INLINE off
    static const int MODE = (UF == 2) ? 1 : ((IN_C * OUT_C <= 4096) ? 0 : 2);
    PointwiseRunner<H, W, IN_C, OUT_C, ACT, W_T, UF, MODE>::run(
        in_stream, out_stream, weights, bias
    );
}

// Helper struct for ConvTranspose2D to bind small weight ROMs to LUTRAM and large weight ROMs to Dual-Port BRAM
template<int H, int W, int IN_C, int OUT_C, int K, int S, int P, typename W_T, int UF, bool USE_LUTRAM>
struct TransposeRunner;

// Specialization 1: Small/Medium transpose conv weights (dconv2, dconv3, dconv4) -> Reshaped LUTRAM ROM
template<int H, int W, int IN_C, int OUT_C, int K, int S, int P, typename W_T, int UF>
struct TransposeRunner<H, W, IN_C, OUT_C, K, S, P, W_T, UF, true> {
    static void run(
        hls::stream<data_t>& in_stream,
        hls::stream<data_t>& out_stream,
        const weight_raw_t weights[IN_C][OUT_C][K][K],
        const bias_raw_t bias[OUT_C]
    ) {
        #pragma HLS INLINE

        static const int W_FRAC = W_T::width - W_T::iwidth;
        static const int ROUND  = (1 << (W_FRAC - 1));
        static const int STEPS  = IN_C / UF;

        raw18_t row_buf[4][W * S][OUT_C];
        #pragma HLS bind_storage variable=row_buf type=ram_s2p impl=lutram

        raw18_t in_val[IN_C];
        #pragma HLS array_reshape variable=in_val cyclic factor=UF dim=1
        #pragma HLS bind_storage variable=in_val type=ram_2p impl=lutram

        #pragma HLS array_reshape variable=weights cyclic factor=UF dim=1
        #pragma HLS bind_storage variable=weights type=rom_1p impl=lutram
        #pragma HLS bind_storage variable=bias type=rom_1p impl=lutram

        L_ROW: for (int r = 0; r < H; r++) {
            int init_start = (r == 0) ? 0 : (2 * r + 1);
            int init_end   = (2 * r + 2 < H * S) ? (2 * r + 2) : (H * S - 1);
            L_INIT_R: for (int ir = init_start; ir <= init_end; ir++) {
                #pragma HLS LOOP_TRIPCOUNT min=1 max=2
                L_INIT_C: for (int ic = 0; ic < W * S; ic++) {
                    L_INIT_CH: for (int ch_out = 0; ch_out < OUT_C; ch_out++) {
                        #pragma HLS PIPELINE II=1
                        row_buf[ir & 3][ic][ch_out] = (raw18_t)bias[ch_out];
                    }
                }
            }

            L_COL: for (int c = 0; c < W; c++) {
                L_IN: for (int s = 0; s < STEPS; s++) {
                    #pragma HLS PIPELINE II=UF
                    for (int u = 0; u < UF; u++) {
                        #pragma HLS UNROLL
                        in_val[s * UF + u] = in_stream.read().V;
                    }
                }

                L_KR: for (int kr = 0; kr < K; kr++) {
                    int out_r = r * S + kr - P;
                    if (out_r >= 0 && out_r < H * S) {
                        L_KC: for (int kc = 0; kc < K; kc++) {
                            int out_c = c * S + kc - P;
                            if (out_c >= 0 && out_c < W * S) {
                                L_STEP: for (int s = 0; s < STEPS; s++) {
                                    L_CH_OUT: for (int ch_out = 0; ch_out < OUT_C; ch_out++) {
                                        #pragma HLS PIPELINE II=1
                                        #pragma HLS dependence variable=row_buf inter false
                                        int partial = 0;
                                        L_UNROLL: for (int u = 0; u < UF; u++) {
                                            #pragma HLS UNROLL
                                            int ch_in = s * UF + u;
                                            partial += (int)in_val[ch_in] * (int)weights[ch_in][ch_out][kr][kc];
                                        }
                                        row_buf[out_r & 3][out_c][ch_out] += (raw18_t)((partial + ROUND) >> W_FRAC);
                                    }
                                }
                            }
                        }
                    }
                }
            }

            int flush_start = (r == 0) ? 0 : (2 * r - 1);
            int flush_end   = (r == H - 1) ? (H * S - 1) : (2 * r);
            L_FLUSH_R: for (int fr = flush_start; fr <= flush_end; fr++) {
                #pragma HLS LOOP_TRIPCOUNT min=1 max=2
                L_FLUSH_C: for (int fc = 0; fc < W * S; fc++) {
                    L_FLUSH_CH: for (int ch_out = 0; ch_out < OUT_C; ch_out++) {
                        #pragma HLS PIPELINE II=1
                        data_t out_v;
                        out_v.V = row_buf[fr & 3][fc][ch_out];
                        out_stream.write(out_v);
                    }
                }
            }
        }
    }
};

// Specialization 2: Large transpose conv weights (dconv1: 491,520 weights) -> Unpartitioned Dual-Port BRAM ROM (2 native read ports)
template<int H, int W, int IN_C, int OUT_C, int K, int S, int P, typename W_T, int UF>
struct TransposeRunner<H, W, IN_C, OUT_C, K, S, P, W_T, UF, false> {
    static void run(
        hls::stream<data_t>& in_stream,
        hls::stream<data_t>& out_stream,
        const weight_raw_t weights[IN_C][OUT_C][K][K],
        const bias_raw_t bias[OUT_C]
    ) {
        #pragma HLS INLINE

        static const int W_FRAC = W_T::width - W_T::iwidth;
        static const int ROUND  = (1 << (W_FRAC - 1));
        static const int STEPS  = IN_C / 2;

        raw18_t row_buf[4][W * S][OUT_C];
        #pragma HLS bind_storage variable=row_buf type=ram_s2p impl=lutram

        raw18_t in_val[IN_C];
        #pragma HLS bind_storage variable=in_val type=ram_2p impl=lutram

        #pragma HLS bind_storage variable=weights type=rom_2p impl=bram
        #pragma HLS bind_storage variable=bias type=rom_1p impl=lutram

        L_ROW: for (int r = 0; r < H; r++) {
            int init_start = (r == 0) ? 0 : (2 * r + 1);
            int init_end   = (2 * r + 2 < H * S) ? (2 * r + 2) : (H * S - 1);
            L_INIT_R: for (int ir = init_start; ir <= init_end; ir++) {
                #pragma HLS LOOP_TRIPCOUNT min=1 max=2
                L_INIT_C: for (int ic = 0; ic < W * S; ic++) {
                    L_INIT_CH: for (int ch_out = 0; ch_out < OUT_C; ch_out++) {
                        #pragma HLS PIPELINE II=1
                        row_buf[ir & 3][ic][ch_out] = (raw18_t)bias[ch_out];
                    }
                }
            }

            L_COL: for (int c = 0; c < W; c++) {
                L_IN: for (int ch_in = 0; ch_in < IN_C; ch_in++) {
                    #pragma HLS PIPELINE II=1
                    in_val[ch_in] = in_stream.read().V;
                }

                L_KR: for (int kr = 0; kr < K; kr++) {
                    int out_r = r * S + kr - P;
                    if (out_r >= 0 && out_r < H * S) {
                        L_KC: for (int kc = 0; kc < K; kc++) {
                            int out_c = c * S + kc - P;
                            if (out_c >= 0 && out_c < W * S) {
                                L_STEP: for (int s = 0; s < STEPS; s++) {
                                    L_CH_OUT: for (int ch_out = 0; ch_out < OUT_C; ch_out++) {
                                        #pragma HLS PIPELINE II=1
                                        #pragma HLS dependence variable=row_buf inter false
                                        int c0 = s * 2;
                                        int c1 = s * 2 + 1;
                                        int partial = (int)in_val[c0] * (int)weights[c0][ch_out][kr][kc] +
                                                      (int)in_val[c1] * (int)weights[c1][ch_out][kr][kc];
                                        row_buf[out_r & 3][out_c][ch_out] += (raw18_t)((partial + ROUND) >> W_FRAC);
                                    }
                                }
                            }
                        }
                    }
                }
            }

            int flush_start = (r == 0) ? 0 : (2 * r - 1);
            int flush_end   = (r == H - 1) ? (H * S - 1) : (2 * r);
            L_FLUSH_R: for (int fr = flush_start; fr <= flush_end; fr++) {
                #pragma HLS LOOP_TRIPCOUNT min=1 max=2
                L_FLUSH_C: for (int fc = 0; fc < W * S; fc++) {
                    L_FLUSH_CH: for (int ch_out = 0; ch_out < OUT_C; ch_out++) {
                        #pragma HLS PIPELINE II=1
                        data_t out_v;
                        out_v.V = row_buf[fr & 3][fc][ch_out];
                        out_stream.write(out_v);
                    }
                }
            }
        }
    }
};

template<int H, int W, int IN_C, int OUT_C, int K, int S, int P, typename W_T, int UF>
void stream_conv_transpose2d(
    hls::stream<data_t>& in_stream,
    hls::stream<data_t>& out_stream,
    const weight_raw_t weights[IN_C][OUT_C][K][K],
    const bias_raw_t bias[OUT_C]
) {
    #pragma HLS INLINE off
    TransposeRunner<H, W, IN_C, OUT_C, K, S, P, W_T, UF, (IN_C * OUT_C * K * K <= 16384)>::run(
        in_stream, out_stream, weights, bias
    );
}

// Duplicate a stream (for short residual bypasses)
template<int N>
void duplicate_stream(
    hls::stream<data_t>& in,
    hls::stream<data_t>& out1,
    hls::stream<data_t>& out2
) {
    #pragma HLS INLINE off
    for (int i = 0; i < N; i++) {
        #pragma HLS PIPELINE II=1
        data_t val = in.read();
        out1.write(val);
        out2.write(val);
    }
}

// Duplicate and pack 2x 7-bit skip values per 14-bit FIFO word across power-of-two split FIFOs
template<int N, int SPLIT>
void duplicate_skip(
    hls::stream<data_t>& in,
    hls::stream<ap_uint<14>>& skip_a,
    hls::stream<ap_uint<14>>& skip_b,
    hls::stream<data_t>& out_conv
) {
    #pragma HLS INLINE off
    L_DUP_SKIP: for (int i = 0; i < N / 2; i++) {
        #pragma HLS PIPELINE II=2
        data_t v0 = in.read();
        out_conv.write(v0);
        data_t v1 = in.read();
        out_conv.write(v1);

        static const int SHIFT = (N >= 120384) ? 4 : 5;
        static const int RND   = (1 << (SHIFT - 1));
        int s0 = ((int)v0.V + RND) >> SHIFT;
        if (s0 > 63) s0 = 63;
        else if (s0 < -64) s0 = -64;

        int s1 = ((int)v1.V + RND) >> SHIFT;
        if (s1 > 63) s1 = 63;
        else if (s1 < -64) s1 = -64;

        unsigned int lo = ((unsigned int)s0) & 0x7FU;
        unsigned int hi = ((unsigned int)s1) & 0x7FU;
        ap_uint<14> packed;
        packed.V = (raw14u_t)(lo | (hi << 7));
        if (i < SPLIT) {
            skip_a.write(packed);
        } else {
            skip_b.write(packed);
        }
    }
}

// Concatenate two streams
template<int H, int W, int C>
void concat_streams(
    hls::stream<data_t>& in1,
    hls::stream<data_t>& in2,
    hls::stream<data_t>& out
) {
    #pragma HLS INLINE off
    for (int i = 0; i < H * W; i++) {
        L_C1: for (int ch = 0; ch < C; ch++) {
            #pragma HLS PIPELINE II=1
            out.write(in1.read());
        }
        L_C2: for (int ch = 0; ch < C; ch++) {
            #pragma HLS PIPELINE II=1
            out.write(in2.read());
        }
    }
}

// Unpack 2x 7-bit skip values from split FIFOs and concatenate with decoder upsampled stream
template<int H, int W, int C, int SPLIT>
void concat_skip(
    hls::stream<ap_uint<14>>& skip_a,
    hls::stream<ap_uint<14>>& skip_b,
    hls::stream<data_t>& in2,
    hls::stream<data_t>& out
) {
    #pragma HLS INLINE off
    static const int USHIFT = (H * W * C >= 120384) ? 21 : 20;
    int idx = 0;
    L_PIX: for (int i = 0; i < H * W; i++) {
        L_C1: for (int ch = 0; ch < C / 2; ch++) {
            #pragma HLS PIPELINE II=2
            ap_uint<14> packed = (idx < SPLIT) ? skip_a.read() : skip_b.read();
            idx++;
            unsigned int raw = (unsigned int)packed.V;
            unsigned int u0 = raw & 0x7FU;
            unsigned int u1 = (raw >> 7) & 0x7FU;
            int s0 = ((int)(u0 << 25)) >> USHIFT;
            int s1 = ((int)(u1 << 25)) >> USHIFT;
            data_t v0, v1;
            v0.V = (raw18_t)s0;
            v1.V = (raw18_t)s1;
            out.write(v0);
            out.write(v1);
        }
        L_C2: for (int ch = 0; ch < C; ch++) {
            #pragma HLS PIPELINE II=1
            out.write(in2.read());
        }
    }
}

// Add two streams element-wise
template<int H, int W, int C>
void stream_add(
    hls::stream<data_t>& in1,
    hls::stream<data_t>& in2,
    hls::stream<data_t>& out
) {
    #pragma HLS INLINE off
    for (int i = 0; i < H * W * C; i++) {
        #pragma HLS PIPELINE II=1
        data_t v;
        v.V = in1.read().V + in2.read().V;
        out.write(v);
    }
}

// Bilinear resize with 2-row rolling LUTRAM buffer and pure integer Q14 interpolation (0 BRAM_18K, 0 ap_ufixed templates)
template<int H_IN, int W_IN, int H_OUT, int W_OUT, int C>
void stream_bilinear_resize(
    hls::stream<data_t>& in_stream,
    hls::stream<data_t>& out_stream
) {
    #pragma HLS INLINE off

    raw18_t row_buf[2][W_IN][C];
    #pragma HLS array_partition variable=row_buf complete dim=1
    #pragma HLS bind_storage variable=row_buf type=ram_2p impl=lutram

    int loaded_r = -1;

    L_ROW: for (int r = 0; r < H_OUT; r++) {
        int r_num = r * (H_IN - 1);
        int r_low = r_num / (H_OUT - 1);
        int r_rem = r_num - r_low * (H_OUT - 1);
        int r_high = (r_low + 1 < H_IN) ? (r_low + 1) : (H_IN - 1);

        // Stream in new input rows on demand into the 2-row circular buffer
        L_LOAD_R: while (loaded_r < r_high) {
            #pragma HLS LOOP_TRIPCOUNT min=1 max=2
            loaded_r++;
            L_RD_C: for (int ic = 0; ic < W_IN; ic++) {
                L_RD_CH: for (int ch = 0; ch < C; ch++) {
                    #pragma HLS PIPELINE II=1
                    row_buf[loaded_r & 1][ic][ch] = in_stream.read().V;
                }
            }
        }

        int wr     = ((r_rem << 15) + (H_OUT - 1)) / (2 * (H_OUT - 1));
        int inv_wr = 16384 - wr;

        L_COL: for (int c = 0; c < W_OUT; c++) {
            int c_num = c * (W_IN - 1);
            int c_low = c_num / (W_OUT - 1);
            int c_rem = c_num - c_low * (W_OUT - 1);
            int c_high = (c_low + 1 < W_IN) ? (c_low + 1) : (W_IN - 1);

            int wc     = ((c_rem << 15) + (W_OUT - 1)) / (2 * (W_OUT - 1));
            int inv_wc = 16384 - wc;

            int w00 = (inv_wr * inv_wc + 8192) >> 14;
            int w01 = (inv_wr * wc     + 8192) >> 14;
            int w10 = (wr     * inv_wc + 8192) >> 14;
            int w11 = (wr     * wc     + 8192) >> 14;

            L_CH: for (int ch = 0; ch < C; ch++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS dependence variable=row_buf inter false
                raw18_t b0_c0 = row_buf[0][c_low][ch];
                raw18_t b0_c1 = row_buf[0][c_high][ch];
                raw18_t b1_c0 = row_buf[1][c_low][ch];
                raw18_t b1_c1 = row_buf[1][c_high][ch];

                int val_00 = (int)((r_low & 1)  ? b1_c0 : b0_c0);
                int val_01 = (int)((r_low & 1)  ? b1_c1 : b0_c1);
                int val_10 = (int)((r_high & 1) ? b1_c0 : b0_c0);
                int val_11 = (int)((r_high & 1) ? b1_c1 : b0_c1);

                int acc = w00 * val_00 + w01 * val_01 + w10 * val_10 + w11 * val_11;
                data_t out_val;
                out_val.V = (raw18_t)((acc + 8192) >> 14);
                out_stream.write(out_val);
            }
        }
    }
}
