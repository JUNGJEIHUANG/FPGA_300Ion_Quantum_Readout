#ifndef DWNET_LAYERS_H
#define DWNET_LAYERS_H

#include "dwnet_types.h"
#include "dwnet_config.h"

// Compile-time resolution-aware DSP unroll factor calculators for Pareto-optimal latency/area
constexpr int calc_pw_uf(int h, int w, int ic, int oc) {
    return (h * w >= 40000) ? ((ic >= 96) ? 32 : ((ic >= 16) ? 16 : ic)) :
           (h * w >= 10000) ? ((ic >= 144) ? 16 : ((ic == 24) ? 24 : ((ic >= 12) ? 6 : ic))) :
           (h * w >= 2500)  ? ((ic >= 192) ? 4 : ((ic >= 32 && oc >= 192) ? 8 : 2)) : 2;
}

constexpr int calc_dconv_uf(int h, int w) {
    return (h * w >= 10000) ? 12 :
           (h * w >= 2500)  ? 4  :
           (h * w >= 600)   ? 4  : 2;
}

template<int H, int W, int IN_C, int OUT_C, int K, int S, int P, bool ACT = true, typename W_T = weight_t>
void stream_conv2d(
    hls::stream<data_t>& in_stream,
    hls::stream<data_t>& out_stream,
    const weight_raw_t weights[OUT_C][IN_C][K][K],
    const bias_raw_t bias[OUT_C]
);

template<int H, int W, int C, int K, int S, int P, bool ACT = true, typename W_T = weight_dw_t>
void stream_depthwise_conv2d(
    hls::stream<data_t>& in_stream,
    hls::stream<data_t>& out_stream,
    const weight_raw_t weights[C][K][K],
    const bias_raw_t bias[C]
);

template<int H, int W, int IN_C, int OUT_C, bool ACT = true, typename W_T = weight_pw_t, int UF = calc_pw_uf(H, W, IN_C, OUT_C)>
void stream_pointwise_conv2d(
    hls::stream<data_t>& in_stream,
    hls::stream<data_t>& out_stream,
    const weight_raw_t weights[OUT_C][IN_C],
    const bias_raw_t bias[OUT_C]
);

template<int H, int W, int IN_C, int OUT_C, int K, int S, int P, typename W_T = weight_dconv_t, int UF = calc_dconv_uf(H, W)>
void stream_conv_transpose2d(
    hls::stream<data_t>& in_stream,
    hls::stream<data_t>& out_stream,
    const weight_raw_t weights[IN_C][OUT_C][K][K],
    const bias_raw_t bias[OUT_C]
);

template<int N>
void duplicate_stream(
    hls::stream<data_t>& in,
    hls::stream<data_t>& out1,
    hls::stream<data_t>& out2
);

template<int N, int SPLIT>
void duplicate_skip(
    hls::stream<data_t>& in,
    hls::stream<ap_uint<14>>& skip_a,
    hls::stream<ap_uint<14>>& skip_b,
    hls::stream<data_t>& out_conv
);

template<int H, int W, int C>
void concat_streams(
    hls::stream<data_t>& in1,
    hls::stream<data_t>& in2,
    hls::stream<data_t>& out
);

template<int H, int W, int C, int SPLIT>
void concat_skip(
    hls::stream<ap_uint<14>>& skip_a,
    hls::stream<ap_uint<14>>& skip_b,
    hls::stream<data_t>& in2,
    hls::stream<data_t>& out
);

template<int H, int W, int C>
void stream_add(
    hls::stream<data_t>& in1,
    hls::stream<data_t>& in2,
    hls::stream<data_t>& out
);

template<int H_IN, int W_IN, int H_OUT, int W_OUT, int C>
void stream_bilinear_resize(
    hls::stream<data_t>& in_stream,
    hls::stream<data_t>& out_stream
);

#endif // DWNET_LAYERS_H

