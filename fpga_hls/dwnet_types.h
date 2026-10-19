#ifndef DWNET_TYPES_H
#define DWNET_TYPES_H

// Raw integer storage types for zero-overhead compile-time ROM initialization in Vitis HLS
typedef signed char weight_raw_t;              // Raw 8-bit two's-complement representation for weight_*_t
typedef int bias_raw_t;                        // Raw 18-bit two's-complement representation for data_t biases

#ifndef DWNET_RAW_WEIGHTS_ONLY
#include <ap_int.h>
#include <ap_fixed.h>
#include <hls_stream.h>

// Custom fixed-point formats to optimize BRAM and accuracy on Kintex-7
typedef ap_fixed<18, 10, AP_RND> data_t;     // Activations/features: 10 integer bits, 8 fractional bits (fits 18-bit BRAM port)
typedef ap_fixed<32, 16, AP_RND> accum_t;    // Intermediate accumulator to prevent overflow
typedef ap_ufixed<9, 3, AP_RND, AP_SAT> dw_buf_t; // 9-bit unsigned [0,6] for ReLU6-clamped DWConv 2-row packed line buffers
typedef ap_fixed<9, 5, AP_RND, AP_SAT> skip_t;    // 9-bit signed [-16,+15.94] for 2-per-word packed U-Net skip FIFOs

typedef ap_fixed<8, 5, AP_RND> weight_t;       // General weights: Stage 0 (max 13.66), conv_last/score (max 2.13)
typedef ap_fixed<8, 6, AP_RND> weight_dw_t;    // DW weights: Backbone/Decoder DW (max 17.93)
typedef ap_fixed<8, 2, AP_RND> weight_pw_t;    // PW weights: Backbone/Decoder PW (max 1.54)
typedef ap_fixed<8, 1, AP_RND> weight_dconv_t; // Dconv weights: Decoder dconv1..4 (max 0.48)

// Primitive bit-accurate integer types (native LLVM i18 / i32 / i14 in Vitis HLS, zero C++ class/DWARF overhead)
typedef data_t::Base::DataType raw18_t;
typedef accum_t::Base::DataType raw32_t;
typedef ap_uint<18>::Base::DataType raw18u_t;
typedef ap_uint<16>::Base::DataType raw16u_t;
typedef ap_uint<14>::Base::DataType raw14u_t;
#endif // DWNET_RAW_WEIGHTS_ONLY

#endif // DWNET_TYPES_H

