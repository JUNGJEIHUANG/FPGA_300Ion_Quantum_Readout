#include "dwnet_unet.h"
#include "dwnet_layers.h"
#include "dwnet_weights.h"

// Explicit template instantiations for HLS compiler
#include "dwnet_layers.cpp"



void dwnet_unet(
    hls::stream<data_t>& image_in,
    hls::stream<IonCenter>& centers_out,
    int& ion_count
) {
    #pragma HLS INTERFACE axis port=image_in
    #pragma HLS INTERFACE axis port=centers_out
    #pragma HLS INTERFACE s_axilite port=ion_count bundle=ctrl
    #pragma HLS INTERFACE s_axilite port=return bundle=ctrl

    #pragma HLS DATAFLOW

    // U-Net skip streams packed 2x 7-bit per 14-bit word and split at power-of-two boundaries to save 62 BRAM_18K
    hls::stream<ap_uint<14>> s_x1_skip_a("s_x1_skip_a");
    #pragma HLS stream variable=s_x1_skip_a depth=131072
    #pragma HLS bind_storage variable=s_x1_skip_a type=fifo impl=bram
    hls::stream<ap_uint<14>> s_x1_skip_b("s_x1_skip_b");
    #pragma HLS stream variable=s_x1_skip_b depth=29440
    #pragma HLS bind_storage variable=s_x1_skip_b type=fifo impl=bram

    hls::stream<ap_uint<14>> s_x2_skip_a("s_x2_skip_a");
    #pragma HLS stream variable=s_x2_skip_a depth=32768
    #pragma HLS bind_storage variable=s_x2_skip_a type=fifo impl=bram
    hls::stream<ap_uint<14>> s_x2_skip_b("s_x2_skip_b");
    #pragma HLS stream variable=s_x2_skip_b depth=27424
    #pragma HLS bind_storage variable=s_x2_skip_b type=fifo impl=bram

    hls::stream<ap_uint<14>> s_x3_skip_a("s_x3_skip_a");
    #pragma HLS stream variable=s_x3_skip_a depth=16384
    #pragma HLS bind_storage variable=s_x3_skip_a type=fifo impl=bram
    hls::stream<ap_uint<14>> s_x3_skip_b("s_x3_skip_b");
    #pragma HLS stream variable=s_x3_skip_b depth=3680
    #pragma HLS bind_storage variable=s_x3_skip_b type=fifo impl=lutram

    hls::stream<ap_uint<14>> s_x4_skip_a("s_x4_skip_a");
    #pragma HLS stream variable=s_x4_skip_a depth=8192
    #pragma HLS bind_storage variable=s_x4_skip_a type=fifo impl=bram
    hls::stream<ap_uint<14>> s_x4_skip_b("s_x4_skip_b");
    #pragma HLS stream variable=s_x4_skip_b depth=6856
    #pragma HLS bind_storage variable=s_x4_skip_b type=fifo impl=lutram

    hls::stream<data_t> s_conv0("s_conv0");
    #pragma HLS stream variable=s_conv0 depth=2
    #pragma HLS bind_storage variable=s_conv0 type=fifo impl=srl
    hls::stream<data_t> s_x1_conv("s_x1_conv");
    #pragma HLS stream variable=s_x1_conv depth=2
    #pragma HLS bind_storage variable=s_x1_conv type=fifo impl=srl
    hls::stream<data_t> s_feat1_dw("s_feat1_dw");
    #pragma HLS stream variable=s_feat1_dw depth=2
    #pragma HLS bind_storage variable=s_feat1_dw type=fifo impl=srl
    hls::stream<data_t> s_feat1_pw("s_feat1_pw");
    #pragma HLS stream variable=s_feat1_pw depth=2
    #pragma HLS bind_storage variable=s_feat1_pw type=fifo impl=srl
    hls::stream<data_t> s_feat2_pw1("s_feat2_pw1");
    #pragma HLS stream variable=s_feat2_pw1 depth=2
    #pragma HLS bind_storage variable=s_feat2_pw1 type=fifo impl=srl
    hls::stream<data_t> s_feat2_dw("s_feat2_dw");
    #pragma HLS stream variable=s_feat2_dw depth=2
    #pragma HLS bind_storage variable=s_feat2_dw type=fifo impl=srl
    hls::stream<data_t> s_feat2_pw2("s_feat2_pw2");
    #pragma HLS stream variable=s_feat2_pw2 depth=2
    #pragma HLS bind_storage variable=s_feat2_pw2 type=fifo impl=srl
    hls::stream<data_t> s_feat3_in_bypass("s_feat3_in_bypass");
    #pragma HLS stream variable=s_feat3_in_bypass depth=912
    #pragma HLS bind_storage variable=s_feat3_in_bypass type=fifo impl=lutram
    hls::stream<data_t> s_feat3_in_conv("s_feat3_in_conv");
    #pragma HLS stream variable=s_feat3_in_conv depth=2
    #pragma HLS bind_storage variable=s_feat3_in_conv type=fifo impl=srl
    hls::stream<data_t> s_feat3_pw1("s_feat3_pw1");
    #pragma HLS stream variable=s_feat3_pw1 depth=2
    #pragma HLS bind_storage variable=s_feat3_pw1 type=fifo impl=srl
    hls::stream<data_t> s_feat3_dw("s_feat3_dw");
    #pragma HLS stream variable=s_feat3_dw depth=2
    #pragma HLS bind_storage variable=s_feat3_dw type=fifo impl=srl
    hls::stream<data_t> s_feat3_pw2("s_feat3_pw2");
    #pragma HLS stream variable=s_feat3_pw2 depth=2
    #pragma HLS bind_storage variable=s_feat3_pw2 type=fifo impl=srl
    hls::stream<data_t> s_feat3_out("s_feat3_out");
    #pragma HLS stream variable=s_feat3_out depth=2
    #pragma HLS bind_storage variable=s_feat3_out type=fifo impl=srl
    hls::stream<data_t> s_x2_conv("s_x2_conv");
    #pragma HLS stream variable=s_x2_conv depth=2
    #pragma HLS bind_storage variable=s_x2_conv type=fifo impl=srl
    hls::stream<data_t> s_feat4_pw1("s_feat4_pw1");
    #pragma HLS stream variable=s_feat4_pw1 depth=2
    #pragma HLS bind_storage variable=s_feat4_pw1 type=fifo impl=srl
    hls::stream<data_t> s_feat4_dw("s_feat4_dw");
    #pragma HLS stream variable=s_feat4_dw depth=2
    #pragma HLS bind_storage variable=s_feat4_dw type=fifo impl=srl
    hls::stream<data_t> s_feat4_pw2("s_feat4_pw2");
    #pragma HLS stream variable=s_feat4_pw2 depth=2
    #pragma HLS bind_storage variable=s_feat4_pw2 type=fifo impl=srl
    hls::stream<data_t> s_feat4_out("s_feat4_out");
    #pragma HLS stream variable=s_feat4_out depth=2
    #pragma HLS bind_storage variable=s_feat4_out type=fifo impl=srl
    hls::stream<data_t> s_feat5_in_bypass("s_feat5_in_bypass");
    #pragma HLS stream variable=s_feat5_in_bypass depth=456
    #pragma HLS bind_storage variable=s_feat5_in_bypass type=fifo impl=lutram
    hls::stream<data_t> s_feat5_in_conv("s_feat5_in_conv");
    #pragma HLS stream variable=s_feat5_in_conv depth=2
    #pragma HLS bind_storage variable=s_feat5_in_conv type=fifo impl=srl
    hls::stream<data_t> s_feat5_pw1("s_feat5_pw1");
    #pragma HLS stream variable=s_feat5_pw1 depth=2
    #pragma HLS bind_storage variable=s_feat5_pw1 type=fifo impl=srl
    hls::stream<data_t> s_feat5_dw("s_feat5_dw");
    #pragma HLS stream variable=s_feat5_dw depth=2
    #pragma HLS bind_storage variable=s_feat5_dw type=fifo impl=srl
    hls::stream<data_t> s_feat5_pw2("s_feat5_pw2");
    #pragma HLS stream variable=s_feat5_pw2 depth=2
    #pragma HLS bind_storage variable=s_feat5_pw2 type=fifo impl=srl
    hls::stream<data_t> s_feat5_out("s_feat5_out");
    #pragma HLS stream variable=s_feat5_out depth=2
    #pragma HLS bind_storage variable=s_feat5_out type=fifo impl=srl
    hls::stream<data_t> s_feat6_in_bypass("s_feat6_in_bypass");
    #pragma HLS stream variable=s_feat6_in_bypass depth=456
    #pragma HLS bind_storage variable=s_feat6_in_bypass type=fifo impl=lutram
    hls::stream<data_t> s_feat6_in_conv("s_feat6_in_conv");
    #pragma HLS stream variable=s_feat6_in_conv depth=2
    #pragma HLS bind_storage variable=s_feat6_in_conv type=fifo impl=srl
    hls::stream<data_t> s_feat6_pw1("s_feat6_pw1");
    #pragma HLS stream variable=s_feat6_pw1 depth=2
    #pragma HLS bind_storage variable=s_feat6_pw1 type=fifo impl=srl
    hls::stream<data_t> s_feat6_dw("s_feat6_dw");
    #pragma HLS stream variable=s_feat6_dw depth=2
    #pragma HLS bind_storage variable=s_feat6_dw type=fifo impl=srl
    hls::stream<data_t> s_feat6_pw2("s_feat6_pw2");
    #pragma HLS stream variable=s_feat6_pw2 depth=2
    #pragma HLS bind_storage variable=s_feat6_pw2 type=fifo impl=srl
    hls::stream<data_t> s_feat6_out("s_feat6_out");
    #pragma HLS stream variable=s_feat6_out depth=2
    #pragma HLS bind_storage variable=s_feat6_out type=fifo impl=srl
    hls::stream<data_t> s_x3_conv("s_x3_conv");
    #pragma HLS stream variable=s_x3_conv depth=2
    #pragma HLS bind_storage variable=s_x3_conv type=fifo impl=srl
    hls::stream<data_t> s_feat7_pw1("s_feat7_pw1");
    #pragma HLS stream variable=s_feat7_pw1 depth=2
    #pragma HLS bind_storage variable=s_feat7_pw1 type=fifo impl=srl
    hls::stream<data_t> s_feat7_dw("s_feat7_dw");
    #pragma HLS stream variable=s_feat7_dw depth=2
    #pragma HLS bind_storage variable=s_feat7_dw type=fifo impl=srl
    hls::stream<data_t> s_feat7_pw2("s_feat7_pw2");
    #pragma HLS stream variable=s_feat7_pw2 depth=2
    #pragma HLS bind_storage variable=s_feat7_pw2 type=fifo impl=srl
    hls::stream<data_t> s_feat7_out("s_feat7_out");
    #pragma HLS stream variable=s_feat7_out depth=2
    #pragma HLS bind_storage variable=s_feat7_out type=fifo impl=srl
    hls::stream<data_t> s_feat8_in_bypass("s_feat8_in_bypass");
    #pragma HLS stream variable=s_feat8_in_bypass depth=228
    #pragma HLS bind_storage variable=s_feat8_in_bypass type=fifo impl=lutram
    hls::stream<data_t> s_feat8_in_conv("s_feat8_in_conv");
    #pragma HLS stream variable=s_feat8_in_conv depth=2
    #pragma HLS bind_storage variable=s_feat8_in_conv type=fifo impl=srl
    hls::stream<data_t> s_feat8_pw1("s_feat8_pw1");
    #pragma HLS stream variable=s_feat8_pw1 depth=2
    #pragma HLS bind_storage variable=s_feat8_pw1 type=fifo impl=srl
    hls::stream<data_t> s_feat8_dw("s_feat8_dw");
    #pragma HLS stream variable=s_feat8_dw depth=2
    #pragma HLS bind_storage variable=s_feat8_dw type=fifo impl=srl
    hls::stream<data_t> s_feat8_pw2("s_feat8_pw2");
    #pragma HLS stream variable=s_feat8_pw2 depth=2
    #pragma HLS bind_storage variable=s_feat8_pw2 type=fifo impl=srl
    hls::stream<data_t> s_feat8_out("s_feat8_out");
    #pragma HLS stream variable=s_feat8_out depth=2
    #pragma HLS bind_storage variable=s_feat8_out type=fifo impl=srl
    hls::stream<data_t> s_feat9_in_bypass("s_feat9_in_bypass");
    #pragma HLS stream variable=s_feat9_in_bypass depth=228
    #pragma HLS bind_storage variable=s_feat9_in_bypass type=fifo impl=lutram
    hls::stream<data_t> s_feat9_in_conv("s_feat9_in_conv");
    #pragma HLS stream variable=s_feat9_in_conv depth=2
    #pragma HLS bind_storage variable=s_feat9_in_conv type=fifo impl=srl
    hls::stream<data_t> s_feat9_pw1("s_feat9_pw1");
    #pragma HLS stream variable=s_feat9_pw1 depth=2
    #pragma HLS bind_storage variable=s_feat9_pw1 type=fifo impl=srl
    hls::stream<data_t> s_feat9_dw("s_feat9_dw");
    #pragma HLS stream variable=s_feat9_dw depth=2
    #pragma HLS bind_storage variable=s_feat9_dw type=fifo impl=srl
    hls::stream<data_t> s_feat9_pw2("s_feat9_pw2");
    #pragma HLS stream variable=s_feat9_pw2 depth=2
    #pragma HLS bind_storage variable=s_feat9_pw2 type=fifo impl=srl
    hls::stream<data_t> s_feat9_out("s_feat9_out");
    #pragma HLS stream variable=s_feat9_out depth=2
    #pragma HLS bind_storage variable=s_feat9_out type=fifo impl=srl
    hls::stream<data_t> s_feat10_in_bypass("s_feat10_in_bypass");
    #pragma HLS stream variable=s_feat10_in_bypass depth=228
    #pragma HLS bind_storage variable=s_feat10_in_bypass type=fifo impl=lutram
    hls::stream<data_t> s_feat10_in_conv("s_feat10_in_conv");
    #pragma HLS stream variable=s_feat10_in_conv depth=2
    #pragma HLS bind_storage variable=s_feat10_in_conv type=fifo impl=srl
    hls::stream<data_t> s_feat10_pw1("s_feat10_pw1");
    #pragma HLS stream variable=s_feat10_pw1 depth=2
    #pragma HLS bind_storage variable=s_feat10_pw1 type=fifo impl=srl
    hls::stream<data_t> s_feat10_dw("s_feat10_dw");
    #pragma HLS stream variable=s_feat10_dw depth=2
    #pragma HLS bind_storage variable=s_feat10_dw type=fifo impl=srl
    hls::stream<data_t> s_feat10_pw2("s_feat10_pw2");
    #pragma HLS stream variable=s_feat10_pw2 depth=2
    #pragma HLS bind_storage variable=s_feat10_pw2 type=fifo impl=srl
    hls::stream<data_t> s_feat10_out("s_feat10_out");
    #pragma HLS stream variable=s_feat10_out depth=2
    #pragma HLS bind_storage variable=s_feat10_out type=fifo impl=srl
    hls::stream<data_t> s_feat11_pw1("s_feat11_pw1");
    #pragma HLS stream variable=s_feat11_pw1 depth=2
    #pragma HLS bind_storage variable=s_feat11_pw1 type=fifo impl=srl
    hls::stream<data_t> s_feat11_dw("s_feat11_dw");
    #pragma HLS stream variable=s_feat11_dw depth=2
    #pragma HLS bind_storage variable=s_feat11_dw type=fifo impl=srl
    hls::stream<data_t> s_feat11_pw2("s_feat11_pw2");
    #pragma HLS stream variable=s_feat11_pw2 depth=2
    #pragma HLS bind_storage variable=s_feat11_pw2 type=fifo impl=srl
    hls::stream<data_t> s_feat11_out("s_feat11_out");
    #pragma HLS stream variable=s_feat11_out depth=2
    #pragma HLS bind_storage variable=s_feat11_out type=fifo impl=srl
    hls::stream<data_t> s_feat12_in_bypass("s_feat12_in_bypass");
    #pragma HLS stream variable=s_feat12_in_bypass depth=228
    #pragma HLS bind_storage variable=s_feat12_in_bypass type=fifo impl=lutram
    hls::stream<data_t> s_feat12_in_conv("s_feat12_in_conv");
    #pragma HLS stream variable=s_feat12_in_conv depth=2
    #pragma HLS bind_storage variable=s_feat12_in_conv type=fifo impl=srl
    hls::stream<data_t> s_feat12_pw1("s_feat12_pw1");
    #pragma HLS stream variable=s_feat12_pw1 depth=2
    #pragma HLS bind_storage variable=s_feat12_pw1 type=fifo impl=srl
    hls::stream<data_t> s_feat12_dw("s_feat12_dw");
    #pragma HLS stream variable=s_feat12_dw depth=2
    #pragma HLS bind_storage variable=s_feat12_dw type=fifo impl=srl
    hls::stream<data_t> s_feat12_pw2("s_feat12_pw2");
    #pragma HLS stream variable=s_feat12_pw2 depth=2
    #pragma HLS bind_storage variable=s_feat12_pw2 type=fifo impl=srl
    hls::stream<data_t> s_feat12_out("s_feat12_out");
    #pragma HLS stream variable=s_feat12_out depth=2
    #pragma HLS bind_storage variable=s_feat12_out type=fifo impl=srl
    hls::stream<data_t> s_feat13_in_bypass("s_feat13_in_bypass");
    #pragma HLS stream variable=s_feat13_in_bypass depth=228
    #pragma HLS bind_storage variable=s_feat13_in_bypass type=fifo impl=lutram
    hls::stream<data_t> s_feat13_in_conv("s_feat13_in_conv");
    #pragma HLS stream variable=s_feat13_in_conv depth=2
    #pragma HLS bind_storage variable=s_feat13_in_conv type=fifo impl=srl
    hls::stream<data_t> s_feat13_pw1("s_feat13_pw1");
    #pragma HLS stream variable=s_feat13_pw1 depth=2
    #pragma HLS bind_storage variable=s_feat13_pw1 type=fifo impl=srl
    hls::stream<data_t> s_feat13_dw("s_feat13_dw");
    #pragma HLS stream variable=s_feat13_dw depth=2
    #pragma HLS bind_storage variable=s_feat13_dw type=fifo impl=srl
    hls::stream<data_t> s_feat13_pw2("s_feat13_pw2");
    #pragma HLS stream variable=s_feat13_pw2 depth=2
    #pragma HLS bind_storage variable=s_feat13_pw2 type=fifo impl=srl
    hls::stream<data_t> s_feat13_out("s_feat13_out");
    #pragma HLS stream variable=s_feat13_out depth=2
    #pragma HLS bind_storage variable=s_feat13_out type=fifo impl=srl
    hls::stream<data_t> s_x4_conv("s_x4_conv");
    #pragma HLS stream variable=s_x4_conv depth=2
    #pragma HLS bind_storage variable=s_x4_conv type=fifo impl=srl
    hls::stream<data_t> s_feat14_pw1("s_feat14_pw1");
    #pragma HLS stream variable=s_feat14_pw1 depth=2
    #pragma HLS bind_storage variable=s_feat14_pw1 type=fifo impl=srl
    hls::stream<data_t> s_feat14_dw("s_feat14_dw");
    #pragma HLS stream variable=s_feat14_dw depth=2
    #pragma HLS bind_storage variable=s_feat14_dw type=fifo impl=srl
    hls::stream<data_t> s_feat14_pw2("s_feat14_pw2");
    #pragma HLS stream variable=s_feat14_pw2 depth=2
    #pragma HLS bind_storage variable=s_feat14_pw2 type=fifo impl=srl
    hls::stream<data_t> s_feat14_out("s_feat14_out");
    #pragma HLS stream variable=s_feat14_out depth=2
    #pragma HLS bind_storage variable=s_feat14_out type=fifo impl=srl
    hls::stream<data_t> s_feat15_in_bypass("s_feat15_in_bypass");
    #pragma HLS stream variable=s_feat15_in_bypass depth=116
    #pragma HLS bind_storage variable=s_feat15_in_bypass type=fifo impl=lutram
    hls::stream<data_t> s_feat15_in_conv("s_feat15_in_conv");
    #pragma HLS stream variable=s_feat15_in_conv depth=2
    #pragma HLS bind_storage variable=s_feat15_in_conv type=fifo impl=srl
    hls::stream<data_t> s_feat15_pw1("s_feat15_pw1");
    #pragma HLS stream variable=s_feat15_pw1 depth=2
    #pragma HLS bind_storage variable=s_feat15_pw1 type=fifo impl=srl
    hls::stream<data_t> s_feat15_dw("s_feat15_dw");
    #pragma HLS stream variable=s_feat15_dw depth=2
    #pragma HLS bind_storage variable=s_feat15_dw type=fifo impl=srl
    hls::stream<data_t> s_feat15_pw2("s_feat15_pw2");
    #pragma HLS stream variable=s_feat15_pw2 depth=2
    #pragma HLS bind_storage variable=s_feat15_pw2 type=fifo impl=srl
    hls::stream<data_t> s_feat15_out("s_feat15_out");
    #pragma HLS stream variable=s_feat15_out depth=2
    #pragma HLS bind_storage variable=s_feat15_out type=fifo impl=srl
    hls::stream<data_t> s_feat16_in_bypass("s_feat16_in_bypass");
    #pragma HLS stream variable=s_feat16_in_bypass depth=116
    #pragma HLS bind_storage variable=s_feat16_in_bypass type=fifo impl=lutram
    hls::stream<data_t> s_feat16_in_conv("s_feat16_in_conv");
    #pragma HLS stream variable=s_feat16_in_conv depth=2
    #pragma HLS bind_storage variable=s_feat16_in_conv type=fifo impl=srl
    hls::stream<data_t> s_feat16_pw1("s_feat16_pw1");
    #pragma HLS stream variable=s_feat16_pw1 depth=2
    #pragma HLS bind_storage variable=s_feat16_pw1 type=fifo impl=srl
    hls::stream<data_t> s_feat16_dw("s_feat16_dw");
    #pragma HLS stream variable=s_feat16_dw depth=2
    #pragma HLS bind_storage variable=s_feat16_dw type=fifo impl=srl
    hls::stream<data_t> s_feat16_pw2("s_feat16_pw2");
    #pragma HLS stream variable=s_feat16_pw2 depth=2
    #pragma HLS bind_storage variable=s_feat16_pw2 type=fifo impl=srl
    hls::stream<data_t> s_feat16_out("s_feat16_out");
    #pragma HLS stream variable=s_feat16_out depth=2
    #pragma HLS bind_storage variable=s_feat16_out type=fifo impl=srl
    hls::stream<data_t> s_feat17_pw1("s_feat17_pw1");
    #pragma HLS stream variable=s_feat17_pw1 depth=2
    #pragma HLS bind_storage variable=s_feat17_pw1 type=fifo impl=srl
    hls::stream<data_t> s_feat17_dw("s_feat17_dw");
    #pragma HLS stream variable=s_feat17_dw depth=2
    #pragma HLS bind_storage variable=s_feat17_dw type=fifo impl=srl
    hls::stream<data_t> s_feat17_pw2("s_feat17_pw2");
    #pragma HLS stream variable=s_feat17_pw2 depth=2
    #pragma HLS bind_storage variable=s_feat17_pw2 type=fifo impl=srl
    hls::stream<data_t> s_feat17_out("s_feat17_out");
    #pragma HLS stream variable=s_feat17_out depth=2
    #pragma HLS bind_storage variable=s_feat17_out type=fifo impl=srl
    hls::stream<data_t> s_feat18("s_feat18");
    #pragma HLS stream variable=s_feat18 depth=2
    #pragma HLS bind_storage variable=s_feat18 type=fifo impl=srl
    hls::stream<data_t> s_dconv1("s_dconv1");
    #pragma HLS stream variable=s_dconv1 depth=2
    #pragma HLS bind_storage variable=s_dconv1 type=fifo impl=srl
    hls::stream<data_t> s_dconv1_resized("s_dconv1_resized");
    #pragma HLS stream variable=s_dconv1_resized depth=2
    #pragma HLS bind_storage variable=s_dconv1_resized type=fifo impl=srl
    hls::stream<data_t> s_invres1_concat("s_invres1_concat");
    #pragma HLS stream variable=s_invres1_concat depth=2
    #pragma HLS bind_storage variable=s_invres1_concat type=fifo impl=srl
    hls::stream<data_t> s_invres1_pw1("s_invres1_pw1");
    #pragma HLS stream variable=s_invres1_pw1 depth=2
    #pragma HLS bind_storage variable=s_invres1_pw1 type=fifo impl=srl
    hls::stream<data_t> s_invres1_dw("s_invres1_dw");
    #pragma HLS stream variable=s_invres1_dw depth=2
    #pragma HLS bind_storage variable=s_invres1_dw type=fifo impl=srl
    hls::stream<data_t> s_invres1_pw2("s_invres1_pw2");
    #pragma HLS stream variable=s_invres1_pw2 depth=2
    #pragma HLS bind_storage variable=s_invres1_pw2 type=fifo impl=srl
    hls::stream<data_t> s_dconv2("s_dconv2");
    #pragma HLS stream variable=s_dconv2 depth=2
    #pragma HLS bind_storage variable=s_dconv2 type=fifo impl=srl
    hls::stream<data_t> s_invres2_concat("s_invres2_concat");
    #pragma HLS stream variable=s_invres2_concat depth=2
    #pragma HLS bind_storage variable=s_invres2_concat type=fifo impl=srl
    hls::stream<data_t> s_invres2_pw1("s_invres2_pw1");
    #pragma HLS stream variable=s_invres2_pw1 depth=2
    #pragma HLS bind_storage variable=s_invres2_pw1 type=fifo impl=srl
    hls::stream<data_t> s_invres2_dw("s_invres2_dw");
    #pragma HLS stream variable=s_invres2_dw depth=2
    #pragma HLS bind_storage variable=s_invres2_dw type=fifo impl=srl
    hls::stream<data_t> s_invres2_pw2("s_invres2_pw2");
    #pragma HLS stream variable=s_invres2_pw2 depth=2
    #pragma HLS bind_storage variable=s_invres2_pw2 type=fifo impl=srl
    hls::stream<data_t> s_dconv3("s_dconv3");
    #pragma HLS stream variable=s_dconv3 depth=2
    #pragma HLS bind_storage variable=s_dconv3 type=fifo impl=srl
    hls::stream<data_t> s_invres3_concat("s_invres3_concat");
    #pragma HLS stream variable=s_invres3_concat depth=2
    #pragma HLS bind_storage variable=s_invres3_concat type=fifo impl=srl
    hls::stream<data_t> s_invres3_pw1("s_invres3_pw1");
    #pragma HLS stream variable=s_invres3_pw1 depth=2
    #pragma HLS bind_storage variable=s_invres3_pw1 type=fifo impl=srl
    hls::stream<data_t> s_invres3_dw("s_invres3_dw");
    #pragma HLS stream variable=s_invres3_dw depth=2
    #pragma HLS bind_storage variable=s_invres3_dw type=fifo impl=srl
    hls::stream<data_t> s_invres3_pw2("s_invres3_pw2");
    #pragma HLS stream variable=s_invres3_pw2 depth=2
    #pragma HLS bind_storage variable=s_invres3_pw2 type=fifo impl=srl
    hls::stream<data_t> s_dconv4("s_dconv4");
    #pragma HLS stream variable=s_dconv4 depth=2
    #pragma HLS bind_storage variable=s_dconv4 type=fifo impl=srl
    hls::stream<data_t> s_invres4_concat("s_invres4_concat");
    #pragma HLS stream variable=s_invres4_concat depth=2
    #pragma HLS bind_storage variable=s_invres4_concat type=fifo impl=srl
    hls::stream<data_t> s_invres4_pw1("s_invres4_pw1");
    #pragma HLS stream variable=s_invres4_pw1 depth=2
    #pragma HLS bind_storage variable=s_invres4_pw1 type=fifo impl=srl
    hls::stream<data_t> s_invres4_dw("s_invres4_dw");
    #pragma HLS stream variable=s_invres4_dw depth=2
    #pragma HLS bind_storage variable=s_invres4_dw type=fifo impl=srl
    hls::stream<data_t> s_invres4_pw2("s_invres4_pw2");
    #pragma HLS stream variable=s_invres4_pw2 depth=2
    #pragma HLS bind_storage variable=s_invres4_pw2 type=fifo impl=srl
    hls::stream<data_t> s_conv_last("s_conv_last");
    #pragma HLS stream variable=s_conv_last depth=2
    #pragma HLS bind_storage variable=s_conv_last type=fifo impl=srl
    hls::stream<data_t> s_conv_score("s_conv_score");
    #pragma HLS stream variable=s_conv_score depth=2
    #pragma HLS bind_storage variable=s_conv_score type=fifo impl=srl



    // -------------------------------------------------------------
    // ENCODER BACKBONE
    // -------------------------------------------------------------
    // Stage 0: Conv 3x3, stride=1, padding=1
    stream_conv2d<INPUT_H, INPUT_W, INPUT_C, 16, 3, 1, 1>(image_in, s_conv0, w_feat0, b_feat0);

    // Stage 1: Inverted Residual (16->8, stride=1, expand=1)
    stream_depthwise_conv2d<88, 456, 16, 3, 1, 1>(s_conv0, s_feat1_dw, w_feat1_dw, b_feat1_dw);
    stream_pointwise_conv2d<88, 456, 16, 8, false>(s_feat1_dw, s_feat1_pw, w_feat1_pw, b_feat1_pw);
    // x1 Skip connection
    duplicate_skip<88 * 456 * 8, 131072>(s_feat1_pw, s_x1_skip_a, s_x1_skip_b, s_x1_conv);

    // Stage 2: Inverted Residual (8->12, stride=2, expand=6)
    stream_pointwise_conv2d<88, 456, 8, 48>(s_x1_conv, s_feat2_pw1, w_feat2_pw1, b_feat2_pw1);
    stream_depthwise_conv2d<88, 456, 48, 3, 2, 1>(s_feat2_pw1, s_feat2_dw, w_feat2_dw, b_feat2_dw);
    stream_pointwise_conv2d<44, 228, 48, 12, false>(s_feat2_dw, s_feat2_pw2, w_feat2_pw2, b_feat2_pw2);

    // Stage 3: Inverted Residual (12->12, stride=1, expand=6)
    duplicate_stream<44 * 228 * 12>(s_feat2_pw2, s_feat3_in_bypass, s_feat3_in_conv);
    stream_pointwise_conv2d<44, 228, 12, 72>(s_feat3_in_conv, s_feat3_pw1, w_feat3_pw1, b_feat3_pw1);
    stream_depthwise_conv2d<44, 228, 72, 3, 1, 1>(s_feat3_pw1, s_feat3_dw, w_feat3_dw, b_feat3_dw);
    stream_pointwise_conv2d<44, 228, 72, 12, false>(s_feat3_dw, s_feat3_pw2, w_feat3_pw2, b_feat3_pw2);
    stream_add<44, 228, 12>(s_feat3_in_bypass, s_feat3_pw2, s_feat3_out);
    // x2 Skip connection
    duplicate_skip<44 * 228 * 12, 32768>(s_feat3_out, s_x2_skip_a, s_x2_skip_b, s_x2_conv);

    // Stage 4: Inverted Residual (12->16, stride=2, expand=6)
    stream_pointwise_conv2d<44, 228, 12, 72>(s_x2_conv, s_feat4_pw1, w_feat4_pw1, b_feat4_pw1);
    stream_depthwise_conv2d<44, 228, 72, 3, 2, 1>(s_feat4_pw1, s_feat4_dw, w_feat4_dw, b_feat4_dw);
    stream_pointwise_conv2d<22, 114, 72, 16, false>(s_feat4_dw, s_feat4_pw2, w_feat4_pw2, b_feat4_pw2);
    // No residual connect
    // Simply rename output stream
    #define s_feat4_out s_feat4_pw2

    // Stage 5: Inverted Residual (16->16, stride=1, expand=6)
    duplicate_stream<22 * 114 * 16>(s_feat4_out, s_feat5_in_bypass, s_feat5_in_conv);
    stream_pointwise_conv2d<22, 114, 16, 96>(s_feat5_in_conv, s_feat5_pw1, w_feat5_pw1, b_feat5_pw1);
    stream_depthwise_conv2d<22, 114, 96, 3, 1, 1>(s_feat5_pw1, s_feat5_dw, w_feat5_dw, b_feat5_dw);
    stream_pointwise_conv2d<22, 114, 96, 16, false>(s_feat5_dw, s_feat5_pw2, w_feat5_pw2, b_feat5_pw2);
    stream_add<22, 114, 16>(s_feat5_in_bypass, s_feat5_pw2, s_feat5_out);

    // Stage 6: Inverted Residual (16->16, stride=1, expand=6)
    duplicate_stream<22 * 114 * 16>(s_feat5_out, s_feat6_in_bypass, s_feat6_in_conv);
    stream_pointwise_conv2d<22, 114, 16, 96>(s_feat6_in_conv, s_feat6_pw1, w_feat6_pw1, b_feat6_pw1);
    stream_depthwise_conv2d<22, 114, 96, 3, 1, 1>(s_feat6_pw1, s_feat6_dw, w_feat6_dw, b_feat6_dw);
    stream_pointwise_conv2d<22, 114, 96, 16, false>(s_feat6_dw, s_feat6_pw2, w_feat6_pw2, b_feat6_pw2);
    stream_add<22, 114, 16>(s_feat6_in_bypass, s_feat6_pw2, s_feat6_out);
    // x3 Skip connection
    duplicate_skip<22 * 114 * 16, 16384>(s_feat6_out, s_x3_skip_a, s_x3_skip_b, s_x3_conv);

    // Stage 7: Inverted Residual (16->32, stride=2, expand=6)
    stream_pointwise_conv2d<22, 114, 16, 96>(s_x3_conv, s_feat7_pw1, w_feat7_pw1, b_feat7_pw1);
    stream_depthwise_conv2d<22, 114, 96, 3, 2, 1>(s_feat7_pw1, s_feat7_dw, w_feat7_dw, b_feat7_dw);
    stream_pointwise_conv2d<11, 57, 96, 32, false>(s_feat7_dw, s_feat7_pw2, w_feat7_pw2, b_feat7_pw2);
    // No residual connect
    // Simply rename output stream
    #define s_feat7_out s_feat7_pw2

    // Stage 8: Inverted Residual (32->32, stride=1, expand=6)
    duplicate_stream<11 * 57 * 32>(s_feat7_out, s_feat8_in_bypass, s_feat8_in_conv);
    stream_pointwise_conv2d<11, 57, 32, 192>(s_feat8_in_conv, s_feat8_pw1, w_feat8_pw1, b_feat8_pw1);
    stream_depthwise_conv2d<11, 57, 192, 3, 1, 1>(s_feat8_pw1, s_feat8_dw, w_feat8_dw, b_feat8_dw);
    stream_pointwise_conv2d<11, 57, 192, 32, false>(s_feat8_dw, s_feat8_pw2, w_feat8_pw2, b_feat8_pw2);
    stream_add<11, 57, 32>(s_feat8_in_bypass, s_feat8_pw2, s_feat8_out);

    // Stage 9: Inverted Residual (32->32, stride=1, expand=6)
    duplicate_stream<11 * 57 * 32>(s_feat8_out, s_feat9_in_bypass, s_feat9_in_conv);
    stream_pointwise_conv2d<11, 57, 32, 192>(s_feat9_in_conv, s_feat9_pw1, w_feat9_pw1, b_feat9_pw1);
    stream_depthwise_conv2d<11, 57, 192, 3, 1, 1>(s_feat9_pw1, s_feat9_dw, w_feat9_dw, b_feat9_dw);
    stream_pointwise_conv2d<11, 57, 192, 32, false>(s_feat9_dw, s_feat9_pw2, w_feat9_pw2, b_feat9_pw2);
    stream_add<11, 57, 32>(s_feat9_in_bypass, s_feat9_pw2, s_feat9_out);

    // Stage 10: Inverted Residual (32->32, stride=1, expand=6)
    duplicate_stream<11 * 57 * 32>(s_feat9_out, s_feat10_in_bypass, s_feat10_in_conv);
    stream_pointwise_conv2d<11, 57, 32, 192>(s_feat10_in_conv, s_feat10_pw1, w_feat10_pw1, b_feat10_pw1);
    stream_depthwise_conv2d<11, 57, 192, 3, 1, 1>(s_feat10_pw1, s_feat10_dw, w_feat10_dw, b_feat10_dw);
    stream_pointwise_conv2d<11, 57, 192, 32, false>(s_feat10_dw, s_feat10_pw2, w_feat10_pw2, b_feat10_pw2);
    stream_add<11, 57, 32>(s_feat10_in_bypass, s_feat10_pw2, s_feat10_out);

    // Stage 11: Inverted Residual (32->48, stride=1, expand=6)
    stream_pointwise_conv2d<11, 57, 32, 192>(s_feat10_out, s_feat11_pw1, w_feat11_pw1, b_feat11_pw1);
    stream_depthwise_conv2d<11, 57, 192, 3, 1, 1>(s_feat11_pw1, s_feat11_dw, w_feat11_dw, b_feat11_dw);
    stream_pointwise_conv2d<11, 57, 192, 48, false>(s_feat11_dw, s_feat11_pw2, w_feat11_pw2, b_feat11_pw2);
    // No residual connect
    // Simply rename output stream
    #define s_feat11_out s_feat11_pw2

    // Stage 12: Inverted Residual (48->48, stride=1, expand=6)
    duplicate_stream<11 * 57 * 48>(s_feat11_out, s_feat12_in_bypass, s_feat12_in_conv);
    stream_pointwise_conv2d<11, 57, 48, 288>(s_feat12_in_conv, s_feat12_pw1, w_feat12_pw1, b_feat12_pw1);
    stream_depthwise_conv2d<11, 57, 288, 3, 1, 1>(s_feat12_pw1, s_feat12_dw, w_feat12_dw, b_feat12_dw);
    stream_pointwise_conv2d<11, 57, 288, 48, false>(s_feat12_dw, s_feat12_pw2, w_feat12_pw2, b_feat12_pw2);
    stream_add<11, 57, 48>(s_feat12_in_bypass, s_feat12_pw2, s_feat12_out);

    // Stage 13: Inverted Residual (48->48, stride=1, expand=6)
    duplicate_stream<11 * 57 * 48>(s_feat12_out, s_feat13_in_bypass, s_feat13_in_conv);
    stream_pointwise_conv2d<11, 57, 48, 288>(s_feat13_in_conv, s_feat13_pw1, w_feat13_pw1, b_feat13_pw1);
    stream_depthwise_conv2d<11, 57, 288, 3, 1, 1>(s_feat13_pw1, s_feat13_dw, w_feat13_dw, b_feat13_dw);
    stream_pointwise_conv2d<11, 57, 288, 48, false>(s_feat13_dw, s_feat13_pw2, w_feat13_pw2, b_feat13_pw2);
    stream_add<11, 57, 48>(s_feat13_in_bypass, s_feat13_pw2, s_feat13_out);
    // x4 Skip connection
    duplicate_skip<11 * 57 * 48, 8192>(s_feat13_out, s_x4_skip_a, s_x4_skip_b, s_x4_conv);

    // Stage 14: Inverted Residual (48->80, stride=2, expand=6)
    stream_pointwise_conv2d<11, 57, 48, 288>(s_x4_conv, s_feat14_pw1, w_feat14_pw1, b_feat14_pw1);
    stream_depthwise_conv2d<11, 57, 288, 3, 2, 1>(s_feat14_pw1, s_feat14_dw, w_feat14_dw, b_feat14_dw);
    stream_pointwise_conv2d<6, 29, 288, 80, false>(s_feat14_dw, s_feat14_pw2, w_feat14_pw2, b_feat14_pw2);
    // No residual connect
    // Simply rename output stream
    #define s_feat14_out s_feat14_pw2

    // Stage 15: Inverted Residual (80->80, stride=1, expand=6)
    duplicate_stream<6 * 29 * 80>(s_feat14_out, s_feat15_in_bypass, s_feat15_in_conv);
    stream_pointwise_conv2d<6, 29, 80, 480>(s_feat15_in_conv, s_feat15_pw1, w_feat15_pw1, b_feat15_pw1);
    stream_depthwise_conv2d<6, 29, 480, 3, 1, 1>(s_feat15_pw1, s_feat15_dw, w_feat15_dw, b_feat15_dw);
    stream_pointwise_conv2d<6, 29, 480, 80, false>(s_feat15_dw, s_feat15_pw2, w_feat15_pw2, b_feat15_pw2);
    stream_add<6, 29, 80>(s_feat15_in_bypass, s_feat15_pw2, s_feat15_out);

    // Stage 16: Inverted Residual (80->80, stride=1, expand=6)
    duplicate_stream<6 * 29 * 80>(s_feat15_out, s_feat16_in_bypass, s_feat16_in_conv);
    stream_pointwise_conv2d<6, 29, 80, 480>(s_feat16_in_conv, s_feat16_pw1, w_feat16_pw1, b_feat16_pw1);
    stream_depthwise_conv2d<6, 29, 480, 3, 1, 1>(s_feat16_pw1, s_feat16_dw, w_feat16_dw, b_feat16_dw);
    stream_pointwise_conv2d<6, 29, 480, 80, false>(s_feat16_dw, s_feat16_pw2, w_feat16_pw2, b_feat16_pw2);
    stream_add<6, 29, 80>(s_feat16_in_bypass, s_feat16_pw2, s_feat16_out);

    // Stage 17: Inverted Residual (80->160, stride=1, expand=6)
    stream_pointwise_conv2d<6, 29, 80, 480>(s_feat16_out, s_feat17_pw1, w_feat17_pw1, b_feat17_pw1);
    stream_depthwise_conv2d<6, 29, 480, 3, 1, 1>(s_feat17_pw1, s_feat17_dw, w_feat17_dw, b_feat17_dw);
    stream_pointwise_conv2d<6, 29, 480, 160, false>(s_feat17_dw, s_feat17_pw2, w_feat17_pw2, b_feat17_pw2);
    // No residual connect
    // Simply rename output stream
    #define s_feat17_out s_feat17_pw2

    // Stage 18: Conv 1x1 PW (160->640)
    stream_pointwise_conv2d<6, 29, 160, C5>(s_feat17_out, s_feat18, w_feat18, b_feat18);

    // -------------------------------------------------------------
    // DECODER
    // -------------------------------------------------------------
    // Decoder 1: TransposeConv (6x29x640 -> 12x58x48) + BilinearResize -> 11x57 + Concat + InvertedResidual
    stream_conv_transpose2d<6, 29, C5, C4, 4, 2, 1>(s_feat18, s_dconv1, w_dconv1, b_dconv1);
    stream_bilinear_resize<12, 58, 11, 57, C4>(s_dconv1, s_dconv1_resized);
    concat_skip<11, 57, C4, 8192>(s_x4_skip_a, s_x4_skip_b, s_dconv1_resized, s_invres1_concat);
    stream_pointwise_conv2d<11, 57, 96, 576>(s_invres1_concat, s_invres1_pw1, w_invres1_pw1, b_invres1_pw1);
    stream_depthwise_conv2d<11, 57, 576, 3, 1, 1>(s_invres1_pw1, s_invres1_dw, w_invres1_dw, b_invres1_dw);
    stream_pointwise_conv2d<11, 57, 576, C4, false>(s_invres1_dw, s_invres1_pw2, w_invres1_pw2, b_invres1_pw2);

    // Decoder 2: TransposeConv (11x57x48 -> 22x114x16) + Concat + InvertedResidual
    stream_conv_transpose2d<11, 57, C4, C3, 4, 2, 1>(s_invres1_pw2, s_dconv2, w_dconv2, b_dconv2);
    concat_skip<22, 114, C3, 16384>(s_x3_skip_a, s_x3_skip_b, s_dconv2, s_invres2_concat);
    stream_pointwise_conv2d<22, 114, 32, 192>(s_invres2_concat, s_invres2_pw1, w_invres2_pw1, b_invres2_pw1);
    stream_depthwise_conv2d<22, 114, 192, 3, 1, 1>(s_invres2_pw1, s_invres2_dw, w_invres2_dw, b_invres2_dw);
    stream_pointwise_conv2d<22, 114, 192, C3, false>(s_invres2_dw, s_invres2_pw2, w_invres2_pw2, b_invres2_pw2);

    // Decoder 3: TransposeConv (22x114x16 -> 44x228x12) + Concat + InvertedResidual
    stream_conv_transpose2d<22, 114, C3, C2, 4, 2, 1>(s_invres2_pw2, s_dconv3, w_dconv3, b_dconv3);
    concat_skip<44, 228, C2, 32768>(s_x2_skip_a, s_x2_skip_b, s_dconv3, s_invres3_concat);
    stream_pointwise_conv2d<44, 228, 24, 144>(s_invres3_concat, s_invres3_pw1, w_invres3_pw1, b_invres3_pw1);
    stream_depthwise_conv2d<44, 228, 144, 3, 1, 1>(s_invres3_pw1, s_invres3_dw, w_invres3_dw, b_invres3_dw);
    stream_pointwise_conv2d<44, 228, 144, C2, false>(s_invres3_dw, s_invres3_pw2, w_invres3_pw2, b_invres3_pw2);

    // Decoder 4: TransposeConv (44x228x12 -> 88x456x8) + Concat + InvertedResidual
    stream_conv_transpose2d<44, 228, C2, C1, 4, 2, 1>(s_invres3_pw2, s_dconv4, w_dconv4, b_dconv4);
    concat_skip<88, 456, C1, 131072>(s_x1_skip_a, s_x1_skip_b, s_dconv4, s_invres4_concat);
    stream_pointwise_conv2d<88, 456, 16, 96>(s_invres4_concat, s_invres4_pw1, w_invres4_pw1, b_invres4_pw1);
    stream_depthwise_conv2d<88, 456, 96, 3, 1, 1>(s_invres4_pw1, s_invres4_dw, w_invres4_dw, b_invres4_dw);
    stream_pointwise_conv2d<88, 456, 96, C1, false>(s_invres4_dw, s_invres4_pw2, w_invres4_pw2, b_invres4_pw2);

    // Final Conv layers
    stream_pointwise_conv2d<88, 456, C1, 3, false, weight_t>(s_invres4_pw2, s_conv_last, w_conv_last, b_conv_last);
    stream_pointwise_conv2d<88, 456, 3, 1, false, weight_t>(s_conv_last, s_conv_score, w_conv_score, b_conv_score);

    // Post-processing
    streaming_ccl(s_conv_score, centers_out, ion_count);
}
