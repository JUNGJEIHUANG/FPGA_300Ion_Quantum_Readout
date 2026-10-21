#pragma once
#include <ap_int.h>
#include <ap_fixed.h>
#include <hls_stream.h>
typedef ap_fixed<18,10,AP_RND,AP_SAT> data_t;
struct IonCenter { float x; float y; int area; };
void vitseg_core(hls::stream<data_t>& image_in,hls::stream<IonCenter>& centers_out,int& ion_count);
#ifndef __SYNTHESIS__
extern int vit_csim_logits[88*456];
int vit_ccl_selftest();
#endif
