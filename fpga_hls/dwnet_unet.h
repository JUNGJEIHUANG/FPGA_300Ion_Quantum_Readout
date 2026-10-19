#ifndef DWNET_UNET_H
#define DWNET_UNET_H

#include "dwnet_types.h"
#include "dwnet_config.h"
#include "streaming_ccl.h"

// Top-level network interface
void dwnet_unet(
    hls::stream<data_t>& image_in,
    hls::stream<IonCenter>& centers_out,
    int& ion_count
);

#endif // DWNET_UNET_H
