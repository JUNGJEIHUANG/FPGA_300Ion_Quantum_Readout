#ifndef STREAMING_CCL_H
#define STREAMING_CCL_H

#include "dwnet_types.h"
#include "dwnet_config.h"

// Struct representing a detected ion center
struct IonCenter {
    float x;
    float y;
    int area;
};

// Stream interfaces
void streaming_ccl(
    hls::stream<data_t>& logit_stream,
    hls::stream<IonCenter>& center_stream,
    int& ion_count
);

#endif // STREAMING_CCL_H
