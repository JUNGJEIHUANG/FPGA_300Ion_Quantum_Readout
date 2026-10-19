#include "streaming_ccl.h"

#define MAX_LABELS 400

void streaming_ccl(
    hls::stream<data_t>& logit_stream,
    hls::stream<IonCenter>& center_stream,
    int& ion_count
) {
    #pragma HLS inline off

    // Label buffer for the previous row
    unsigned short label_buf[INPUT_W];
    #pragma HLS bind_storage variable=label_buf type=ram_2p impl=lutram

    // Equivalence table for label merging
    unsigned short parent[MAX_LABELS];
    #pragma HLS bind_storage variable=parent type=ram_2p impl=lutram

    // Centroid accumulators (32-bit unsigned int is sufficient since max sum_x < 40128 * 456 = 1.83e7)
    unsigned int sum_x[MAX_LABELS];
    unsigned int sum_y[MAX_LABELS];
    unsigned int area[MAX_LABELS];
    #pragma HLS bind_storage variable=sum_x type=ram_2p impl=lutram
    #pragma HLS bind_storage variable=sum_y type=ram_2p impl=lutram
    #pragma HLS bind_storage variable=area type=ram_2p impl=lutram


    // Initialize equivalence table and accumulators
    L_INIT: for (int i = 0; i < MAX_LABELS; i++) {
        #pragma HLS PIPELINE II=1
        parent[i] = i;
        sum_x[i] = 0;
        sum_y[i] = 0;
        area[i] = 0;
    }

    unsigned short next_label = 1;
    unsigned short west_label = 0;

    // Single-pass streaming raster scan
    L_ROW: for (int r = 0; r < INPUT_H; r++) {
        L_COL: for (int c = 0; c < INPUT_W; c++) {
            #pragma HLS PIPELINE II=1
            #pragma HLS dependence variable=label_buf inter false
            #pragma HLS dependence variable=sum_x inter false
            #pragma HLS dependence variable=sum_y inter false
            #pragma HLS dependence variable=area inter false

            data_t logit = logit_stream.read();
            // Logit > 0 is mathematically equivalent to sigmoid(logit) > 0.5
            bool is_fg = (logit.V > 0);

            unsigned short current_label = 0;

            if (is_fg) {
                unsigned short north_label = (r > 0) ? label_buf[c] : 0;

                // Determine label based on West and North neighbors
                if (west_label == 0 && north_label == 0) {
                    if (next_label < MAX_LABELS) {
                        current_label = next_label;
                        next_label++;
                    } else {
                        current_label = MAX_LABELS - 1; // Overflow protection
                    }
                } else if (west_label != 0 && north_label == 0) {
                    current_label = west_label;
                } else if (west_label == 0 && north_label != 0) {
                    current_label = north_label;
                } else {
                    // Both West and North have labels: copy West and resolve equivalence
                    current_label = west_label;
                    if (west_label != north_label) {
                        // Bounded root finding for hardware pipeline (II=1)
                        unsigned short root_w = west_label;
                        for (int s = 0; s < 4; s++) {
                            #pragma HLS UNROLL
                            root_w = parent[root_w];
                        }
                        unsigned short root_n = north_label;
                        for (int s = 0; s < 4; s++) {
                            #pragma HLS UNROLL
                            root_n = parent[root_n];
                        }
                        // Merge roots
                        if (root_w != root_n) {
                            parent[root_n] = root_w;
                        }
                    }
                }

                // Accumulate spatial coordinates and area
                sum_x[current_label] += c;
                sum_y[current_label] += r;
                area[current_label] += 1;
            }

            // Save label for next pixel/row
            label_buf[c] = current_label;
            west_label = current_label;
        }
        west_label = 0; // Reset West label at the start of a new row
    }

    // Resolve equivalences and compile final centroids
    unsigned short resolved_count = 0;

    // Resolve roots in the parent table (bounded path traversal)
    L_RESOLVE: for (int i = 1; i < next_label; i++) {
        #pragma HLS loop_tripcount min=1 max=400
        unsigned short root = i;
        for (int s = 0; s < 8; s++) {
            #pragma HLS PIPELINE II=1
            if (parent[root] != root) {
                root = parent[root];
            }
        }
        if (root != i) {
            sum_x[root] += sum_x[i];
            sum_y[root] += sum_y[i];
            area[root] += area[i];
            // Clear the child's accumulator
            area[i] = 0;
        }
    }

    // Stream out valid labels (area > 0)
    L_OUT: for (int i = 1; i < next_label; i++) {
        #pragma HLS PIPELINE II=1
        #pragma HLS loop_tripcount min=1 max=400
        if (area[i] > 0) {
            IonCenter center;
            // Float division in hardware
            center.x = (float)sum_x[i] / (float)area[i];
            center.y = (float)sum_y[i] / (float)area[i];
            center.area = area[i];
            center_stream.write(center);
            resolved_count++;
        }
    }

    ion_count = resolved_count;
}
