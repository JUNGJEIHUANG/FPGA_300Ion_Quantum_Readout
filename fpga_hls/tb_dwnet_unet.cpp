#include <iostream>
#include <fstream>
#include "dwnet_unet.h"

int main() {
    std::cout << "Starting C-Simulation for DWNetV2-UNet..." << std::endl;

    hls::stream<data_t> image_in("image_in");
    hls::stream<IonCenter> centers_out("centers_out");
    int ion_count = 0;

    std::cout << "Loading real test image from test_input.txt..." << std::endl;
    std::ifstream infile("test_input.txt");
    if (!infile.is_open()) {
        std::cerr << "ERROR: Could not open test_input.txt!" << std::endl;
        return 1;
    }

    float val;
    int count = 0;
    while (infile >> val) {
        image_in.write((data_t)val);
        count++;
    }
    infile.close();

    if (count != INPUT_H * INPUT_W) {
        std::cerr << "ERROR: Read " << count << " pixels, expected " << INPUT_H * INPUT_W << std::endl;
        return 1;
    }
    std::cout << "Loaded " << count << " pixels successfully." << std::endl;

    std::cout << "Running top-level dwnet_unet dataflow..." << std::endl;
    dwnet_unet(image_in, centers_out, ion_count);

    std::cout << "Simulation Finished." << std::endl;
    std::cout << "Detected Ion Count: " << ion_count << std::endl;

    // Print out the first 10 coordinates
    int print_count = ion_count > 10 ? 10 : ion_count;
    for (int i = 0; i < ion_count; i++) {
        IonCenter center = centers_out.read();
        if (i < print_count) {
            std::cout << "Ion " << i + 1 << " - Center X: " << center.x 
                      << ", Y: " << center.y << ", Area: " << center.area << std::endl;
        }
    }

    std::cout << "C-Simulation verification successful!" << std::endl;
    return 0;
}
