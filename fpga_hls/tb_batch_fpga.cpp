#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "dwnet_unet.h"

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: test_batch_fpga.exe <input_list.txt> <output_centers.txt>" << std::endl;
        return 1;
    }
    std::ifstream list_file(argv[1]);
    std::ofstream out_file(argv[2]);
    if (!list_file.is_open() || !out_file.is_open()) {
        std::cerr << "ERROR opening list or output file!" << std::endl;
        return 1;
    }

    std::string frame_txt_path;
    int frame_idx = 0;
    while (std::getline(list_file, frame_txt_path)) {
        if (frame_txt_path.empty()) continue;
        std::ifstream infile(frame_txt_path);
        if (!infile.is_open()) {
            std::cerr << "ERROR opening " << frame_txt_path << std::endl;
            return 1;
        }
        hls::stream<data_t> image_in("image_in");
        hls::stream<IonCenter> centers_out("centers_out");
        int ion_count = 0;

        float val;
        int count = 0;
        while (infile >> val) {
            image_in.write((data_t)val);
            count++;
        }
        infile.close();

        if (count != INPUT_H * INPUT_W) {
            std::cerr << "ERROR: Read " << count << " pixels in " << frame_txt_path << std::endl;
            return 1;
        }

        dwnet_unet(image_in, centers_out, ion_count);

        out_file << "FRAME " << frame_idx << " " << ion_count << "\n";
        for (int i = 0; i < ion_count; i++) {
            IonCenter c = centers_out.read();
            out_file << c.x << " " << c.y << " " << c.area << "\n";
        }
        frame_idx++;
        if (frame_idx % 10 == 0) {
            std::cout << "Processed " << frame_idx << " frames in FPGA bit-accurate C-Sim..." << std::endl;
        }
    }
    out_file.close();
    std::cout << "Completed " << frame_idx << " frames." << std::endl;
    return 0;
}
