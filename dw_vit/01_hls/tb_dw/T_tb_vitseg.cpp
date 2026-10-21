#include "../src_dw/vitseg_core.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <cstdlib>
int main(int argc,char** argv) {
    if(argc!=3) { std::cerr<<"Expected input_q18.bin and output prefix\n";return 2; }
    int unit=vit_ccl_selftest();
    if(unit) { std::cerr<<"CCL_UNIT_FAIL "<<unit<<"\n";return 3; }
    std::cout<<"CCL_UNIT_PASS: empty,40128-pixel blob,6688 components,centroid\n";
    std::vector<int32_t> input(88*456);
    std::ifstream in(argv[1],std::ios::binary);
    in.read((char*)input.data(),input.size()*sizeof(int32_t));
    if(in.gcount()!=(std::streamsize)(input.size()*sizeof(int32_t)))return 4;
    hls::stream<data_t> images;hls::stream<IonCenter> centers;
    for(size_t i=0;i<input.size();i++) {
        data_t p;p.range(17,0)=(ap_uint<18>)(input[i]&0x3ffff);images.write(p);
    }
    int count=-1;auto start=std::chrono::steady_clock::now();
    vitseg_core(images,centers,count);
    double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    std::string prefix(argv[2]);
    std::ofstream logits(prefix+"_logits_q18.bin",std::ios::binary);
    logits.write((const char*)vit_csim_logits,88*456*sizeof(int));
    std::ofstream coords(prefix+"_centers.tsv");coords<<"x\ty\tarea\n"<<std::setprecision(9);
    std::ofstream raw(prefix+"_centers_raw96.hex");raw<<std::hex<<std::setfill('0');
    int received=0;
    while(!centers.empty()) {
        IonCenter c=centers.read();received++;
        coords<<c.x<<"\t"<<c.y<<"\t"<<c.area<<"\n";
        uint32_t x,y;std::memcpy(&x,&c.x,4);std::memcpy(&y,&c.y,4);
        raw<<std::setw(8)<<(uint32_t)c.area<<std::setw(8)<<y<<std::setw(8)<<x<<"\n";
    }
    if(received!=count||!images.empty())return 5;
    std::cout<<"FULL_VIT_CSIM_COMPLETE pixels="<<input.size()<<" centers="<<count<<" seconds="<<seconds<<"\n";
    return 0;
}
