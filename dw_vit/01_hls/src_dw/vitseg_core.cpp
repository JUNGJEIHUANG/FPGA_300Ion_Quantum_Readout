// Decoder-only factorization, isolated candidate; baseline board image unchanged.
// Complete low-throughput candidate, NOT yet synthesis/RTL/board validated.
// W8/A18 storage, exact wide integer MACs, FP32 LayerNorm/Softmax, Q10.8 GELU LUT.
#include "vitseg_core.h"
#include <hls_math.h>
#ifdef __SYNTHESIS__
typedef ap_int<18> act_t;
typedef ap_int<48> acc_t;
typedef ap_int<64> wide_t;
typedef ap_uint<18> prob_t;
#else
typedef int act_t;
typedef long long acc_t;
typedef long long wide_t;
typedef unsigned int prob_t;
int vit_csim_logits[88*456];
#endif
#include "vit_parameters.h"
static const int H=88,W=456,N=627,D=64,MAX_DOT=128;
static const int DEC_DILATION=1;
// sigmoid(logit)>0.55, on Q10.8 values, is exactly raw_logit>51.
static const int LOGIT_THRESHOLD_RAW=MODEL_THRESHOLD_RAW;

static act_t sat18(wide_t x) {
#pragma HLS INLINE
    if(x>131071)return 131071;
    if(x<-131072)return -131072;
    return (act_t)x;
}
static act_t qfloat(float x) {
#pragma HLS INLINE off
    if(x>=511.99609375f)return 131071;
    if(x<=-512.f)return -131072;
    return (act_t)(int)hls::floorf(x*256.f+.5f);
}
static act_t gelu(act_t x) {
#pragma HLS INLINE off
#pragma HLS BIND_STORAGE variable=GELU_POS type=rom_1p impl=bram
    int z=(int)x;
    if(z>=2048)return x;
    if(z<=-2048)return 0;
    return z>=0?(act_t)GELU_POS[z]:(act_t)((int)GELU_POS[-z]+z);
}
static act_t requant(acc_t sum,int bias,int exponent) {
#pragma HLS INLINE off
    // All supported values fit signed64: at most576 A18*W8 products and
    // exporter-enforced exponent[-24,8]. Ties round toward positive infinity.
    wide_t v;int shift;
    if(exponent>=-8) { v=(wide_t)sum*((wide_t)1<<(exponent+8))+(wide_t)bias;shift=8; }
    else { shift=-exponent;v=(wide_t)sum+(wide_t)bias*((wide_t)1<<(shift-8)); }
    return sat18((v+((wide_t)1<<(shift-1)))>>shift);
}
static act_t dot_w8(const act_t vec[MAX_DOT],MatId id,int row) {
#pragma HLS INLINE off
#pragma HLS BIND_STORAGE variable=W_BANK0 type=rom_1p impl=bram
#pragma HLS BIND_STORAGE variable=W_BANK1 type=rom_1p impl=bram
#pragma HLS BIND_STORAGE variable=W_BANK2 type=rom_1p impl=bram
#pragma HLS BIND_STORAGE variable=W_BANK3 type=rom_1p impl=bram
    MatSpec m=MATS[(int)id];acc_t sum=0;
    DOT_BLOCK: for(int block=0;block<m.blocks;block++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=1 max=8
        int address=m.word_offset+row*m.blocks+block;
        int p[16];
#pragma HLS ARRAY_PARTITION variable=p complete
#ifdef __SYNTHESIS__
        ap_uint<128> packed;
        packed.range(31,0)=W_BANK0[address];packed.range(63,32)=W_BANK1[address];
        packed.range(95,64)=W_BANK2[address];packed.range(127,96)=W_BANK3[address];
#endif
        for(int lane=0;lane<16;lane++) {
#pragma HLS UNROLL
#ifdef __SYNTHESIS__
            ap_int<8> weight=packed.range(lane*8+7,lane*8);
            p[lane]=(ap_int<26>)(vec[block*16+lane]*weight);
#else
            p[lane]=vec[block*16+lane]*(int)W_NATIVE[address*16+lane];
#endif
        }
        for(int lane=0;lane<8;lane++) {
#pragma HLS UNROLL
            p[lane]=p[2*lane]+p[2*lane+1];
        }
        for(int lane=0;lane<4;lane++) {
#pragma HLS UNROLL
            p[lane]=p[2*lane]+p[2*lane+1];
        }
        p[0]=(p[0]+p[1])+(p[2]+p[3]);sum+=(acc_t)p[0];
    }
    int bi=(id==DEC0)?(row&63):row;
    return requant(sum,BIAS[m.bias_offset+bi],m.exponent);
}
static void norm64(const act_t input[64],act_t output[64],int gamma,int beta) {
#pragma HLS INLINE off
#pragma HLS BIND_STORAGE variable=NORM18 type=rom_2p impl=bram
    int raw_sum=0;
    for(int c=0;c<64;c++)raw_sum+=(int)input[c];
    float mean=(float)raw_sum*(1.f/16384.f),variance=0.f;
    for(int c=0;c<64;c++) { float d=(float)input[c]*(1.f/256.f)-mean;variance+=d*d; }
    float inv=1.f/hls::sqrtf(variance*(1.f/64.f)+1e-5f);
    for(int c=0;c<64;c++) {
        float d=(float)input[c]*(1.f/256.f)-mean;
        output[c]=qfloat(d*inv*((float)NORM18[gamma-40128+c]*(1.f/256.f))+(float)NORM18[beta-40128+c]*(1.f/256.f));
    }
}
static acc_t dot_qk(const act_t query[32],const act_t key[32]) {
#pragma HLS INLINE off
    acc_t sum=0;
    for(int block=0;block<2;block++) {
#pragma HLS PIPELINE II=1
        acc_t p[16];
#pragma HLS ARRAY_PARTITION variable=p complete
        for(int lane=0;lane<16;lane++) {
#pragma HLS UNROLL
#ifdef __SYNTHESIS__
            p[lane]=(ap_int<36>)(query[block*16+lane]*key[block*16+lane]);
#else
            p[lane]=(long long)query[block*16+lane]*key[block*16+lane];
#endif
        }
        for(int i=0;i<8;i++) {
#pragma HLS UNROLL
            p[i]=p[2*i]+p[2*i+1];
        }
        for(int i=0;i<4;i++) {
#pragma HLS UNROLL
            p[i]=p[2*i]+p[2*i+1];
        }
        sum+=(p[0]+p[1])+(p[2]+p[3]);
    }
    return sum;
}
static void attention_head(const act_t shared[N*192],int token,int head,act_t context[64]) {
#pragma HLS INLINE off
    act_t query[32],key[32];
#pragma HLS ARRAY_PARTITION variable=query cyclic factor=16
#pragma HLS ARRAY_PARTITION variable=key cyclic factor=16
    float scores[N];
    prob_t probabilities[N];
#pragma HLS BIND_STORAGE variable=scores type=ram_2p impl=bram
#pragma HLS BIND_STORAGE variable=probabilities type=ram_2p impl=bram
    for(int c=0;c<32;c++)query[c]=shared[token*192+head*32+c];
    float maximum=-3.4e38f;
    for(int j=0;j<N;j++) {
        for(int c=0;c<32;c++)key[c]=shared[j*192+64+head*32+c];
        float score=(float)dot_qk(query,key)*(1.f/65536.f)*0.1767766952966369f;
        scores[j]=score;if(score>maximum)maximum=score;
    }
    float total=0.f;
    for(int j=0;j<N;j++) { float value=hls::expf(scores[j]-maximum);scores[j]=value;total+=value; }
    for(int j=0;j<N;j++) {
        int p=(int)hls::floorf(scores[j]/total*131072.f+.5f);
        probabilities[j]=(prob_t)(p<0?0:(p>262143?262143:p));
    }
    acc_t accumulator[32];
#pragma HLS ARRAY_PARTITION variable=accumulator complete
    for(int c=0;c<32;c++)accumulator[c]=0;
    for(int j=0;j<N;j++) {
        for(int block=0;block<2;block++) {
#pragma HLS PIPELINE II=1
            for(int lane=0;lane<16;lane++) {
#pragma HLS UNROLL
                int c=block*16+lane;
#ifdef __SYNTHESIS__
                accumulator[c]+=(ap_int<37>)((ap_int<19>)probabilities[j]*shared[j*192+128+head*32+c]);
#else
                accumulator[c]+=(long long)probabilities[j]*shared[j*192+128+head*32+c];
#endif
            }
        }
    }
    for(int c=0;c<32;c++)context[head*32+c]=sat18(((wide_t)accumulator[c]+65536)>>17);
}
static void decoder_row(int row,const act_t tokens[N][64],act_t shared[N*192],act_t work[MAX_DOT]) {
#pragma HLS INLINE
    for(int x=0;x<W;x++) {
        int token=(row/8)*57+x/8;
        if((x&7)==0)for(int c=0;c<64;c++)work[c]=tokens[token][c];
        int k=(row&7)*8+(x&7);
        for(int c=0;c<64;c++)shared[((row/DEC_DILATION)%3)*W*64+x*64+c]=gelu(dot_w8(work,DEC0,k*64+c));
    }
}
static void components(bool foreground[H*W],act_t queue[N][64],hls::stream<IonCenter>& output,int& count) {
#pragma HLS INLINE off
    // Full-frame visited mask + bounded BFS avoids the old400-label overflow
    // and bounded union-find traversal; every accepted component is exact4-CCL.
    // Tokens are dead after decoding. Each BFS pixel index fits unsigned16 and signed18.
    count=0;
    for(int start=0;start<H*W;start++) {
        if(!foreground[start])continue;
        int head=0,tail=1;queue[0][0]=(act_t)start;foreground[start]=false;
        unsigned int sx=0,sy=0,area=0;
        while(head<tail) {
#pragma HLS LOOP_TRIPCOUNT min=1 max=40128
            int index=head++;int pixel=(int)queue[index>>6][index&63],r=pixel/W,c=pixel%W;area++;sx+=c;sy+=r;
            int neighbors[4]={pixel-1,pixel+1,pixel-W,pixel+W};
            bool valid[4]={c>0,c+1<W,r>0,r+1<H};
            for(int n=0;n<4;n++)if(valid[n]&&foreground[neighbors[n]]) {
                foreground[neighbors[n]]=false;queue[tail>>6][tail&63]=(act_t)neighbors[n];tail++;
            }
        }
        if(area>=2&&area<=45) {
            IonCenter center;center.x=(float)sx/(float)area;center.y=(float)sy/(float)area;center.area=(int)area;
            output.write(center);count++;
        }
    }
}
void vitseg_core(hls::stream<data_t>& image_in,hls::stream<IonCenter>& centers_out,int& ion_count) {
#pragma HLS INTERFACE axis port=image_in
#pragma HLS INTERFACE axis port=centers_out
#pragma HLS INTERFACE s_axilite port=ion_count bundle=ctrl
#pragma HLS INTERFACE s_axilite port=return bundle=ctrl
#pragma HLS ALLOCATION function instances=dot_w8 limit=1
#pragma HLS ALLOCATION function instances=norm64 limit=1
#pragma HLS ALLOCATION function instances=gelu limit=1
#pragma HLS ALLOCATION function instances=attention_head limit=1
    static act_t tokens[N][64],shared[N*192],patch_rows[8][W];
    static bool foreground[H*W];
#pragma HLS BIND_STORAGE variable=tokens type=ram_2p impl=bram
#pragma HLS BIND_STORAGE variable=shared type=ram_2p impl=bram
#pragma HLS BIND_STORAGE variable=POSITION18 type=rom_1p impl=bram
#pragma HLS BIND_STORAGE variable=patch_rows type=ram_2p impl=bram
#pragma HLS BIND_STORAGE variable=foreground type=ram_2p impl=bram
#pragma HLS ARRAY_PARTITION variable=shared cyclic factor=16 dim=1
    act_t work[MAX_DOT],norm_input[64],norm_output[64],residual[64],context[64],ffn[128],hidden[32],dw[64];
#pragma HLS ARRAY_PARTITION variable=work cyclic factor=16
    for(int ty=0;ty<11;ty++) {
        for(int r=0;r<8;r++)for(int c=0;c<W;c++) {
            data_t value=image_in.read();ap_int<18> raw=value.range(17,0);patch_rows[r][c]=(act_t)raw;
        }
        for(int tx=0;tx<57;tx++) {
            for(int r=0;r<8;r++)for(int c=0;c<8;c++)work[r*8+c]=patch_rows[r][tx*8+c];
            int token=ty*57+tx;
            for(int d=0;d<64;d++)tokens[token][d]=sat18((wide_t)dot_w8(work,PATCH,d)+(wide_t)POSITION18[token*64+d]);
        }
    }
    for(int layer=0;layer<2;layer++) {
        int g1=layer?L1_N1_GAMMA:L0_N1_GAMMA,b1=layer?L1_N1_BETA:L0_N1_BETA;
        int g2=layer?L1_N2_GAMMA:L0_N2_GAMMA,b2=layer?L1_N2_BETA:L0_N2_BETA;
        MatId qm=layer?L1_QKV:L0_QKV,om=layer?L1_OUT:L0_OUT;
        MatId f1=layer?L1_FF1:L0_FF1,f2=layer?L1_FF2:L0_FF2;
        for(int t=0;t<N;t++) {
            for(int d=0;d<64;d++)norm_input[d]=tokens[t][d];
            norm64(norm_input,norm_output,g1,b1);
            for(int d=0;d<64;d++)work[d]=norm_output[d];
            for(int d=0;d<192;d++)shared[t*192+d]=dot_w8(work,qm,d);
        }
        // QKV captures all pre-attention tokens, so the residual/FFN token
        // updates are safe in-place. No full FFN or second-token tensor stored.
        for(int t=0;t<N;t++) {
            attention_head(shared,t,0,context);attention_head(shared,t,1,context);
            for(int d=0;d<64;d++)work[d]=context[d];
            for(int d=0;d<64;d++)residual[d]=sat18((wide_t)tokens[t][d]+(wide_t)dot_w8(work,om,d));
            norm64(residual,norm_output,g2,b2);
            for(int d=0;d<64;d++)work[d]=norm_output[d];
            for(int d=0;d<128;d++)ffn[d]=gelu(dot_w8(work,f1,d));
            for(int d=0;d<128;d++)work[d]=ffn[d];
            for(int d=0;d<64;d++)tokens[t][d]=sat18((wide_t)residual[d]+(wide_t)dot_w8(work,f2,d));
        }
    }
    for(int t=0;t<N;t++) {
        for(int d=0;d<64;d++)norm_input[d]=tokens[t][d];
        norm64(norm_input,norm_output,FINAL_GAMMA,FINAL_BETA);
        for(int d=0;d<64;d++)tokens[t][d]=norm_output[d];
    }
    for(int parity=0;parity<DEC_DILATION;parity++) {
        decoder_row(parity,tokens,shared,work);
        for(int y=parity;y<H;y+=DEC_DILATION) {
            if(y+DEC_DILATION<H)decoder_row(y+DEC_DILATION,tokens,shared,work);
            for(int x=0;x<W;x++) {
                for(int c=0;c<64;c++) {
                    for(int ky=0;ky<3;ky++)for(int kx=0;kx<3;kx++) {
                        int yy=y+(ky-1)*DEC_DILATION,xx=x+(kx-1)*DEC_DILATION;
                        work[ky*3+kx]=(yy>=0&&yy<H&&xx>=0&&xx<W)?
                            shared[((yy/DEC_DILATION)%3)*W*64+xx*64+c]:(act_t)0;
                    }
                    for(int k=9;k<16;k++)work[k]=0;
                    dw[c]=dot_w8(work,DEC_DW,c);
                }
                for(int c=0;c<64;c++)work[c]=dw[c];
                for(int c=0;c<32;c++)hidden[c]=gelu(dot_w8(work,DEC_PW,c));
                for(int c=0;c<32;c++)work[c]=hidden[c];
                act_t logit=dot_w8(work,DEC4,0);foreground[y*W+x]=((int)logit>LOGIT_THRESHOLD_RAW);
#ifndef __SYNTHESIS__
                vit_csim_logits[y*W+x]=(int)logit;
#endif
            }
        }
    }
    components(foreground,tokens,centers_out,ion_count);
}
#ifndef __SYNTHESIS__
int vit_ccl_selftest() {
    static bool mask[H*W];static act_t test_queue[N][64];hls::stream<IonCenter> output;int count=0;
    for(int i=0;i<H*W;i++)mask[i]=false;
    components(mask,test_queue,output,count);if(count!=0||!output.empty())return 1;
    for(int i=0;i<H*W;i++)mask[i]=true;
    components(mask,test_queue,output,count);if(count!=0||!output.empty())return 2;
    for(int i=0;i<H*W;i++)mask[i]=false;
    for(int y=0;y<H;y+=2)for(int x=0;x+1<W;x+=3)mask[y*W+x]=mask[y*W+x+1]=true;
    components(mask,test_queue,output,count);if(count!=6688)return 3;
    while(!output.empty())if(output.read().area!=2)return 4;
    for(int i=0;i<H*W;i++)mask[i]=false;
    mask[0]=mask[1]=mask[W+1]=true;
    components(mask,test_queue,output,count);if(count!=1)return 5;
    IonCenter c=output.read();
    if(c.area!=3||c.x!=2.f/3.f||c.y!=1.f/3.f)return 6;
    return 0;
}
#endif
