import os
import sys
import numpy as np

sys.path.append(r"E:\AAA_Project_300ions_CNNDetect")
import torch
import torch.nn as nn
from nets.DWNetV2_unet import DWNetV2_unet

# Create folders if not existing
hls_dir = r"E:\AAA_Project_300ions_CNNDetect\fpga_hls"
os.makedirs(hls_dir, exist_ok=True)

# 1. Load Trained Model
print("Loading model and checkpoint...")
model = DWNetV2_unet(pre_trained=None, width_mult=0.5)
state_dict = torch.load(r"E:\AAA_Project_300ions_CNNDetect\outputs\scaled_model_run\sample_3000\best.pth", map_location="cpu")
model.load_state_dict(state_dict)
model.eval()

weights_header_path = os.path.join(hls_dir, "dwnet_weights.h")
unet_cpp_path = os.path.join(hls_dir, "dwnet_unet.cpp")
unet_h_path = os.path.join(hls_dir, "dwnet_unet.h")

# Helper for BN Folding
def fold_conv_bn(conv, bn):
    w = conv.weight.data
    b = conv.bias.data if conv.bias is not None else torch.zeros(conv.out_channels)
    
    gamma = bn.weight.data
    beta = bn.bias.data
    mean = bn.running_mean.data
    var = bn.running_var.data
    eps = bn.eps
    
    scale = gamma / torch.sqrt(var + eps)
    
    if isinstance(conv, nn.ConvTranspose2d):
        w_folded = w * scale.view(1, -1, 1, 1)
    else:
        w_folded = w * scale.view(-1, 1, 1, 1)
        
    b_folded = beta + (b - mean) * scale
    return w_folded.numpy(), b_folded.numpy()

# 2. Extract and Fold all layers
layers_data = {}

# Backbone Stage 0
print("Extracting Stage 0...")
w_feat0, b_feat0 = fold_conv_bn(model.backbone.features[0][0], model.backbone.features[0][1])
layers_data["w_feat0"] = (w_feat0, "weight_t", ["16", "INPUT_C", "3", "3"])
layers_data["b_feat0"] = (b_feat0, "data_t", ["16"])

# Backbone Stage 1 (expand_ratio=1)
print("Extracting Stage 1...")
w_feat1_dw, b_feat1_dw = fold_conv_bn(model.backbone.features[1].conv[0], model.backbone.features[1].conv[1])
w_feat1_pw, b_feat1_pw = fold_conv_bn(model.backbone.features[1].conv[3], model.backbone.features[1].conv[4])
layers_data["w_feat1_dw"] = (w_feat1_dw[:, 0, :, :], "weight_dw_t", ["16", "3", "3"]) # depthwise weight shape: (C, K, K)
layers_data["b_feat1_dw"] = (b_feat1_dw, "data_t", ["16"])
layers_data["w_feat1_pw"] = (w_feat1_pw[:, :, 0, 0], "weight_pw_t", ["C1", "16"]) # pointwise weight shape: (OUT, IN)
layers_data["b_feat1_pw"] = (b_feat1_pw, "data_t", ["C1"])

# Backbone Stages 2..17
for i in range(2, 18):
    print(f"Extracting Stage {i}...")
    feat_layer = model.backbone.features[i]
    w_pw1, b_pw1 = fold_conv_bn(feat_layer.conv[0], feat_layer.conv[1])
    w_dw, b_dw = fold_conv_bn(feat_layer.conv[3], feat_layer.conv[4])
    w_pw2, b_pw2 = fold_conv_bn(feat_layer.conv[6], feat_layer.conv[7])
    
    in_c = feat_layer.conv[0].in_channels
    hidden_c = feat_layer.conv[0].out_channels
    out_c = feat_layer.conv[6].out_channels
    
    layers_data[f"w_feat{i}_pw1"] = (w_pw1[:, :, 0, 0], "weight_pw_t", [str(hidden_c), str(in_c)])
    layers_data[f"b_feat{i}_pw1"] = (b_pw1, "data_t", [str(hidden_c)])
    layers_data[f"w_feat{i}_dw"] = (w_dw[:, 0, :, :], "weight_dw_t", [str(hidden_c), "3", "3"])
    layers_data[f"b_feat{i}_dw"] = (b_dw, "data_t", [str(hidden_c)])
    layers_data[f"w_feat{i}_pw2"] = (w_pw2[:, :, 0, 0], "weight_pw_t", [str(out_c), str(hidden_c)])
    layers_data[f"b_feat{i}_pw2"] = (b_pw2, "data_t", [str(out_c)])

# Backbone Stage 18
print("Extracting Stage 18...")
w_feat18, b_feat18 = fold_conv_bn(model.backbone.features[18][0], model.backbone.features[18][1])
layers_data["w_feat18"] = (w_feat18[:, :, 0, 0], "weight_pw_t", ["C5", "160"])
layers_data["b_feat18"] = (b_feat18, "data_t", ["C5"])

# Decoders dconv1..4 (no BN)
decoders_c_in = [640, 48, 16, 12]
decoders_c_out = [48, 16, 12, 8]
for d in range(1, 5):
    print(f"Extracting Decoder dconv{d}...")
    dconv = getattr(model, f"dconv{d}")
    w = dconv.weight.data.numpy()
    b = dconv.bias.data.numpy() if dconv.bias is not None else np.zeros(dconv.out_channels)
    
    in_c = decoders_c_in[d-1]
    out_c = decoders_c_out[d-1]
    # ConvTranspose2D weights: [IN_C][OUT_C][K][K]
    layers_data[f"w_dconv{d}"] = (w, "weight_dconv_t", [str(in_c), str(out_c), "4", "4"])
    layers_data[f"b_dconv{d}"] = (b, "data_t", [str(out_c)])

# Decoders invres1..4
invres_in_c = [96, 32, 24, 16]
invres_out_c = [48, 16, 12, 8]
for d in range(1, 5):
    print(f"Extracting Decoder invres{d}...")
    invres = getattr(model, f"invres{d}")
    w_pw1, b_pw1 = fold_conv_bn(invres.conv[0], invres.conv[1])
    w_dw, b_dw = fold_conv_bn(invres.conv[3], invres.conv[4])
    w_pw2, b_pw2 = fold_conv_bn(invres.conv[6], invres.conv[7])
    
    in_c = invres_in_c[d-1]
    hidden_c = invres.conv[0].out_channels
    out_c = invres_out_c[d-1]
    
    layers_data[f"w_invres{d}_pw1"] = (w_pw1[:, :, 0, 0], "weight_pw_t", [str(hidden_c), str(in_c)])
    layers_data[f"b_invres{d}_pw1"] = (b_pw1, "data_t", [str(hidden_c)])
    layers_data[f"w_invres{d}_dw"] = (w_dw[:, 0, :, :], "weight_dw_t", [str(hidden_c), "3", "3"])
    layers_data[f"b_invres{d}_dw"] = (b_dw, "data_t", [str(hidden_c)])
    layers_data[f"w_invres{d}_pw2"] = (w_pw2[:, :, 0, 0], "weight_pw_t", [str(out_c), str(hidden_c)])
    layers_data[f"b_invres{d}_pw2"] = (b_pw2, "data_t", [str(out_c)])

# Final layers (no BN)
print("Extracting final layers...")
layers_data["w_conv_last"] = (model.conv_last.weight.data.numpy()[:, :, 0, 0], "weight_t", ["3", "C1"])
layers_data["b_conv_last"] = (model.conv_last.bias.data.numpy(), "data_t", ["3"])
layers_data["w_conv_score"] = (model.conv_score.weight.data.numpy()[:, :, 0, 0], "weight_t", ["1", "3"])
layers_data["b_conv_score"] = (model.conv_score.bias.data.numpy(), "data_t", ["1"])

# Helper for C++ array formatting (flat initializer to prevent GCC stack overflow)
def format_numpy_array(arr):
    flat_arr = arr.flatten()
    str_list = [f"{x:.6f}" for x in flat_arr]
    # Group by 20 elements per line for readability and compiler tolerance
    lines = []
    for i in range(0, len(str_list), 20):
        lines.append(", ".join(str_list[i:i+20]))
    return "{\n" + ",\n".join(lines) + "\n}"


# 3. Write dwnet_weights.h and separate .cpp files
print("Writing weight declarations and source files...")
weights_sub_dir = os.path.join(hls_dir, "weights")
os.makedirs(weights_sub_dir, exist_ok=True)

weight_cpp_files = []
with open(weights_header_path, "w", encoding="utf-8") as f:
    f.write("#ifndef DWNET_WEIGHTS_H\n")
    f.write("#define DWNET_WEIGHTS_H\n\n")
    f.write('#include "dwnet_types.h"\n')
    f.write('#include "dwnet_config.h"\n\n')
    
    for name, (arr, dtype, shape_dims) in layers_data.items():
        shape_str = "".join(f"[{d}]" for d in shape_dims)
        # Write extern declaration to header
        if name == "w_dconv1":
            f.write("#ifndef __SYNTHESIS__\n")
            f.write(f"extern {dtype} {name}{shape_str};\n")
            f.write("#else\n")
            f.write(f"extern const {dtype} {name}{shape_str};\n")
            f.write("#endif\n")
        else:
            f.write(f"extern const {dtype} {name}{shape_str};\n")
        
        # Write definition to separate .cpp file
        cpp_file_name = f"{name}.cpp"
        cpp_file_path = os.path.join(weights_sub_dir, cpp_file_name)
        weight_cpp_files.append(f"weights/{cpp_file_name}")
        with open(cpp_file_path, "w", encoding="utf-8") as cpp_f:
            cpp_f.write('#include "../dwnet_weights.h"\n\n')
            if name == "w_dconv1":
                # Save binary file for C-Simulation
                bin_path = os.path.join(weights_sub_dir, "w_dconv1.bin")
                arr.astype(np.float32).tofile(bin_path)
                print(f"Saved binary weights to {bin_path}")
                
                # Write conditional loading for C-Simulation and static array for C-Synthesis
                cpp_f.write("#ifndef __SYNTHESIS__\n")
                cpp_f.write("#include <fstream>\n")
                cpp_f.write("#include <iostream>\n\n")
                cpp_f.write(f"{dtype} {name}{shape_str};\n\n")
                cpp_f.write("struct LoadDConv1 {\n")
                cpp_f.write("    LoadDConv1() {\n")
                abs_bin_path = bin_path.replace("\\", "/")
                cpp_f.write(f'        std::ifstream f("{abs_bin_path}", std::ios::binary);\n')
                cpp_f.write("        if (!f) {\n")
                cpp_f.write(f'            std::cerr << "Error: Could not open {abs_bin_path}" << std::endl;\n')
                cpp_f.write("            exit(1);\n")
                cpp_f.write("        }\n")
                cpp_f.write("        float val;\n")
                cpp_f.write("        for (int i = 0; i < 640; ++i) {\n")
                cpp_f.write("            for (int j = 0; j < 48; ++j) {\n")
                cpp_f.write("                for (int k = 0; k < 4; ++k) {\n")
                cpp_f.write("                    for (int l = 0; l < 4; ++l) {\n")
                cpp_f.write("                        if (f.read(reinterpret_cast<char*>(&val), sizeof(float))) {\n")
                cpp_f.write(f"                            {name}[i][j][k][l] = val;\n")
                cpp_f.write("                        } else {\n")
                cpp_f.write('                            std::cerr << "Error: Unexpected EOF in binary weights" << std::endl;\n')
                cpp_f.write("                            exit(1);\n")
                cpp_f.write("                        }\n")
                cpp_f.write("                    }\n")
                cpp_f.write("                }\n")
                cpp_f.write("            }\n")
                cpp_f.write("        }\n")
                cpp_f.write("    }\n")
                cpp_f.write("} loader_dconv1;\n\n")
                cpp_f.write("#else\n\n")
                cpp_f.write(f"const {dtype} {name}{shape_str} = ")
                cpp_f.write(format_numpy_array(arr))
                cpp_f.write(";\n\n")
                cpp_f.write("#endif\n")
            else:
                cpp_f.write(f"const {dtype} {name}{shape_str} = ")
                cpp_f.write(format_numpy_array(arr))
                cpp_f.write(";\n")
            
    f.write("\n#endif // DWNET_WEIGHTS_H\n")
print("dwnet_weights.h and weight .cpp files written successfully!")

# Write run_hls.tcl
run_hls_path = os.path.join(hls_dir, "run_hls.tcl")
print("Writing run_hls.tcl...")
tcl_content = """# Vitis HLS 2022.2 Synthesis Script for xc7k325tffg900-2
open_project -reset proj_dwnet_unet

# Add design files
add_files dwnet_unet.cpp
add_files streaming_ccl.cpp
"""
for wf in weight_cpp_files:
    tcl_content += f"add_files {wf}\n"

tcl_content += """
# Add testbench
add_files -tb tb_dwnet_unet.cpp

# Set top-level function name
set_top dwnet_unet

# Create solution
open_solution -reset "solution1" -flow_target vivado

# Set target FPGA part and speed grade (Kintex-7 325T, -2 speed grade)
set_part {xc7k325tffg900-2}

# Target clock period: 5.0 ns (200 MHz)
create_clock -period 5.0 -name default

# Run HLS C-Simulation (behavioral test)
csim_design

# Run HLS C-Synthesis (translate to Verilog RTL)
# csynth_design

# Exit Vitis HLS
exit
"""
with open(run_hls_path, "w", encoding="utf-8") as tcl_f:
    tcl_f.write(tcl_content)
print("run_hls.tcl written successfully!")


# 4. Generate C++ Top Module dwnet_unet.cpp
print("Generating dwnet_unet.cpp...")

# First write dwnet_config.h
config_path = os.path.join(hls_dir, "dwnet_config.h")
print("Writing dwnet_config.h...")
with open(config_path, "w", encoding="utf-8") as config_f:
    config_f.write("""#ifndef DWNET_CONFIG_H
#define DWNET_CONFIG_H

// Input resolution configuration
#define INPUT_H 88
#define INPUT_W 456
#define INPUT_C 1

// Dynamic channel dimensions for width_mult=0.5
#define C1 8
#define C2 12
#define C3 16
#define C4 48
#define C5 640 // 1280 * 0.5

#endif // DWNET_CONFIG_H
""")

cpp_content = """#include "dwnet_unet.h"
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

    // Intermediate streams connecting the layers
"""

# Let's write the stream allocations
# 1. Skip connections
cpp_content += '    hls::stream<data_t> s_x1_skip("s_x1_skip");\n'
cpp_content += '    #pragma HLS stream variable=s_x1_skip depth=321024\n' # 88 * 456 * 8
cpp_content += '    hls::stream<data_t> s_x2_skip("s_x2_skip");\n'
cpp_content += '    #pragma HLS stream variable=s_x2_skip depth=120432\n' # 44 * 228 * 12
cpp_content += '    hls::stream<data_t> s_x3_skip("s_x3_skip");\n'
cpp_content += '    #pragma HLS stream variable=s_x3_skip depth=40128\n' # 22 * 114 * 16
cpp_content += '    hls::stream<data_t> s_x4_skip("s_x4_skip");\n'
cpp_content += '    #pragma HLS stream variable=s_x4_skip depth=30096\n\n' # 11 * 57 * 48

# 2. Main layer streams
streams_list = [
    ("s_conv0", 64),
    ("s_x1_conv", 64),
    ("s_feat1_dw", 64),
    ("s_feat1_pw", 64),
    ("s_feat2_pw1", 64),
    ("s_feat2_dw", 64),
    ("s_feat2_pw2", 64),
]

settings = [
    # (i, expand, oup, stride)
    (3, 6, 12, 1),
    (4, 6, 16, 2),
    (5, 6, 16, 1),
    (6, 6, 16, 1),
    (7, 6, 32, 2),
    (8, 6, 32, 1),
    (9, 6, 32, 1),
    (10, 6, 32, 1),
    (11, 6, 48, 1),
    (12, 6, 48, 1),
    (13, 6, 48, 1),
    (14, 6, 80, 2),
    (15, 6, 80, 1),
    (16, 6, 80, 1),
    (17, 6, 160, 1),
]

cur_h = 44
cur_w = 228
cur_in_c = 12

for i, expand, oup, stride in settings:
    if stride == 2:
        cur_h = (cur_h - 1) // 2 + 1
        cur_w = (cur_w - 1) // 2 + 1
    use_res = (stride == 1 and cur_in_c == oup)
    if use_res:
        streams_list.append((f"s_feat{i}_in_bypass", 4 * cur_w))
        streams_list.append((f"s_feat{i}_in_conv", 64))
    streams_list.append((f"s_feat{i}_pw1", 64))
    streams_list.append((f"s_feat{i}_dw", 64))
    streams_list.append((f"s_feat{i}_pw2", 64))
    streams_list.append((f"s_feat{i}_out", 64))
    if i in [3, 6, 13]:
        skip_num = 2 if i==3 else (3 if i==6 else 4)
        streams_list.append((f"s_x{skip_num}_conv", 64))
    cur_in_c = oup

streams_list.append(("s_feat18", 64))

# Decoder streams
streams_list.append(("s_dconv1", 64))
streams_list.append(("s_dconv1_resized", 64))
streams_list.append(("s_invres1_concat", 64))
streams_list.append(("s_invres1_pw1", 64))
streams_list.append(("s_invres1_dw", 64))
streams_list.append(("s_invres1_pw2", 64))

for d in range(2, 5):
    streams_list.append((f"s_dconv{d}", 64))
    streams_list.append((f"s_invres{d}_concat", 64))
    streams_list.append((f"s_invres{d}_pw1", 64))
    streams_list.append((f"s_invres{d}_dw", 64))
    streams_list.append((f"s_invres{d}_pw2", 64))

streams_list.append(("s_conv_last", 64))
streams_list.append(("s_conv_score", 64))

for s_name, depth in streams_list:
    cpp_content += f'    hls::stream<data_t> {s_name}("{s_name}");\n'
    cpp_content += f'    #pragma HLS stream variable={s_name} depth={depth}\n'

cpp_content += "\n    // -------------------------------------------------------------\n"
cpp_content += "    // ENCODER BACKBONE\n"
cpp_content += "    // -------------------------------------------------------------\n"

# Stage 0
cpp_content += "    // Stage 0: Conv 3x3, stride=1, padding=1\n"
cpp_content += "    stream_conv2d<INPUT_H, INPUT_W, INPUT_C, 16, 3, 1, 1>(image_in, s_conv0, w_feat0, b_feat0);\n\n"

# Stage 1
cpp_content += "    // Stage 1: Inverted Residual (16->8, stride=1, expand=1)\n"
cpp_content += "    stream_depthwise_conv2d<88, 456, 16, 3, 1, 1>(s_conv0, s_feat1_dw, w_feat1_dw, b_feat1_dw);\n"
cpp_content += "    stream_pointwise_conv2d<88, 456, 16, 8, false>(s_feat1_dw, s_feat1_pw, w_feat1_pw, b_feat1_pw);\n"
cpp_content += "    // x1 Skip connection\n"
cpp_content += "    duplicate_stream<88 * 456 * 8>(s_feat1_pw, s_x1_skip, s_x1_conv);\n\n"

# Stage 2
cpp_content += "    // Stage 2: Inverted Residual (8->12, stride=2, expand=6)\n"
cpp_content += "    stream_pointwise_conv2d<88, 456, 8, 48>(s_x1_conv, s_feat2_pw1, w_feat2_pw1, b_feat2_pw1);\n"
cpp_content += "    stream_depthwise_conv2d<88, 456, 48, 3, 2, 1>(s_feat2_pw1, s_feat2_dw, w_feat2_dw, b_feat2_dw);\n"
cpp_content += "    stream_pointwise_conv2d<44, 228, 48, 12, false>(s_feat2_dw, s_feat2_pw2, w_feat2_pw2, b_feat2_pw2);\n\n"

# Stages 3..17
cur_h = 44
cur_w = 228
cur_in_c = 12

for i, expand, oup, stride in settings:
    hidden_c = cur_in_c * expand
    use_res_connect = (stride == 1 and cur_in_c == oup)
    
    cpp_content += f"    // Stage {i}: Inverted Residual ({cur_in_c}->{oup}, stride={stride}, expand={expand})\n"
    
    input_stream = f"s_feat{i-1}_out" if i > 3 else "s_feat2_pw2"
    if i-1 == 3: input_stream = "s_x2_conv"
    if i-1 == 6: input_stream = "s_x3_conv"
    if i-1 == 13: input_stream = "s_x4_conv"
    
    if use_res_connect:
        cpp_content += f"    duplicate_stream<{cur_h} * {cur_w} * {cur_in_c}>({input_stream}, s_feat{i}_in_bypass, s_feat{i}_in_conv);\n"
        in_s = f"s_feat{i}_in_conv"
    else:
        in_s = input_stream
        
    cpp_content += f"    stream_pointwise_conv2d<{cur_h}, {cur_w}, {cur_in_c}, {hidden_c}>({in_s}, s_feat{i}_pw1, w_feat{i}_pw1, b_feat{i}_pw1);\n"
    cpp_content += f"    stream_depthwise_conv2d<{cur_h}, {cur_w}, {hidden_c}, 3, {stride}, 1>(s_feat{i}_pw1, s_feat{i}_dw, w_feat{i}_dw, b_feat{i}_dw);\n"
    
    if stride == 2:
        next_h = (cur_h - 1) // 2 + 1
        next_w = (cur_w - 1) // 2 + 1
    else:
        next_h = cur_h
        next_w = cur_w
        
    cpp_content += f"    stream_pointwise_conv2d<{next_h}, {next_w}, {hidden_c}, {oup}, false>(s_feat{i}_dw, s_feat{i}_pw2, w_feat{i}_pw2, b_feat{i}_pw2);\n"
    
    out_s = f"s_feat{i}_out"
    if use_res_connect:
        cpp_content += f"    stream_add<{next_h}, {next_w}, {oup}>(s_feat{i}_in_bypass, s_feat{i}_pw2, {out_s});\n"
    else:
        cpp_content += f"    // No residual connect\n"
        cpp_content += f"    // Simply rename output stream\n"
        cpp_content += f"    #define s_feat{i}_out s_feat{i}_pw2\n"
        
    if i in [3, 6, 13]:
        skip_num = 2 if i==3 else (3 if i==6 else 4)
        cpp_content += f"    // x{skip_num} Skip connection\n"
        cpp_content += f"    duplicate_stream<{next_h} * {next_w} * {oup}>({out_s}, s_x{skip_num}_skip, s_x{skip_num}_conv);\n"
        
    cpp_content += "\n"
    cur_h = next_h
    cur_w = next_w
    cur_in_c = oup

# Stage 18
cpp_content += "    // Stage 18: Conv 1x1 PW (160->640)\n"
cpp_content += f"    stream_pointwise_conv2d<{cur_h}, {cur_w}, 160, C5>(s_feat17_out, s_feat18, w_feat18, b_feat18);\n\n"

cpp_content += "    // -------------------------------------------------------------\n"
cpp_content += "    // DECODER\n"
cpp_content += "    // -------------------------------------------------------------\n"

# Decoder 1:
cpp_content += f"    // Decoder 1: TransposeConv ({cur_h}x{cur_w}x640 -> {cur_h*2}x{cur_w*2}x48) + BilinearResize -> 11x57 + Concat + InvertedResidual\n"
cpp_content += f"    stream_conv_transpose2d<{cur_h}, {cur_w}, C5, C4, 4, 2, 1>(s_feat18, s_dconv1, w_dconv1, b_dconv1);\n"
cpp_content += f"    stream_bilinear_resize<{cur_h*2}, {cur_w*2}, 11, 57, C4>(s_dconv1, s_dconv1_resized);\n"
cpp_content += "    concat_streams<11, 57, C4>(s_x4_skip, s_dconv1_resized, s_invres1_concat);\n"
cpp_content += "    stream_pointwise_conv2d<11, 57, 96, 576>(s_invres1_concat, s_invres1_pw1, w_invres1_pw1, b_invres1_pw1);\n"
cpp_content += "    stream_depthwise_conv2d<11, 57, 576, 3, 1, 1>(s_invres1_pw1, s_invres1_dw, w_invres1_dw, b_invres1_dw);\n"
cpp_content += "    stream_pointwise_conv2d<11, 57, 576, C4, false>(s_invres1_dw, s_invres1_pw2, w_invres1_pw2, b_invres1_pw2);\n\n"

# Decoder 2:
cpp_content += "    // Decoder 2: TransposeConv (11x57x48 -> 22x114x16) + Concat + InvertedResidual\n"
cpp_content += "    stream_conv_transpose2d<11, 57, C4, C3, 4, 2, 1>(s_invres1_pw2, s_dconv2, w_dconv2, b_dconv2);\n"
cpp_content += "    concat_streams<22, 114, C3>(s_x3_skip, s_dconv2, s_invres2_concat);\n"
cpp_content += "    stream_pointwise_conv2d<22, 114, 32, 192>(s_invres2_concat, s_invres2_pw1, w_invres2_pw1, b_invres2_pw1);\n"
cpp_content += "    stream_depthwise_conv2d<22, 114, 192, 3, 1, 1>(s_invres2_pw1, s_invres2_dw, w_invres2_dw, b_invres2_dw);\n"
cpp_content += "    stream_pointwise_conv2d<22, 114, 192, C3, false>(s_invres2_dw, s_invres2_pw2, w_invres2_pw2, b_invres2_pw2);\n\n"

# Decoder 3:
cpp_content += "    // Decoder 3: TransposeConv (22x114x16 -> 44x228x12) + Concat + InvertedResidual\n"
cpp_content += "    stream_conv_transpose2d<22, 114, C3, C2, 4, 2, 1>(s_invres2_pw2, s_dconv3, w_dconv3, b_dconv3);\n"
cpp_content += "    concat_streams<44, 228, C2>(s_x2_skip, s_dconv3, s_invres3_concat);\n"
cpp_content += "    stream_pointwise_conv2d<44, 228, 24, 144>(s_invres3_concat, s_invres3_pw1, w_invres3_pw1, b_invres3_pw1);\n"
cpp_content += "    stream_depthwise_conv2d<44, 228, 144, 3, 1, 1>(s_invres3_pw1, s_invres3_dw, w_invres3_dw, b_invres3_dw);\n"
cpp_content += "    stream_pointwise_conv2d<44, 228, 144, C2, false>(s_invres3_dw, s_invres3_pw2, w_invres3_pw2, b_invres3_pw2);\n\n"

# Decoder 4:
cpp_content += "    // Decoder 4: TransposeConv (44x228x12 -> 88x456x8) + Concat + InvertedResidual\n"
cpp_content += "    stream_conv_transpose2d<44, 228, C2, C1, 4, 2, 1>(s_invres3_pw2, s_dconv4, w_dconv4, b_dconv4);\n"
cpp_content += "    concat_streams<88, 456, C1>(s_x1_skip, s_dconv4, s_invres4_concat);\n"
cpp_content += "    stream_pointwise_conv2d<88, 456, 16, 96>(s_invres4_concat, s_invres4_pw1, w_invres4_pw1, b_invres4_pw1);\n"
cpp_content += "    stream_depthwise_conv2d<88, 456, 96, 3, 1, 1>(s_invres4_pw1, s_invres4_dw, w_invres4_dw, b_invres4_dw);\n"
cpp_content += "    stream_pointwise_conv2d<88, 456, 96, C1, false>(s_invres4_dw, s_invres4_pw2, w_invres4_pw2, b_invres4_pw2);\n\n"

# Final layers
cpp_content += "    // Final Conv layers\n"
cpp_content += "    stream_pointwise_conv2d<88, 456, C1, 3, false>(s_invres4_pw2, s_conv_last, w_conv_last, b_conv_last);\n"
cpp_content += "    stream_pointwise_conv2d<88, 456, 3, 1, false>(s_conv_last, s_conv_score, w_conv_score, b_conv_score);\n\n"

cpp_content += "    // Post-processing\n"
cpp_content += "    streaming_ccl(s_conv_score, centers_out, ion_count);\n"
cpp_content += "}\n"

with open(unet_cpp_path, "w", encoding="utf-8") as f:
    f.write(cpp_content)
print("dwnet_unet.cpp written successfully!")

# 5. Write dwnet_unet.h
print("Writing dwnet_unet.h...")
h_content = """#ifndef DWNET_UNET_H
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
"""
with open(unet_h_path, "w", encoding="utf-8") as f:
    f.write(h_content)
print("dwnet_unet.h written successfully!")

print("\n--- ALL CODE GENERATION COMPLETED SUCCESSFULLY ---")
