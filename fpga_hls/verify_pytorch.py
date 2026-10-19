import sys
import os
import cv2
import numpy as np
import torch

sys.path.append(r"E:\AAA_Project_300ions_CNNDetect")
from nets.DWNetV2_unet import DWNetV2_unet
from dataset import extract_centers_from_mask

# 1. Load image and normalize
img_path = r"E:\AAA_Project_300ions_CNNDetect\Test_Dataset\TEST_1000_Easy\000000.png"
img_np = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
img_tensor = torch.from_numpy(img_np).float() / 255.0
img_tensor = img_tensor.unsqueeze(0).unsqueeze(0) # shape (1, 1, 224, 224)

# 2. Save pixel values as text for HLS testbench
txt_path = r"E:\AAA_Project_300ions_CNNDetect\fpga_hls\test_input.txt"
np.savetxt(txt_path, img_tensor.squeeze().numpy().flatten(), fmt="%.6f")
print(f"Saved normalized image pixels to {txt_path}")

# 3. Load model and run inference
model = DWNetV2_unet(pre_trained=None, width_mult=0.5)
state_dict = torch.load(r"E:\AAA_Project_300ions_CNNDetect\outputs\scaled_model_run\sample_3000\best.pth", map_location="cpu")
model.load_state_dict(state_dict)
model.eval()

with torch.no_grad():
    logits = model(img_tensor)
    # Binarize logits
    pred_mask = (logits > 0).float()
    
# Extract centers from mask
mask_np = (pred_mask.squeeze().numpy() * 255).astype(np.uint8)
centers, valid = extract_centers_from_mask(mask_np)

print("\n--- PyTorch Inference Results ---")
ion_count = int(valid.sum())
print(f"Predicted Ion Count: {ion_count}")
for i in range(ion_count):
    print(f"Ion {i+1}: X={centers[i, 0]:.2f}, Y={centers[i, 1]:.2f}")
