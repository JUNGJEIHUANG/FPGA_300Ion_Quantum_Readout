import copy
import json
import sys
from pathlib import Path
from PIL import Image
import cv2
import numpy as np
import torch
import torch.nn as nn
from sklearn.metrics import roc_curve

sys.path.insert(0, r"E:\05_Project_300ions_CNNDetect")
from nets.DWNetV2_unet import DWNetV2_unet
from nets.StandardUNet import StandardUNet
from nets.ViTSeg import ViTSeg
from nets.psfanet import PSFANet

DATASET_F = Path(r"F:\00_Site_AIT\300-Qubits-Register-Dataset")
WEIGHTS_ROOT = Path(r"F:\01_DWUnet\DL-Ion-Readout-anon_code\model_weights")

def read_gray(path: Path) -> np.ndarray:
    with Image.open(path) as im:
        return np.array(im.convert("L"), dtype=np.uint8)

def cross_scores_single(prob: np.ndarray, positions: np.ndarray) -> np.ndarray:
    H, W = prob.shape
    scores = np.empty(len(positions), dtype=np.float32)
    for idx, (xf, yf) in enumerate(positions):
        x = int(np.clip(round(float(xf)), 1, W - 2))
        y = int(np.clip(round(float(yf)), 1, H - 2))
        scores[idx] = (prob[y, x] + prob[y - 1, x] + prob[y + 1, x] + prob[y, x - 1] + prob[y, x + 1]) / 5.0
    return scores

def load_f_subset(cat_dir: Path, label_dir: Path, prefix: str, indices: range):
    imgs, gt_states, gt_xy, gt_counts = [], [], [], []
    for idx in indices:
        img_p = cat_dir / f"{prefix}_{idx:04d}.png"
        lbl_p = label_dir / f"{prefix}_{idx:04d}_label.json"
        im = read_gray(img_p)
        ann = json.load(lbl_p.open(encoding="utf-8"))
        ions = ann["ions"]
        st = np.array([ion["state"] for ion in ions], dtype=np.uint8)
        xy = np.array([[float(ion["x"]), float(ion["y"])] for ion in ions], dtype=np.float32)
        imgs.append(im)
        gt_states.append(st)
        gt_xy.append(xy)
        gt_counts.append(int(ann["bright_count"]))
    return np.stack(imgs), np.stack(gt_states), gt_xy, np.array(gt_counts)

def fuse_bn_inplace(mod: nn.Module):
    for _, child in mod.named_children():
        if isinstance(child, nn.Sequential):
            i = 0
            while i < len(child) - 1:
                if isinstance(child[i], nn.Conv2d) and isinstance(child[i + 1], nn.BatchNorm2d):
                    child[i] = torch.nn.utils.fusion.fuse_conv_bn_eval(child[i], child[i + 1])
                    child[i + 1] = nn.Identity()
                    i += 2
                else:
                    fuse_bn_inplace(child[i])
                    i += 1
        elif hasattr(child, "conv") and hasattr(child, "bn") and isinstance(child.conv, nn.Conv2d) and isinstance(child.bn, nn.BatchNorm2d):
            child.conv = torch.nn.utils.fusion.fuse_conv_bn_eval(child.conv, child.bn)
            child.bn = nn.Identity()
            fuse_bn_inplace(child)
        else:
            fuse_bn_inplace(child)

def quantize_tensor_int8_per_channel(w: torch.Tensor) -> torch.Tensor:
    # Symmetric INT8 [-127, 127] per output channel (or per tensor for 1D)
    if w.ndim >= 2:
        max_val = w.abs().amax(dim=tuple(range(1, w.ndim)), keepdim=True).clamp(min=1e-8)
    else:
        max_val = w.abs().max().clamp(min=1e-8)
    scale = max_val / 127.0
    return torch.clamp(torch.round(w / scale), -127, 127) * scale

def quantize_tensor_int8_per_tensor(w: torch.Tensor) -> torch.Tensor:
    # Symmetric INT8 [-127, 127] per layer tensor (matches HLS per-layer ap_fixed<8, I>)
    max_val = w.abs().max().clamp(min=1e-8)
    # Power-of-two step size (exact ap_fixed<8, I> hardware shift!)
    int_bits = torch.ceil(torch.log2(max_val)).item() + 1  # +1 for sign bit
    frac_bits = 8 - int_bits
    step = 2.0 ** (-frac_bits)
    q_min = -128.0 * step
    q_max = 127.0 * step
    return torch.clamp(torch.round(w / step) * step, q_min, q_max)

def make_int8_model(model: nn.Module, mode: str = "w8a8_dynamic") -> nn.Module:
    q_mod = copy.deepcopy(model).cpu().eval()
    fuse_bn_inplace(q_mod)
    for name, module in q_mod.named_modules():
        if isinstance(module, (nn.Conv2d, nn.ConvTranspose2d, nn.Linear)):
            if mode == "hls_ap_fixed8":
                module.weight.data = quantize_tensor_int8_per_tensor(module.weight.data)
                if module.bias is not None:
                    # bias in HLS is 18-bit data_t ap_fixed<18,10> (8 frac bits = 1/256)
                    module.bias.data = torch.clamp(torch.round(module.bias.data * 256.0) / 256.0, -512.0, 511.996)
            else:
                module.weight.data = quantize_tensor_int8_per_channel(module.weight.data)
                if module.bias is not None:
                    module.bias.data = quantize_tensor_int8_per_channel(module.bias.data)
    return q_mod

def evaluate_centers_against_gt(frames_centers, gt_states, gt_xy_list, match_radius: float = 3.2):
    pred_counts = []
    dists = []
    for idx, centers in enumerate(frames_centers):
        pred_counts.append(len(centers))
        gt_xy = gt_xy_list[idx]
        st = gt_states[idx]
        if len(centers) > 0:
            c_arr = np.array([[c[0], c[1]] for c in centers], dtype=np.float32)
            for i, (gx, gy) in enumerate(gt_xy):
                d = np.linalg.norm(c_arr - np.array([gx, gy]), axis=1)
                min_i = int(np.argmin(d))
                min_d = float(d[min_i])
                if st[i] == 1 and min_d <= match_radius:
                    dists.append(min_d)
    return np.array(pred_counts), dists

def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    f_train = DATASET_F / "Train-Dataset"
    tr_b_imgs, tr_b_st, tr_b_xy, tr_b_cnt = load_f_subset(f_train / "300B", f_train / "300B" / "300B_label", "300B", range(1, 51))
    tr_p_imgs, tr_p_st, tr_p_xy, tr_p_cnt = load_f_subset(f_train / "150D_150B", f_train / "150D_150B" / "150D_150B_label", "150D_150B", range(1, 51))

    all_imgs = np.concatenate([tr_b_imgs, tr_p_imgs], axis=0)
    all_st = np.concatenate([tr_b_st, tr_p_st], axis=0)
    all_xy = tr_b_xy + tr_p_xy

    models_to_test = {
        "ViTSeg": ("vit", WEIGHTS_ROOT / "ViTSeg" / "seed_1" / "best.pth"),
        "StandardUNet": ("unet", WEIGHTS_ROOT / "StandardUNet" / "seed_1" / "best.pth"),
        "DWNetV2_scratch": ("dwunet", WEIGHTS_ROOT / "DWNetV2_scratch" / "seed_1" / "best.pth"),
        "DWNetV2_pretrained": ("dwunet", WEIGHTS_ROOT / "DWNetV2_pretrained" / "seed_1" / "best.pth"),
        "PSFANet / This_Works": ("psfanet", WEIGHTS_ROOT / "This_Works" / "seed_1" / "best.pth"),
    }

    print("=========================================================================================================")
    print("  INT8 Unified Evaluation on Train-Dataset (50 Full-Bright 300B + 50 Mixed pi/2 150D_150B)")
    print("=========================================================================================================")
    for m_name, (kind, ckpt_path) in models_to_test.items():
        if kind == "dwunet":
            model = DWNetV2_unet(None)
        elif kind == "unet":
            model = StandardUNet()
        elif kind == "vit":
            model = ViTSeg()
        elif kind == "psfanet":
            model = PSFANet(None)
        state = torch.load(ckpt_path, map_location="cpu", weights_only=True)
        if isinstance(state, dict) and "model_state_dict" in state:
            state = state["model_state_dict"]
        state = {k.removeprefix("_orig_mod."): v for k, v in state.items()}
        model.load_state_dict(state, strict=True)

        for q_label, q_mode in [("INT8 (Per-Ch Symmetric)", "w8a8_dynamic"), ("HLS ap_fixed<8,I> + Streaming CCL", "hls_ap_fixed8")]:
            if q_mode == "hls_ap_fixed8" and kind != "dwunet":
                continue
            q_model = make_int8_model(model, q_mode).to(device).eval()

            # Register activation quantization hook for W8A8 or HLS ap_fixed<18,10>
            hooks = []
            if q_mode == "w8a8_dynamic":
                def act_hook(mod, inp, out):
                    if isinstance(out, torch.Tensor):
                        mx = out.abs().amax(dim=(-2, -1), keepdim=True).clamp(min=1e-6) if out.ndim == 4 else out.abs().max().clamp(min=1e-6)
                        sc = mx / 127.0
                        return torch.clamp(torch.round(out / sc), -127, 127) * sc
                    return out
                for sub in q_model.modules():
                    if isinstance(sub, (nn.Conv2d, nn.ConvTranspose2d, nn.Linear)):
                        hooks.append(sub.register_forward_hook(act_hook))
            else:
                def hls_hook(mod, inp, out):
                    if isinstance(out, torch.Tensor):
                        # ap_fixed<18,10> (step 1/256, range [-512, 511.996])
                        return torch.clamp(torch.round(out * 256.0) / 256.0, -512.0, 511.996)
                    return out
                for sub in q_model.modules():
                    if isinstance(sub, (nn.Conv2d, nn.ConvTranspose2d, nn.ReLU6)):
                        hooks.append(sub.register_forward_hook(hls_hook))

            scores_list = []
            ccl_centers = []
            with torch.inference_mode():
                for start in range(0, len(all_imgs), 25):
                    stop = min(start + 25, len(all_imgs))
                    x = torch.from_numpy(all_imgs[start:stop, None].astype(np.float32) / 255.0).to(device)
                    x = torch.round(x * 255.0) / 255.0  # 8-bit input pixel
                    probs = torch.sigmoid(q_model(x)).squeeze(1).cpu().numpy()
                    for j in range(stop - start):
                        idx = start + j
                        scores_list.append(cross_scores_single(probs[j], all_xy[idx]))
                        b_mask = (probs[j] > 0.5).astype(np.uint8)
                        conn = 4 if q_mode == "hls_ap_fixed8" else 8
                        n_l, _, stats, cents = cv2.connectedComponentsWithStats(b_mask, connectivity=conn)
                        c_list = []
                        for k in range(1, n_l):
                            area = int(stats[k, cv2.CC_STAT_AREA])
                            if q_mode == "hls_ap_fixed8":
                                if 2 <= area <= 45:
                                    inv_a = round((1.0 / area) * 65536.0) / 65536.0
                                    x_sum = round(float(cents[k][0]) * area)
                                    y_sum = round(float(cents[k][1]) * area)
                                    cx = round(x_sum * inv_a * 256.0) / 256.0
                                    cy = round(y_sum * inv_a * 256.0) / 256.0
                                    c_list.append((cx, cy, area))
                            else:
                                c_list.append((float(cents[k][0]), float(cents[k][1]), area))
                        ccl_centers.append(c_list)
            for h in hooks:
                h.remove()

            sc_all = np.stack(scores_list)
            fpr, tpr, ths = roc_curve(all_st.ravel(), sc_all.ravel())
            opt_th = float(ths[np.argmax(tpr - fpr)])
            pred_all = (sc_all >= opt_th).astype(np.uint8)
            cnts, dists = evaluate_centers_against_gt(ccl_centers, all_st, all_xy)
            cr_b = (88 * 456 * 18) / (np.mean(cnts[:50]) * 96.0)
            cr_p = (88 * 456 * 18) / (np.mean(cnts[50:]) * 96.0)
            tag = f"{m_name} [{q_label}]"
            print(f"{tag:<52} | FullBright={np.mean(pred_all[:50]==tr_b_st)*100:.2f}% | pi/2={np.mean(pred_all[50:]==tr_p_st)*100:.2f}% | F_assign={np.mean(pred_all==all_st)*100:.2f}% | BrightCnt={np.mean(cnts[:50]):.1f}/300 ({cr_b:.1f}x) | pi/2Cnt={np.mean(cnts[50:]):.1f}/160.5 ({cr_p:.1f}x) | MAE={np.mean(dists):.3f}px")

if __name__ == "__main__":
    main()
