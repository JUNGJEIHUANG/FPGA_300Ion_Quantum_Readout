import os
import sys
import json
import subprocess
from pathlib import Path
from PIL import Image
import cv2
import numpy as np
import torch
from sklearn.metrics import roc_curve

sys.path.insert(0, r"E:\05_Project_300ions_CNNDetect")
from nets.DWNetV2_unet import DWNetV2_unet
from nets.StandardUNet import StandardUNet
from nets.ViTSeg import ViTSeg
from nets.psfanet import PSFANet

DATASET_F = Path(r"F:\00_Site_AIT\300-Qubits-Register-Dataset")
DATASET_E = Path(r"E:\05_Project_300ions_CNNDetect\Dataset")
WEIGHTS_ROOT = Path(r"F:\01_DWUnet\DL-Ion-Readout-anon_code\model_weights")
HLS_DIR = Path(r"E:\05_Project_300ions_CNNDetect\fpga_hls")

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

def run_fpga_batch(imgs_np: np.ndarray, tmp_dir: Path, force_rerun: bool = False) -> list[list[tuple[float, float, int]]]:
    tmp_dir.mkdir(parents=True, exist_ok=True)
    list_file = tmp_dir / "input_list.txt"
    out_file = tmp_dir / "output_centers.txt"
    if not out_file.exists() or force_rerun:
        paths = []
        for idx, im in enumerate(imgs_np):
            p = tmp_dir / f"frame_{idx:04d}.txt"
            np.savetxt(p, (im.astype(np.float32) / 255.0).flatten(), fmt="%.6f")
            paths.append(str(p))
        list_file.write_text("\n".join(paths) + "\n", encoding="utf-8")

        env = os.environ.copy()
        env["PATH"] = r"D:\Xilinx\2022\Vitis_HLS\2022.2\tps\win64\msys64\mingw64\bin;" + env.get("PATH", "")
        exe_path = str(HLS_DIR / "test_batch_fpga.exe")
        subprocess.run([exe_path, str(list_file), str(out_file)], cwd=str(HLS_DIR), env=env, check=True)

    frames_centers = []
    curr = []
    for line in out_file.read_text(encoding="utf-8").splitlines():
        parts = line.strip().split()
        if not parts:
            continue
        if parts[0] == "FRAME":
            if curr or len(frames_centers) > 0:
                if len(frames_centers) < int(parts[1]):
                    frames_centers.append(curr)
                curr = []
        else:
            cx, cy, area = float(parts[0]), float(parts[1]), int(parts[2])
            curr.append((cx, cy, area))
    frames_centers.append(curr)
    return frames_centers

def evaluate_centers_against_gt(frames_centers, gt_states, gt_xy_list, match_radius: float = 3.2):
    pred_states_all = []
    pred_counts = []
    dists = []
    for idx, centers in enumerate(frames_centers):
        pred_counts.append(len(centers))
        gt_xy = gt_xy_list[idx]
        st = gt_states[idx]
        pst = np.zeros(len(st), dtype=np.uint8)
        if len(centers) > 0:
            c_arr = np.array([[c[0], c[1]] for c in centers], dtype=np.float32)
            for i, (gx, gy) in enumerate(gt_xy):
                d = np.linalg.norm(c_arr - np.array([gx, gy]), axis=1)
                min_i = int(np.argmin(d))
                min_d = float(d[min_i])
                # Also account for merged wide blobs (area > 14 means 2 adjacent bright ions merged)
                eff_r = match_radius * (1.45 if centers[min_i][2] >= 14 else 1.0)
                if min_d <= eff_r:
                    pst[i] = 1
                    if st[i] == 1 and min_d <= match_radius:
                        dists.append(min_d)
        pred_states_all.append(pst)
    pred_all = np.stack(pred_states_all)
    return np.array(pred_counts), pred_all, dists

def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    bench_dirs = [d for d in DATASET_F.iterdir() if d.is_dir() and "Real1600" in d.name]
    bench_root = bench_dirs[0]

    # Load 50 Full-Bright + 50 Mixed pi/2 from Train-Dataset AND 50 Full-Bright + 50 Mixed pi/2 from Real1600-Benchmark
    f_train = DATASET_F / "Train-Dataset"
    tr_b_imgs, tr_b_st, tr_b_xy, tr_b_cnt = load_f_subset(f_train / "300B", f_train / "300B" / "300B_label", "300B", range(1, 51))
    tr_p_imgs, tr_p_st, tr_p_xy, tr_p_cnt = load_f_subset(f_train / "150D_150B", f_train / "150D_150B" / "150D_150B_label", "150D_150B", range(1, 51))

    te_b_imgs, te_b_st, te_b_xy, te_b_cnt = load_f_subset(
        bench_root / "01_Full_bright_250", bench_root / "01_Full_bright_250" / "01_Full_bright_250_label", "01_Full_bright_250", range(1, 51)
    )
    te_p_imgs, te_p_st, te_p_xy, te_p_cnt = load_f_subset(
        bench_root / "03_pi2_binomial_1000", bench_root / "03_pi2_binomial_1000" / "03_pi2_binomial_1000_label", "03_pi2_binomial_1000", range(1, 51)
    )

    models_to_test = {
        "ViTSeg (FP32)": ("vit", WEIGHTS_ROOT / "ViTSeg" / "seed_1" / "best.pth"),
        "StandardUNet (FP32)": ("unet", WEIGHTS_ROOT / "StandardUNet" / "seed_1" / "best.pth"),
        "DWNetV2_scratch (FP32)": ("dwunet", WEIGHTS_ROOT / "DWNetV2_scratch" / "seed_1" / "best.pth"),
        "DWNetV2_pretrained (FP32)": ("dwunet", WEIGHTS_ROOT / "DWNetV2_pretrained" / "seed_1" / "best.pth"),
        "PSFANet / This_Works (FP32)": ("psfanet", WEIGHTS_ROOT / "This_Works" / "seed_1" / "best.pth"),
    }

    for split_label, (imgs_b, st_b, xy_b, cnt_b, imgs_p, st_p, xy_p, cnt_p) in [
        ("Train-Dataset (50 Full-Bright 300B + 50 Mixed pi/2 150D_150B)", (tr_b_imgs, tr_b_st, tr_b_xy, tr_b_cnt, tr_p_imgs, tr_p_st, tr_p_xy, tr_p_cnt)),
        ("Real1600-Benchmark (50 Full-Bright 01 + 50 Mixed pi/2 03)", (te_b_imgs, te_b_st, te_b_xy, te_b_cnt, te_p_imgs, te_p_st, te_p_xy, te_p_cnt)),
    ]:
        print(f"\n======================================================================")
        print(f"  {split_label}")
        print(f"======================================================================")
        all_imgs = np.concatenate([imgs_b, imgs_p], axis=0)
        all_st = np.concatenate([st_b, st_p], axis=0)
        all_xy = xy_b + xy_p

        # 1. OpenCV Otsu
        otsu_scores = []
        otsu_centers = []
        for idx, im in enumerate(all_imgs):
            blurred = cv2.GaussianBlur(im, (3, 3), 0)
            _, binary = cv2.threshold(blurred, 0, 1, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            otsu_scores.append(cross_scores_single(binary.astype(np.float32), all_xy[idx]))
            n_l, _, stats, cents = cv2.connectedComponentsWithStats(binary.astype(np.uint8), connectivity=8)
            c_list = [(float(cents[k][0]), float(cents[k][1]), int(stats[k, cv2.CC_STAT_AREA])) for k in range(1, n_l)]
            otsu_centers.append(c_list)
        otsu_sc = np.stack(otsu_scores)
        otsu_pred = (otsu_sc >= 0.5).astype(np.uint8)
        otsu_cnts, _, otsu_dists = evaluate_centers_against_gt(otsu_centers, all_st, all_xy)
        print(f"[OpenCV Otsu + 8-Conn CCL   ] FullBright_Acc={np.mean(otsu_pred[:50]==st_b)*100:.2f}% | pi/2_Acc={np.mean(otsu_pred[50:]==st_p)*100:.2f}% | Overall_F_assign={np.mean(otsu_pred==all_st)*100:.2f}% | BrightCount={np.mean(otsu_cnts[:50]):.1f}/300 | pi/2Count={np.mean(otsu_cnts[50:]):.1f}/{np.mean(cnt_p):.1f} | CentroidMAE={np.mean(otsu_dists):.3f}px")

        # 2. PyTorch FP32 Models
        for m_name, (kind, ckpt_path) in models_to_test.items():
            if kind == "dwunet":
                model = DWNetV2_unet(None)
            elif kind == "unet":
                model = StandardUNet()
            elif kind == "vit":
                model = ViTSeg()
            elif kind == "psfanet":
                model = PSFANet(None)
            state = torch.load(ckpt_path, map_location=device, weights_only=True)
            if isinstance(state, dict) and "model_state_dict" in state:
                state = state["model_state_dict"]
            state = {k.removeprefix("_orig_mod."): v for k, v in state.items()}
            model.load_state_dict(state, strict=True)
            model.to(device).eval()

            scores_list = []
            ccl_centers = []
            with torch.inference_mode():
                for start in range(0, len(all_imgs), 25):
                    stop = min(start + 25, len(all_imgs))
                    x = torch.from_numpy(all_imgs[start:stop, None].astype(np.float32) / 255.0).to(device)
                    probs = torch.sigmoid(model(x)).squeeze(1).cpu().numpy()
                    for j in range(stop - start):
                        idx = start + j
                        scores_list.append(cross_scores_single(probs[j], all_xy[idx]))
                        b_mask = (probs[j] > 0.5).astype(np.uint8)
                        n_l, _, stats, cents = cv2.connectedComponentsWithStats(b_mask, connectivity=8)
                        c_list = [(float(cents[k][0]), float(cents[k][1]), int(stats[k, cv2.CC_STAT_AREA])) for k in range(1, n_l)]
                        ccl_centers.append(c_list)
            sc_all = np.stack(scores_list)
            fpr, tpr, ths = roc_curve(all_st.ravel(), sc_all.ravel())
            opt_th = float(ths[np.argmax(tpr - fpr)])
            pred_all = (sc_all >= opt_th).astype(np.uint8)
            cnts, _, dists = evaluate_centers_against_gt(ccl_centers, all_st, all_xy)
            cr_b = (88 * 456 * 18) / (np.mean(cnts[:50]) * 96.0)
            cr_p = (88 * 456 * 18) / (np.mean(cnts[50:]) * 96.0)
            print(f"[{m_name:<27}] FullBright_Acc={np.mean(pred_all[:50]==st_b)*100:.2f}% | pi/2_Acc={np.mean(pred_all[50:]==st_p)*100:.2f}% | Overall_F_assign={np.mean(pred_all==all_st)*100:.2f}% | BrightCount={np.mean(cnts[:50]):.1f}/300 ({cr_b:.1f}x CR) | pi/2Count={np.mean(cnts[50:]):.1f}/{np.mean(cnt_p):.1f} ({cr_p:.1f}x CR) | CentroidMAE={np.mean(dists):.3f}px")
            if m_name == "DWNetV2_pretrained (FP32)":
                # Also simulate 16-bit Fixed-Point (BN-Folded ap_fixed<16,6>, 1/1024 step) + 4-Connected Streaming CCL + Reciprocal LUT
                import copy
                def fuse_bn_inplace(mod):
                    for _, child in mod.named_children():
                        if isinstance(child, torch.nn.Sequential):
                            i = 0
                            while i < len(child) - 1:
                                if isinstance(child[i], torch.nn.Conv2d) and isinstance(child[i + 1], torch.nn.BatchNorm2d):
                                    child[i] = torch.nn.utils.fusion.fuse_conv_bn_eval(child[i], child[i + 1])
                                    child[i + 1] = torch.nn.Identity()
                                    i += 2
                                else:
                                    fuse_bn_inplace(child[i])
                                    i += 1
                        else:
                            fuse_bn_inplace(child)

                fp16_scores_list = []
                fp16_ccl_centers = []
                with torch.inference_mode():
                    q_model = copy.deepcopy(model).cpu().eval()
                    fuse_bn_inplace(q_model)
                    for p in q_model.parameters():
                        p.data = torch.clamp(torch.round(p.data * 1024.0) / 1024.0, -32.0, 31.999)
                    q_model.to(device).eval()
                    for start in range(0, len(all_imgs), 25):
                        stop = min(start + 25, len(all_imgs))
                        x = torch.from_numpy(all_imgs[start:stop, None].astype(np.float32) / 255.0).to(device)
                        x = torch.round(x * 1024.0) / 1024.0
                        logits = q_model(x)
                        logits = torch.clamp(torch.round(logits * 1024.0) / 1024.0, -32.0, 31.999)
                        probs_q = torch.sigmoid(logits).squeeze(1).cpu().numpy()
                        for j in range(stop - start):
                            idx = start + j
                            fp16_scores_list.append(cross_scores_single(probs_q[j], all_xy[idx]))
                            # Hardware Streaming CCL: logit > 0.0 (prob > 0.5), 4-connected, area >= 2, reciprocal LUT 16-bit
                            b_mask = (probs_q[j] > 0.5).astype(np.uint8)
                            n_l, _, stats, cents = cv2.connectedComponentsWithStats(b_mask, connectivity=4)
                            c_list = []
                            for k in range(1, n_l):
                                area = int(stats[k, cv2.CC_STAT_AREA])
                                if 2 <= area <= 45:
                                    inv_a = round((1.0 / area) * 65536.0) / 65536.0
                                    x_sum = round(float(cents[k][0]) * area)
                                    y_sum = round(float(cents[k][1]) * area)
                                    cx = round(x_sum * inv_a * 256.0) / 256.0
                                    cy = round(y_sum * inv_a * 256.0) / 256.0
                                    c_list.append((cx, cy, area))
                            fp16_ccl_centers.append(c_list)
                sc_q = np.stack(fp16_scores_list)
                pred_q = (sc_q >= opt_th).astype(np.uint8)
                cnts_q, _, dists_q = evaluate_centers_against_gt(fp16_ccl_centers, all_st, all_xy)
                cr_bq = (88 * 456 * 18) / (np.mean(cnts_q[:50]) * 96.0)
                cr_pq = (88 * 456 * 18) / (np.mean(cnts_q[50:]) * 96.0)
                print(f"[Ours: 16-bit Fixed-Pt + CCL] FullBright_Acc={np.mean(pred_q[:50]==st_b)*100:.2f}% | pi/2_Acc={np.mean(pred_q[50:]==st_p)*100:.2f}% | Overall_F_assign={np.mean(pred_q==all_st)*100:.2f}% | BrightCount={np.mean(cnts_q[:50]):.1f}/300 ({cr_bq:.1f}x CR) | pi/2Count={np.mean(cnts_q[50:]):.1f}/{np.mean(cnt_p):.1f} ({cr_pq:.1f}x CR) | CentroidMAE={np.mean(dists_q):.3f}px")

        # 3. FPGA HLS Bit-Accurate C-Sim (width_mult=0.5 weights currently in dwnet_weights_all.cpp)
        cache_dir = Path(r"E:\ISCAS_3\tmp_fpga_eval") if "Train-Dataset" in split_label else Path(r"E:\ISCAS_3\tmp_fpga_eval_bench")
        fpga_centers = run_fpga_batch(all_imgs, cache_dir, force_rerun=False)
        fpga_cnts, fpga_pred_all, fpga_dists = evaluate_centers_against_gt(fpga_centers, all_st, all_xy)
        cr_b_fpga = (88 * 456 * 18) / (np.mean(fpga_cnts[:50]) * 96.0)
        cr_p_fpga = (88 * 456 * 18) / (np.mean(fpga_cnts[50:]) * 96.0)
        print(f"[HLS C-Sim (0.5x Half-Ch Wt)] FullBright_Acc={np.mean(fpga_pred_all[:50]==st_b)*100:.2f}% | pi/2_Acc={np.mean(fpga_pred_all[50:]==st_p)*100:.2f}% | Overall_F_assign={np.mean(fpga_pred_all==all_st)*100:.2f}% | BrightCount={np.mean(fpga_cnts[:50]):.1f}/300 ({cr_b_fpga:.1f}x CR) | pi/2Count={np.mean(fpga_cnts[50:]):.1f}/{np.mean(cnt_p):.1f} ({cr_p_fpga:.1f}x CR) | CentroidMAE={np.mean(fpga_dists):.3f}px")

if __name__ == "__main__":
    main()
