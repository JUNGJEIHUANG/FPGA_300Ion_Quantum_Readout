"""Validation-only threshold freeze, then paired FP32 benchmark diagnostic.

Run only after both fixed-budget training runs are complete. The benchmark is
previously exposed, not a blind test. Its labels never enter inference.
"""
import argparse, hashlib, json, sys, time
from pathlib import Path
import os
import numpy as np
import torch

ROOT = Path(__file__).resolve().parent.parent
BASE = ROOT
FROZEN = Path(os.environ.get('DWVIT_BENCHMARK_DATA', str(ROOT/'local_data/benchmark')))
sys.path.insert(0, str(ROOT / "support"))
sys.path.insert(0, str(ROOT / "source"))
from training_utils import FrozenData, packet_metrics, summarize, atomic_json, sha
from evaluation_utils import configure_torch, predictions, detailed_summary
from packet_scoring import score_centers
from T_vit_depthwise import build


def load(variant):
    run = ROOT / "runs" / variant
    status = json.loads((run / "status.json").read_text())
    assert status["status"] == "complete"
    assert sha(run / "best.pth") == status["best_sha256"]
    checkpoint = torch.load(run / "best.pth", map_location="cpu", weights_only=True)
    model = build(variant).cuda().eval()
    model.load_state_dict(checkpoint["model_state_dict"], strict=True)
    return model, checkpoint, status


def calibrate(variant, model, checkpoint, data):
    dest = ROOT / "reports" / f"T_{variant}_validation_thresholds.json"
    if dest.exists():
        receipt = json.loads(dest.read_text())
        assert receipt["checkpoint_sha256"] == sha(ROOT / "runs" / variant / "best.pth")
        return receipt["selected_threshold"]
    thresholds = [round(.35+.05*i,2) for i in range(13)]
    rows = {t: [] for t in thresholds}
    with torch.inference_mode():
        for off in range(0,len(data.val),16):
            ids=data.val[off:off+16]
            x,*_=data.batch(ids)
            prob=model(x).sigmoid().squeeze(1).cpu().numpy()
            assert np.isfinite(prob).all()
            for idx,pp in zip(ids,prob):
                for t in thresholds:
                    metric=packet_metrics(pp>t,data.coords[idx],data.states[idx])
                    metric.update(cache_row=int(idx),category=str(data.categories[idx]))
                    rows[t].append(metric)
            if off%1024==0:print("CALIBRATE",variant,off,flush=True)
    curve={str(t):summarize(rows[t]) for t in thresholds}
    assert curve["0.5"] == checkpoint["validation"]["groups"], "Checkpoint validation mismatch"
    def key(t):
        g=curve[str(t)];m=g["150D_150B"];a=g["ALL"]
        return (m["packet_errors"],-m["state_exact"],m["fp"],a["packet_errors"],abs(t-.5),t)
    selected=min(thresholds,key=key)
    atomic_json(dict(status="frozen",variant=variant,checkpoint_sha256=sha(ROOT/"runs"/variant/"best.pth"),
                     selected_threshold=selected,selection_key=list(key(selected)),
                     selected_validation=curve[str(selected)],threshold_curve=curve,
                     frozen_manifest=data.receipt["manifest_sha256"],test_accessed=False),dest)
    print("THRESHOLD_FROZEN",variant,selected,flush=True)
    return selected


def test(variant, model, threshold):
    source=json.loads((FROZEN/"reports"/"T_inputs_frozen.json").read_text())
    assert source["status"]=="frozen" and source["frames"]==2000
    for name, expected in source["cache_sha256"].items():
        assert sha(FROZEN/"cache"/name)==expected, name
    images=np.load(FROZEN/"cache"/"T_images_u8.npy",mmap_mode="r")
    xy=np.load(FROZEN/"cache"/"T_truth_xy.npy",mmap_mode="r")
    truth=np.load(FROZEN/"cache"/"T_truth_states.npy",mmap_mode="r")
    isotope=np.load(FROZEN/"cache"/"T_isotope.npy",mmap_mode="r")
    out=ROOT/"reports"/f"T_{variant}_fp32_test.json"
    if out.exists():raise RuntimeError(f"Refuse overwrite: {out}")
    rows=[];allpred=[];t0=time.perf_counter()
    with torch.inference_mode():
        for off in range(0,2000,16):
            x=torch.from_numpy(np.array(images[off:off+16])).unsqueeze(1).float().cuda()/255.
            prob=model(x).sigmoid().squeeze(1).cpu().numpy()
            assert np.isfinite(prob).all()
            for j,pp in enumerate(prob):
                i=off+j
                _,centers,areas=predictions(pp>threshold,xy[i])
                metric,pred=score_centers(centers,xy[i],truth[i],isotope[i])
                metric.update(id=i,category=source["images"][i]["group"],relative=source["images"][i]["relative"])
                rows.append(metric);allpred.append(pred)
            if off%160==0:print("TEST",variant,off+len(prob),flush=True)
    pred=np.stack(allpred)
    np.save(ROOT/"reports"/f"T_{variant}_test_predictions.npy",pred)
    atomic_json(rows,ROOT/"reports"/f"T_{variant}_test_per_frame.json")
    result=dict(status="complete",variant=variant,checkpoint_sha256=sha(ROOT/"runs"/variant/"best.pth"),
                threshold=threshold,frames=2000,groups=detailed_summary(rows),
                prediction_sha256=sha(ROOT/"reports"/f"T_{variant}_test_predictions.npy"),
                seconds=time.perf_counter()-t0,
                caveat="Previously exposed benchmark; descriptive paired diagnostic, not fresh blind test",
                ground_truth_coordinates_in_inference=False)
    atomic_json(result,out)
    print("TEST_COMPLETE",variant,json.dumps({"ALL":result["groups"]["ALL"]}),flush=True)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--stage',choices=['validation','test'],required=True)
    args=parser.parse_args()
    configure_torch()
    (ROOT/"reports").mkdir(exist_ok=True)
    data=FrozenData(torch.device("cuda:0"))
    for variant in ("dw","dw_dilated"):
        model,checkpoint,_=load(variant)
        threshold=calibrate(variant,model,checkpoint,data)
        if args.stage=='test':
            if not (ROOT/'reports/T_board_candidate_pretest.json').exists():
                raise RuntimeError('Board candidate must be frozen from validation before test exposure')
            test(variant,model,threshold)
        del model;torch.cuda.empty_cache()
    if args.stage=='validation':
        print('VALIDATION_THRESHOLDS_COMPLETE_TEST_NOT_ACCESSED',flush=True)
        return
    baseline=json.loads((FROZEN/"reports"/"T_fp32"/"T_summary.json").read_text())
    results={v:json.loads((ROOT/"reports"/f"T_{v}_fp32_test.json").read_text()) for v in ("dw","dw_dilated")}
    summary={"baseline":baseline["groups"],"variants":{v:r["groups"] for v,r in results.items()},
             "base_params":392065,"variant_params":376321,
             "base_macs":1049587968,"variant_macs":415244544,
             "mac_caveat":"Architectural dense MAC count only; not FPGA cycles or latency",
             "test_scope":"Previously exposed 2000-image benchmark; threshold and checkpoint frozen using train/validation only"}
    atomic_json(summary,ROOT/"reports"/"T_fp32_comparison.json")
    print("FP32_COMPARISON_COMPLETE",flush=True)

if __name__=="__main__":main()
