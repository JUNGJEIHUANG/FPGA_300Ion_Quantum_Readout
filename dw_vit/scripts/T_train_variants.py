"""Train two decoder-only ViTSeg variants on the frozen train/val split.

Run: python -u T_train_variants.py --variant dw [--resume]
This script never reads the held-out 2000-image benchmark.
"""
import argparse, csv, hashlib, json, math, os, random, sys, time
from pathlib import Path
import numpy as np
import torch

HERE = Path(__file__).resolve().parent.parent
BASE = HERE
sys.path.insert(0, str(HERE / "support"))
sys.path.insert(0, str(HERE / "source"))
from training_utils import sha, selection_key, validate, atomic_save, atomic_json
from cache_acceleration import CachedData, FAST_OBJECTIVE
import training_utils
from T_vit_depthwise import build


def seed_all(seed):
    torch.manual_seed(seed); torch.cuda.manual_seed_all(seed)
    np.random.seed(seed); random.seed(seed)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--variant", choices=["dw", "dw_dilated"], required=True)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    out = HERE / "runs" / args.variant
    out.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(8)
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA GPU unavailable; no silent CPU fallback")
    device = torch.device("cuda:0")
    data = CachedData(device)
    assert data.receipt["manifest_sha256"] == "8596cbc0a9671282b2470a544a5ed266cccdc271499ae5ceaf2076d4ffda2613"
    training_utils.objective = FAST_OBJECTIVE
    seed_all(1)
    model = build(args.variant).to(device)
    parameters = sum(p.numel() for p in model.parameters())
    assert parameters == 376321, parameters
    with torch.no_grad():
        assert tuple(model(torch.zeros((1, 1, 88, 456), device=device)).shape) == (1, 1, 88, 456)
    optimizer = torch.optim.AdamW(model.parameters(), lr=.001, weight_decay=.0001)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=20, eta_min=1e-5)
    protocol = dict(variant=args.variant, seed=1, epochs=20, micro_batch=16,
                    effective_batch=32, accumulation=2, optimizer="AdamW",
                    lr=.001, weight_decay=.0001,
                    scheduler="CosineAnnealingLR(T_max=20,eta_min=1e-5)",
                    fp32_no_amp_tf32=True, loss="unchanged frozen baseline loss",
                    selection="unchanged baseline validation key",
                    threshold=.5, source_train=sha(Path(__file__)),
                    source_model=sha(HERE / "source" / "T_vit_depthwise.py"),
                    source_original=sha(HERE.parent/'nets/DWViT.py'),
                    frozen_manifest=data.receipt["manifest_sha256"],
                    test_accessed=False,
                    comparability="Scratch seed1 fixed20 epochs, same baseline objective and validation selection; no convergence claim")
    config = dict(parameters=parameters, protocol=protocol)
    config_path = out / "run_config.json"
    if config_path.exists():
        assert json.loads(config_path.read_text()) == config, "Protocol changed"
    else:
        atomic_json(config, config_path)
    last = out / "last_training.pth"
    if args.resume and last.exists():
        state = torch.load(last, map_location="cpu", weights_only=False)
        assert state["config"] == config
        model.load_state_dict(state["model_state_dict"])
        optimizer.load_state_dict(state["optimizer_state_dict"])
        scheduler.load_state_dict(state["scheduler_state_dict"])
        history = state["history"]; best = tuple(state["best_key"])
        best_epoch = state["best_epoch"]; start_epoch = state["epoch"] + 1
        torch.set_rng_state(state["rng_torch"]); torch.cuda.set_rng_state_all(state["rng_cuda"])
        random.setstate(state["rng_python"])
        nr = state["rng_numpy"]
        np.random.set_state((nr[0], np.asarray(nr[1], np.uint32), nr[2], nr[3], nr[4]))
    else:
        if last.exists(): raise RuntimeError("Existing state; pass --resume")
        history=[]; best=(math.inf,); best_epoch=0; start_epoch=1
    print("TRAIN_START", args.variant, "parameters", parameters, "epoch", start_epoch, flush=True)
    for epoch in range(start_epoch, 21):
        model.train(); t0=time.perf_counter(); order=np.random.permutation(data.train)
        loss_sum=0.
        for off in range(0, len(order), 32):
            optimizer.zero_grad(set_to_none=True); group=order[off:off+32]
            all_weight=float(np.where(data.categories[group]=='150D_150B',1.5,1.).sum())
            for micro in range(0,len(group),16):
                ids=group[micro:micro+16]
                x,y,states,cross,peak,mixed=data.batch(ids)
                logits=model(x)
                loss,_=FAST_OBJECTIVE(logits,y,states,cross,peak,mixed)
                if not torch.isfinite(loss):raise RuntimeError(f"nonfinite loss at {epoch}/{off}")
                (loss*(torch.where(mixed,1.5,1.).sum()/all_weight)).backward()
                loss_sum+=float(loss.detach())*len(ids)
            torch.nn.utils.clip_grad_norm_(model.parameters(),1.)
            optimizer.step()
            if off==0 or (off//32+1)%250==0:
                print("STEP",args.variant,epoch,min(off+32,len(order)),flush=True)
        val=validate(model,data,batch_size=16,save_path=out/f"validation_epoch{epoch:02d}.json")
        key=selection_key(val)
        if key < best:
            best=key; best_epoch=epoch
            atomic_save(dict(variant=args.variant,epoch=epoch,model_state_dict={k:v.detach().cpu().clone() for k,v in model.state_dict().items()},validation=val,config=config),out/'best.pth')
            atomic_json(val,out/'best_validation_summary.json')
        scheduler.step()
        mixed=val['groups']['150D_150B']; all_=val['groups']['ALL']
        row=dict(epoch=epoch,train_objective=loss_sum/len(data.train),val_objective=val['objective'],
                 mixed_packet_errors=mixed['packet_errors'],mixed_site_accuracy=mixed['site_accuracy'],
                 mixed_exact=mixed['state_exact_rate'],mixed_false_bright=mixed['false_bright_rate'],
                 mixed_false_dark=mixed['false_dark_rate'],all_site_accuracy=all_['site_accuracy'],
                 all_exact=all_['state_exact_rate'],best_epoch=best_epoch,
                 lr=optimizer.param_groups[0]['lr'],epoch_seconds=time.perf_counter()-t0)
        history.append(row)
        with (out/'history.csv').open('w',newline='',encoding='utf-8') as f:
            w=csv.DictWriter(f,fieldnames=list(row));w.writeheader();w.writerows(history)
        nr=np.random.get_state(); rng_numpy=(nr[0],nr[1].tolist(),nr[2],nr[3],nr[4])
        atomic_save(dict(config=config,epoch=epoch,model_state_dict={k:v.detach().cpu().clone() for k,v in model.state_dict().items()},
                         optimizer_state_dict=optimizer.state_dict(),scheduler_state_dict=scheduler.state_dict(),
                         history=history,best_key=list(best),best_epoch=best_epoch,
                         rng_torch=torch.get_rng_state(),rng_cuda=torch.cuda.get_rng_state_all(),
                         rng_python=random.getstate(),rng_numpy=rng_numpy),last)
        atomic_json(dict(status="training",last_epoch=epoch,best_epoch=best_epoch,last=row,test_accessed=False),out/'status.json')
        print("EPOCH_RESULT",args.variant,json.dumps(row),flush=True)
    atomic_json(dict(status="complete",epochs=20,best_epoch=best_epoch,parameters=parameters,
                     best_sha256=sha(out/'best.pth'),test_accessed=False,converged=False),out/'status.json')
    print("TRAIN_COMPLETE",args.variant,"best_epoch",best_epoch,flush=True)

if __name__ == '__main__': main()
