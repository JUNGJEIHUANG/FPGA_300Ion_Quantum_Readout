# DW-ViT source and reproduction prerequisites

## Portable model smoke test

From the repository root:

```powershell
python dw_vit/tests/test_model.py
```

Requires PyTorch, NumPy, SciPy and OpenCV. Model keys match the experiment checkpoint, including `decoder.2.0.*` (depthwise) and `decoder.2.1.*` (pointwise). The selected dilation is 2; dilation 1 is a controlled local variant, not another published baseline. A source-relocation test against the original checkpoint is recorded in the release results; it is not a retraining experiment.

## Train / validate

Local inputs are intentionally not uploaded. `DWVIT_TRAIN_DATA` must point to a directory with:

- `cache/images.npy`: uint8 `(25000,88,456)`;
- `cache/xy.npy`: `(25000,300,2)` annotated coordinates;
- `cache/states.npy`: `(25000,300)` states;
- `cache/training_indices.npy`, `cache/validation_indices.npy`: frozen 20000/5000 memberships;
- `cache/split_manifest.csv`: includes category names;
- `reports/training_data_receipt.json`: includes the manifest SHA-256.

The training script checks the original frozen manifest hash. A different split requires a separately declared experiment and configuration change; do not silently reuse the published result names. GPU-cached training needs sufficient CUDA memory. No AMP/TF32, no pretrained initialization, and the recorded 20 epochs are a fixed budget, not claimed convergence.

```powershell
$env:DWVIT_TRAIN_DATA = 'D:\local-data\train-validation'
python dw_vit/scripts/T_train_variants.py --variant dw
python dw_vit/scripts/T_train_variants.py --variant dw_dilated
python dw_vit/scripts/T_evaluate_variants.py --help
```

Read script CLI flags before evaluation. Freeze validation-selected thresholds and candidate before allowing benchmark access. `DWVIT_BENCHMARK_DATA` points to the separately sealed benchmark fixture (`cache/T_images_u8.npy`, `T_truth_xy.npy`, `T_truth_states.npy`, `T_isotope.npy`, and `reports/T_inputs_frozen.json`). Ground-truth arrays enter offline scoring only. Aggregate archived results are under `../results/dw_vit/`; writable re-runs use this folder's `runs/` and `reports/`, so results are not silently overwritten.

## Numerical validation and HLS

`source/T_quant_vit_depthwise.py` simulates W8/A18 boundary storage but keeps FP32 normalization/nonlinear internals. It is not a bit-accurate hardware simulator. The complete HLS C source lives in `01_hls/src_dw_dilated/` and `01_hls/src_dw_dilated_fast/`; `src_dw/` preserves the non-dilated control. Only C++ source/headers and testbench code are included. Learned `vit_parameters.h`, weight bank binaries and training checkpoints are excluded.

Export requires locally supplied/retrained `runs/<variant>/best.pth`, matching run status, and frozen validation thresholds. `T_export_hls_parameters.py` generates the serial parameter header and four bank files. `T_prepare_fast_depthwise.py` generates a fast source/header view from the serial export and deliberately refuses an already-existing destination: use a fresh generation workspace rather than overwriting the shipped reference source. `parameters/gelu_q10_8_positive_2048.hex` is a deterministic GELU LUT, not learned weights.

`01_hls/T_run_variant.tcl` and `hardware/workspace_scripts/` are **as-executed workspace tooling**, not portable turnkey commands. They retain local paths and rely on proprietary Xilinx tools, locally generated DCPs/IP, frozen validation inputs, USB transport helpers, and a compatible camera project that are not included. Adjust those paths in a separate workspace. Some archived scripts record programming flows, but the release's copied build/replay source does not itself run them.

The camera integration preserves sensor/DDR logic but uses **preloaded dataset replay**. It is not a sensor-synchronous pipeline demonstration or direct 300-bit register interface. The serial/fast five-frame record match is separate from HLS-C equivalence.

## Environment

Recorded runs used Python/PyTorch 2.11, CUDA-enabled training, Vitis HLS 2022.2 and Vivado 2022.2. See the per-run JSON files for exact configuration and checkpoint hashes. Do not substitute different checkpoints/thresholds for the archived measurements.

## Audit the completed board run (offline only)

`tests/audit_board_results.py` independently checks a complete run and recomputes site/record metrics. It does not open a USB/JTAG connection or perform inference, training, or model selection. Supply the locally held raw packet files and frozen fixture; neither raw packets nor truth arrays are included in the public release.

```powershell
python dw_vit/tests/audit_board_results.py --run-dir <complete-run-folder> --fixture-dir <frozen-benchmark-folder> --checkpoint <selected-best.pth> --bitstream <attested.bit> --attestation <attestation.json> --output <new-audit.json> --output-hashes <new-hashes.json>
```

The auditor refuses existing output files and requires `complete`, exactly 2,000 requested/completed unique frames, matching checkpoint/bitstream hashes, intact input signatures and packet hashes, valid raw centroid records, and agreement between recomputed and recorded scores. Published outputs contain aggregate metrics and output hashes, not image pixels, coordinates, truth labels, or per-frame state predictions. An aggregate FPGA-versus-FP32 comparison is in the repository README; these are distinct numerical executions.
