# M11 baseline protocol

Windows MSVC Release, 2026-09-27. Synthetic curved registration and fixed-spacing flat full inspection. The full inspection includes normalization, preprocessing, registration, gates, transform, comparison and detection; file I/O/serialization are excluded. Both surfaces have supplied analytic normals. Registration uses a known (0.003,-0.002,0.01) mm translation and verifies convergence/RMSE <=0.001 mm and exact fresh/reused pose and RMSE. Full inspection must PASS without regions.

Reproduce after `pwsh ./scripts/build.ps1 -Preset windows-msvc-release` in the initialized developer shell:

```
cmake --build --preset windows-msvc-release --target pointcloud_ad_performance_benchmark
out/build/windows-msvc-release/bin/pointcloud_ad_performance_benchmark.exe 65 7
out/build/windows-msvc-release/bin/pointcloud_ad_performance_benchmark.exe 317 3
```

One warm-up sample is excluded. CSV reports nearest-rank P50/P95; three samples do not establish a reliable tail estimate. Each context processes three identical scans to isolate reuse costs; changing scans remain covered by HET-003 and context tests. Generation and correctness checks are outside timing. One-shot includes temporary context destruction; prepare/first/reused are separate timings. Inspection total includes report creation; stage_2 normalize, 3 preprocess, 4 registration, 5 registration gate, 6 transform, 7 comparison/statistics, 8 coverage gate, 9 detection. Timestamp/timing fields are not deterministic result data.

Raw data: PCAD-PERF-001-baseline.csv (4,225 points), PCAD-PERF-001-100k.csv (100,489 points). Official measurements run without concurrent builds/analysis. Side 1000 enables one million points; this baseline does not claim a million-point full inspection measurement, memory/transfer profiling, industrial accuracy or multi-device performance. CUDA search transfer/memory baselines remain in HET-002; broader workloads and real samples belong to subsequent performance acceptance.

Host: Intel Core i5-13400F (10 cores / 16 logical processors), NVIDIA RTX 5070 Ti (16 GiB), driver 616.64. Baseline uses CPU only. Compiler MSVC 19.51; GPU regression toolkit CUDA 13.2.78.
