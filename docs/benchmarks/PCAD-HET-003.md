# Reusable reference registration context, 2026-09-13

Windows MSVC 19.51 Release, Intel Core i5-13400F, NVIDIA RTX 5070 Ti, CUDA 13.2.78,
driver 616.64, native GPU architecture. One warm-up sequence and median of three measured
sequences on one machine. This is synthetic registration, not full inspection or industrial accuracy.

## Protocol

Each sequence contains five different scans against one reference. An anisotropic paraboloid spans
[-4,4] mm in x/y, z=0.08*x*x+0.13*y*y, with analytic normals and 65/129/257 points per side.
Scan-to-reference translation is (0.003+0.0001*frame, -0.002, 0.01+0.0002*frame) mm for frame 0..4.
Initial pose is identity. Parameters: 40 iterations, 0.2 mm correspondence bound, 0.1 mm Huber
threshold, 1e-7 mm translation, 1e-8 rad rotation and 1e-9 mm residual convergence thresholds;
default GICP neighborhoods/regularization. This fixture differs from HET-002; do not interpret
cross-document times as a controlled regression comparison.

Within each sequence, five one-shot calls are timed individually and summed. Then one context is
prepared and its five align calls timed individually. Context destruction is timed separately.
Preparation includes reference copy/validation, CPU tree or CUDA index construction and upload,
GPU capacity allocation, and reference covariance where needed. Align includes scan validation,
extraction/covariance, all iterations/transfers/solving and final evaluation. Total reused time is
prepare + five align calls + destruction. Input creation, driver initialization, correctness checks,
and benchmark result-retention overhead are excluded. Later-frame time is the mean of frames 1..4
within a sequence; the table reports its median across the three measured sequences. Component
medians need not sum to total medians. No other project build/sanitizer job ran during measurement.

Each one-shot solve must converge with inlier RMSE <=0.001 mm. Each reused result must equal its
one-shot counterpart exactly in transform matrix, convergence, iterations, pair count, fitness and
RMSE. All rows passed. Tests additionally compare movement metrics, ownership/lifetime behavior,
organized padding, changed frame sizes, independent concurrency and failure recovery.

## Results

| Points | Method | Backend | Prepare ms | First frame ms | Later frame ms | Reused 5 frames ms | One-shot 5 frames ms |
|---:|---|---|---:|---:|---:|---:|---:|
| 4225 | point_to_plane | cpu | 0.558 | 18.834 | 18.961 | 95.608 | 97.957 |
| 4225 | point_to_plane | gpu | 0.979 | 0.761 | 0.796 | 5.303 | 9.536 |
| 4225 | point_to_point | cpu | 0.420 | 12.443 | 12.673 | 63.575 | 67.039 |
| 4225 | point_to_point | gpu | 0.952 | 1.323 | 1.314 | 7.727 | 12.210 |
| 4225 | gicp | cpu | 6.927 | 19.992 | 19.966 | 106.906 | 135.950 |
| 4225 | gicp | gpu | 7.822 | 8.067 | 7.707 | 47.032 | 78.818 |
| 66049 | point_to_plane | cpu | 7.742 | 204.156 | 207.052 | 1040.670 | 1065.630 |
| 66049 | point_to_plane | gpu | 14.338 | 12.367 | 12.179 | 76.728 | 139.778 |
| 66049 | point_to_point | cpu | 7.973 | 216.257 | 219.814 | 1119.880 | 1108.020 |
| 66049 | point_to_point | gpu | 14.447 | 20.325 | 20.095 | 117.275 | 175.163 |
| 66049 | gicp | cpu | 108.682 | 331.218 | 340.277 | 1801.820 | 2203.040 |
| 66049 | gicp | gpu | 109.085 | 151.972 | 146.938 | 846.510 | 1133.630 |

At 66,049 points over five scans, indexed GPU point-to-plane registration falls from 139.778 ms
to 76.728 ms (45.1% lower), and GPU GICP from 1133.630 ms to 846.510 ms (25.3% lower).
CPU GICP falls from 2203.040 ms to 1801.820 ms (18.2% lower). Reference preparation is amortized;
scan covariance and ordered optimization remain per-frame CPU work.

CPU point-to-point at 66,049 points measured 1119.880 ms reused versus 1108.020 ms one-shot
(about 1.1% slower). CPU indexed preparation is a small fraction of its total workload, so these
three samples do not establish a speedup for that case. Call order is fixed and the sample set is
small; no significance or universal crossover policy is claimed. Additional retained reference
storage and a caller-chosen capacity are the explicit memory tradeoff. No million-point full
registration claim follows from the previous search-only benchmark.

## Reproduce

```powershell
& ./scripts/build.ps1 -Preset windows-cuda-release
cmake --build --preset windows-cuda-release --target pointcloud_ad_context_benchmark
./out/build/windows-cuda-release/bin/pointcloud_ad_context_benchmark.exe
```

The same benchmark target also builds with CPU-only presets and prints CPU rows. Raw data,
including the intermediate 16,641-point case: [PCAD-HET-003.csv](PCAD-HET-003.csv).
Next profiling targets are deterministic CPU correspondence/accumulation parallelism and SIMD;
this task does not silently add cross-run caching to InspectionPipeline.
