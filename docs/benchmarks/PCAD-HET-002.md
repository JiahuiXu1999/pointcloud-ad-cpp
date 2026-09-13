# Exact indexed CUDA search, 2026-09-13

PCAD-HET-002, Windows MSVC 19.51 Release, Intel Core i5-13400F, CUDA 13.2.78, NVIDIA
RTX 5070 Ti (driver 616.64), native GPU architecture. Synthetic measurements on one machine;
not industrial acceptance, a cross-hardware guarantee, or a full-inspection throughput claim.

## Search benchmark

One warm-up and median of three measured runs per row. Each run creates and destroys its own
search context. Preparation includes host index construction, allocation, and reference/index
upload. Query time includes coordinate validation, host-to-device copy, kernel execution,
device-to-host result copy, and synchronization. Total includes preparation, one query and cleanup.
Input creation, driver initialization and post-timing correctness checks are excluded. Component
medians need not sum to the total median. Device payload counts allocated arrays, not driver/context
memory or peak process memory. CPU preparation is single-threaded.

Reference and query counts are equal. Lattice: 1 mm spacing, query z offset 0.125 mm, radius
0.5 mm. Plane: 0.02 mm spacing, z offset 0.0025 mm, radius 0.01 mm. Duplicates: identical points,
z offset 0.125 mm, radius 0.5 mm. Each returned index must match the analytically known original
index (zero for duplicates); any mismatch fails the benchmark. Direct unit tests independently
compare indexed and tiled search against an exhaustive double-distance oracle on less regular data.

| Workload | Points/queries | Strategy | Prepare ms | Query ms | Total ms | Device MiB |
|---|---:|---|---:|---:|---:|---:|
| Lattice | 16,384 | Tiled | 0.302 | 11.797 | 12.380 | 0.438 |
| Lattice | 16,384 | Indexed | 3.198 | 0.161 | 3.517 | 0.578 |
| Plane | 16,384 | Tiled | 0.324 | 11.549 | 12.222 | 0.438 |
| Plane | 16,384 | Indexed | 3.327 | 0.173 | 3.625 | 0.578 |
| Duplicates | 16,384 | Tiled | 0.284 | 11.817 | 12.363 | 0.438 |
| Duplicates | 16,384 | Indexed | 0.706 | 0.125 | 0.926 | 0.578 |
| Lattice | 131,072 | Indexed | 31.715 | 1.167 | 33.481 | 4.625 |
| Plane | 131,072 | Indexed | 31.203 | 1.000 | 32.503 | 4.625 |
| Lattice | 1,048,576 | Indexed | 312.707 | 9.381 | 322.849 | 37.000 |
| Plane | 1,048,576 | Indexed | 219.815 | 8.966 | 229.369 | 37.000 |
| Duplicates | 1,048,576 | Indexed | 51.968 | 6.894 | 59.359 | 37.000 |

At 16,384 lattice points, indexed query time is about 73x lower than tiled; the total lifetime is
only about 3.5x lower because preparation becomes dominant. Larger tiled rows are intentionally
omitted to avoid quadratic work; no measured million-point speedup ratio is claimed. Duplicate
performance benefits from subtree minimum-index tie pruning and is not representative of all
nonuniform clouds. Exact search can still visit all reference points in unfavorable geometry.

## Complete registration benchmark

Same anisotropic paraboloid, translation, analytic normals, warm-up and three-run median as
[PCAD-HET-001](PCAD-HET-001.md). Timings include a complete registration call: allocation,
preparation, covariance estimation where relevant, every correspondence/transfer, CPU optimization,
final evaluation and cleanup. All rows converged in two updates, CPU/GPU RMSE matched at printed
precision and was below 1.3e-7 mm. Noise-free residuals do not imply physical measurement accuracy.

| Points | Objective | CPU ms | Indexed GPU ms |
|---:|---|---:|---:|
| 289 | Point-to-plane | 0.7961 | 0.6583 |
| 289 | Point-to-point | 0.8824 | 0.8031 |
| 289 | GICP | 1.6426 | 1.5126 |
| 1,089 | Point-to-plane | 3.3395 | 1.0630 |
| 1,089 | Point-to-point | 3.6396 | 1.0918 |
| 1,089 | GICP | 6.5627 | 4.8765 |
| 4,225 | Point-to-plane | 14.0867 | 2.4421 |
| 4,225 | Point-to-point | 13.9706 | 2.7661 |
| 4,225 | GICP | 26.4499 | 16.4837 |

These are local registration fixtures, with no outlier/noise/poor-initial-pose throughput claims.
The million-point search benchmark does not validate million-point GICP or inspection. GICP still
spends time on CPU neighborhood covariance and optimization. Automatic backend selection stays CPU.

## Reproduce

Initialize the developer environment in the same PowerShell process, then build the optional targets:

```powershell
& ./scripts/build.ps1 -Preset windows-cuda-release
cmake --build --preset windows-cuda-release --target pointcloud_ad_cuda_search_benchmark pointcloud_ad_registration_benchmark
./out/build/windows-cuda-release/bin/pointcloud_ad_cuda_search_benchmark.exe
./out/build/windows-cuda-release/bin/pointcloud_ad_registration_benchmark.exe
```

Raw outputs: [search CSV](PCAD-HET-002-search.csv), [registration CSV](PCAD-HET-002-registration.csv).
The next priority is explicit reference-context reuse across registrations, including ownership,
invalidation and error semantics. This would amortize the measured preparation bottleneck;
SIMD and automatic dispatch need separate profiling and correctness gates.
