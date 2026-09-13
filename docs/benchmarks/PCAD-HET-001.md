# Heterogeneous registration baseline, 2026-09-13

Windows MSVC 19.51 Release, CUDA 13.2.78, NVIDIA RTX 5070 Ti (driver 616.64), native GPU architecture.
Synthetic anisotropic paraboloid, fixed 0.03/-0.02/0.04 mm scan-to-reference translation, supplied
analytic normals. Every solve converged in two updates. One warm-up and median of three solves per
row. Timing includes per-solve allocation, reference upload, query transfers, covariance preparation
where relevant, final evaluation and cleanup; driver initialization and input creation are excluded.
This is a registration-only baseline, not full inspection or industrial throughput.

| Points | Method | CPU ms | GPU ms |
|---:|---|---:|---:|
| 289 | Point-to-plane | 1.1104 | 0.9268 |
| 289 | Point-to-point | 0.8680 | 0.8253 |
| 289 | GICP | 1.6838 | 2.0010 |
| 1,089 | Point-to-plane | 3.7319 | 2.2267 |
| 1,089 | Point-to-point | 3.4795 | 2.2175 |
| 1,089 | GICP | 7.0012 | 5.6795 |
| 4,225 | Point-to-plane | 13.8108 | 5.8751 |
| 4,225 | Point-to-point | 14.2175 | 6.2532 |
| 4,225 | GICP | 27.8183 | 19.6072 |

CPU/GPU RMSE matched at the printed precision for every row, all below 1.3e-7 mm. These unusually
small residuals reflect the noise-free fixture and are not measurement accuracy claims. GPU is
slower for the smallest GICP fixture. Neither these limited samples nor the quadratic GPU search
justify an automatic selection threshold or a million-point speedup claim.

Reproduce after initializing the developer environment with the documented build script:

```powershell
cmake --build --preset windows-cuda-release --target pointcloud_ad_registration_benchmark
./out/build/windows-cuda-release/bin/pointcloud_ad_registration_benchmark.exe
```

Raw output: [PCAD-HET-001.csv](PCAD-HET-001.csv). The benchmark fails if registration does not
converge or its inlier RMSE exceeds 0.001 mm. Indexed CUDA search and CPU SIMD require a separate
task with larger, varied datasets before making production performance claims.

This historical tiled-search baseline is superseded by [PCAD-HET-002](PCAD-HET-002.md).
