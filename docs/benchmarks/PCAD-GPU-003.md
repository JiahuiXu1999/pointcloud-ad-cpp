# M14 resident GPU acceptance

Measured 2026-09-27 on Windows/MSVC 19.51, i5-13400F, RTX 5070 Ti 16 GiB, driver 616.64, CUDA 13.2.78. All timings use the Release CUDA build, explicit CPU/GPU backends and four CPU workers. The fixed curved fixture and known translation are shared with M11-M13. One warm-up, three measured runs, two reused frames per run; nearest-rank P50/P95. Runs were sequential without concurrent builds/analysis. Generation and correctness checks are outside timing; one-shot includes context creation/destruction, and preparation is excluded from reused calls. Short samples are not production tail-latency evidence.

## Complete registration P50 (ms)

| Points | Method | CPU one-shot | GPU one-shot | CPU reused | GPU reused |
|---:|---|---:|---:|---:|---:|
| 4,225 | point_to_plane | 11.702 | 1.648 | 10.851 | 0.401 |
| 4,225 | point_to_point | 10.794 | 1.769 | 9.694 | 0.411 |
| 4,225 | gicp | 20.092 | 3.764 | 14.222 | 1.546 |
| 100,489 | point_to_plane | 253.896 | 26.732 | 240.671 | 4.486 |
| 100,489 | point_to_point | 235.363 | 26.586 | 214.807 | 4.149 |
| 100,489 | gicp | 478.217 | 59.236 | 342.121 | 29.075 |
| 1,000,000 | point_to_plane | 2453.448 | 280.688 | 2327.716 | 44.711 |
| 1,000,000 | point_to_point | 2369.177 | 276.600 | 2185.938 | 45.444 |
| 1,000,000 | gicp | 4954.198 | 565.324 | 3456.315 | 289.099 |

The CPU GICP oracle now resolves KNN cutoff ties canonically in double precision. Rechecking and sorting candidates adds cost relative to the prior FLANN-selected subset (100k four-worker one-shot was 364 ms in M13 and is about 478 ms here). This is an explicit determinism/precision cost, not a CPU speedup. Existing numerical tolerances were retained. GPU figures include the new exact neighborhood policy. GPU index construction still runs on CPU once per reference/scan; reference covariance is reused, scan covariance is rebuilt once per frame. Cold driver/context startup is covered by warm-up, not these steady-state numbers.

## Transfers and storage

The private adapter counts explicit submitted host/device copy payloads, including the reference index, normals, scan data and GICP scan index. CUDA kernel argument/driver traffic, allocator/context overhead and device-to-device copies are excluded. Test-oracle covariance/KNN downloads are counted if invoked; the production solve does not invoke them. Two evaluated poses transfer **720 bytes D2H and zero point-array H2D bytes**, at every measured size. A GICP covariance validity result is one integer per surface. Transforms are passed as small kernel arguments. Thus per-iteration communication is independent of cloud size.

| Points | GICP | Preparation + scan H2D bytes | Preparation D2H bytes | Current device-array bytes |
|---:|---|---:|---:|---:|
| 4,225 | 0 | 402,560 | 0 | 634,860 |
| 4,225 | 1 | 500,920 | 8 | 2,609,936 |
| 100,489 | 0 | 9,498,352 | 0 | 15,007,100 |
| 100,489 | 1 | 11,761,496 | 8 | 37,665,936 |
| 1,000,000 | 0 | 93,242,840 | 0 | 148,055,880 |
| 1,000,000 | 1 | 114,485,680 | 8 | 330,018,128 |

Device-array storage is a current payload ledger, not process or GPU peak allocation. Replacing a scan index can transiently retain the previous index until the new one is ready. No peak-memory reduction claim is made. The 16k-query KNN workspace bounds scratch growth independently of total point count; requested K is still a material memory/work factor.

## Validation

- CPU Debug/Release: 35/35 each; CUDA Debug/Release: 37/37 each.
- Exact self-KNN compared with brute force, including duplicates, ties, K=1/3/12/32/all points; GPU covariance compared with independent Eigen results (1e-10), and rotated/nonzero GICP equations with scalar arithmetic (1e-9 relative scale).
- Existing heterogeneous pose/metric gates and frozen scalar regression remain unchanged. All benchmark solves converged with RMSE <=0.001 mm, CPU/GPU pose-entry differences <=1e-5 and equal pair counts; fresh/reused pose and RMSE matched exactly.
- Resident tests cover tails, empty correspondences, normal rejection, transform overflow, failed scan preparation and subsequent reuse, and fixed 360-byte iteration packets.
- Compute Sanitizer memcheck with full leak checking, racecheck and synccheck: zero errors/leaks/hazards. Format-check and configured C++ static analysis passed (advisory style diagnostics remain); CUDA device code is checked by NVCC and the device sanitizers.

## Reproduce

```powershell
. ./scripts/build.ps1 -Preset windows-cuda-release
cmake --build --preset windows-cuda-release --target pointcloud_ad_resident_benchmark
./out/build/windows-cuda-release/bin/pointcloud_ad_resident_benchmark.exe 65 3
./out/build/windows-cuda-release/bin/pointcloud_ad_resident_benchmark.exe 317 3
./out/build/windows-cuda-release/bin/pointcloud_ad_resident_benchmark.exe 1000 3
```

Timing CSV is stdout; byte/storage ledger is stderr. Raw data: [4,225](PCAD-GPU-003-65.csv), [100,489](PCAD-GPU-003-317.csv), [1,000,000](PCAD-GPU-003-1000.csv); transfer ledgers: [4,225](PCAD-GPU-003-65-transfers.txt), [100,489](PCAD-GPU-003-317-transfers.txt), [1,000,000](PCAD-GPU-003-1000-transfers.txt).

This is complete registration, not complete inspection or industrial acceptance. M15 measures inspection reference reuse/preprocessing/comparison; M16 handles measured dispatch and generated ground-truth acceptance. Different hardware, driver cold starts, difficult geometry, noise and real production samples can change results.
