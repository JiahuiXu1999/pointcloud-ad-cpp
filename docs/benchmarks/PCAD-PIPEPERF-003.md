# M15 complete inspection performance and ownership

Windows/MSVC 19.51, i5-13400F, RTX 5070 Ti 16 GiB, CUDA 13.2.78. Release CUDA binary, explicit CPU/GPU registration; preprocessing/comparison/detection remain CPU. One warm-up, three measured runs and six reused samples, nearest-rank P50/P95. Data creation and parity checks are outside timing. Every measurement runs a complete inspection, including normal/boundary preparation, registration, both comparison directions, gates and detection. Small sample P95 is not production tail-latency evidence.

`InspectionContext` owns a reference snapshot, its normals/boundaries and prepared registration search/covariance resources. The caller may release or mutate the original reference. Changing reference/configuration requires a new context. Each context must be serialized; independent contexts are independent. Preparation is timed separately; reused results exclude that one-time cost. The one-shot API still works. The reference comparison KD-tree is currently rebuilt per call, and coverage necessarily builds a scan tree each frame; measurements include both costs.

Normal estimation, unorganized boundary detection and comparison search use bounded 256-point blocks with private worker scratch. Normals keep their prior neighbor accumulation order. Query outputs retain storage indices, masks and padding; equal-distance behavior and inclusive search thresholds are preserved. Per-point classification/statistics and clustering keep their established deterministic ordering.

## Measured stage decisions

The inspection pipeline does not invoke the standalone voxel utility. Inserting downsampling into the inspection path would change point-level coverage/defect semantics; M15 therefore leaves that utility unchanged, with its existing ordering/mask tests. No voxel speedup is claimed. GPU transfers for preprocessing/comparison would require a wider resident inspection representation, whereas current measurements and exact threaded tests support the implemented CPU parallelism.

Detection is measured on both normal and Gaussian bump scans. Scalar clustering remains the policy for this release; defect-heavy/dense all-outlier scans may have different costs and are not proven by these samples. A future clustering change must retain stable component ordering and region IDs. No GPU clustering speedup is claimed.

## Validation

CPU Debug/Release: 36/36 each; CUDA Debug/Release: 38/38 each. Installed CPU/CUDA consumers exercised the new inspection context in PCAD-PIPEPERF-001. Exact multi-worker comparisons cover padded organized storage, masked queries, duplicate/tied points and inclusive search boundaries. Cached/fresh inspection tests cover changing scans, ownership mutation, invalid input then reuse, moves and independent contexts. Format-check and configured static analysis completed without compile errors; existing numeric-parameter/float-loop advisories were reviewed, as was intentional integer division used to create duplicate test points. No checks were disabled.

## Reproduce

```powershell
. ./scripts/build.ps1 -Preset windows-cuda-release
cmake --build --preset windows-cuda-release --target pointcloud_ad_inspection_context_benchmark
./out/build/windows-cuda-release/bin/pointcloud_ad_inspection_context_benchmark.exe 317 3 4
./out/build/windows-cuda-release/bin/pointcloud_ad_inspection_context_benchmark.exe 317 3 4 defect
```

Use sizes 65, 317 and 1000 and worker budgets 1 and 4. All normal runs must PASS without regions; bump runs must FAIL with regions; cached/fresh poses, RMS, coverage and region counts must match. These are generated fixtures, not industrial production evidence.

## Complete inspection P50 (ms)

| Points | Workers | Backend | One-shot | Prepare | Reused | Preprocess | Registration | Compare | Detect |
|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|
| 4225 | 1 | cpu | 40.525 | 13.761 | 25.920 | 12.678 | 8.869 | 4.210 | 0.016 |
| 4225 | 1 | gpu | 30.934 | 13.767 | 17.209 | 12.363 | 0.566 | 4.000 | 0.016 |
| 4225 | 4 | cpu | 21.889 | 6.723 | 17.279 | 5.924 | 7.834 | 3.085 | 0.016 |
| 4225 | 4 | gpu | 16.542 | 6.676 | 8.617 | 5.086 | 0.627 | 2.824 | 0.016 |
| 100489 | 1 | cpu | 892.208 | 313.688 | 964.229 | 352.439 | 386.471 | 181.473 | 0.385 |
| 100489 | 1 | gpu | 1178.835 | 474.498 | 637.864 | 459.621 | 6.427 | 162.146 | 0.385 |
| 100489 | 4 | cpu | 451.052 | 122.369 | 334.221 | 110.571 | 158.365 | 68.975 | 0.379 |
| 100489 | 4 | gpu | 485.782 | 203.474 | 283.613 | 178.081 | 6.324 | 90.627 | 0.385 |
| 1000000 | 1 | cpu | 9789.946 | 3293.379 | 6501.270 | 2997.956 | 2301.965 | 1120.021 | 4.450 |
| 1000000 | 1 | gpu | 7312.969 | 3201.144 | 4214.753 | 2973.401 | 100.517 | 1089.093 | 4.421 |
| 1000000 | 4 | cpu | 4687.330 | 1263.902 | 3368.596 | 1122.502 | 1600.561 | 561.021 | 4.739 |
| 1000000 | 4 | gpu | 3091.139 | 1368.909 | 1751.134 | 1107.014 | 70.421 | 543.003 | 4.446 |

Raw [1000000 points / 4 workers](PCAD-PIPEPERF-003-1000-4.csv).


Raw files for each size/budget are adjacent (`PCAD-PIPEPERF-003-{65,317,1000}-{1,4}.csv`). Earlier context/preprocessing stages: [PIPE001](PCAD-PIPEPERF-001.csv), [PIPE002](PCAD-PIPEPERF-002.csv).

### Defect workload (100489 points, four workers)

- cpu reused: P50 399.7405 ms; P95 450.5999 ms.
- cpu reused_stage_9: P50 0.849 ms; P95 0.984 ms.
- gpu reused: P50 182.0156 ms; P95 191.4832 ms.
- gpu reused_stage_9: P50 0.873 ms; P95 0.949 ms.

[Defect raw samples](PCAD-PIPEPERF-003-defect.csv). The bounded Gaussian defect occupies only a small subset; this supports retaining scalar clustering for this workload, not an all-defect scaling guarantee.

### Comparison with M13

The prior four-worker one-shot full inspection was 849.608 ms at 100489 points and 8425.475 ms at one million. M15 CPU one-shot is 451.052/4687.330 ms (about 47%/44% lower), while GPU-registration one-shot is 485.782/3091.139 ms. The 100k GPU one-shot is slower than the new CPU one-shot in this sample even though its registration is much faster: preprocessing/compare timings vary with scheduling/system state. The automatic policy therefore is a measured warm-registration heuristic, not a universal full-pipeline win. The million-point cached CPU/GPU figures of 3368.596/1751.134 ms exclude one-time reference preparation and must not be presented as equivalent one-shot costs. Measurements are short, single-host samples; deployers should measure their own workloads.
