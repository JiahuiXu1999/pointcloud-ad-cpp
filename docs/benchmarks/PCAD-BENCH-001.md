# PCAD-BENCH-001 baseline

## Environment

- Recorded: 2026-08-26
- Build: Windows MSVC Release, warnings as errors
- Compiler: MSVC 19.51 (`_MSC_VER=1951`)
- OS: Windows 11 Pro 10.0.26100
- CPU: Intel Core i5-13400F, 10 cores / 16 logical processors
- Memory: 16 GB
- Workload: deterministic planar points with normals and valid masks; 10k-point warm-up excluded
  from timings; voxel size 1.0 mm.

## Observed range

Three consecutive executions were recorded. Times are wall-clock milliseconds; peak RSS is the
process peak working set and therefore cumulative within each benchmark process.

| Input | Normalize | Validity | Voxel sampling | Total | Peak RSS | Sampled | Checksum |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 100,000 | 1.67-1.75 | 1.14-1.30 | 5.02-5.27 | 7.95-8.10 | 14.3-14.8 MiB | 1,024 | 2016294576301127947 |
| 1,000,000 | 15.34-16.41 | 10.72-11.29 | 54.89-56.67 | 80.95-84.38 | 98.1-98.6 MiB | 10,000 | 3891969989755537791 |

The observed total-time scaling ratio was 10.00-10.62x for a 10x input increase. Output counts and
checksums were identical across all executions.

## Regression guardrails

The executable returns non-zero when any portable guardrail is exceeded:

- 1M-point total time greater than 180 seconds;
- 1M/100k total-time scaling greater than 25x;
- peak RSS greater than 2 GiB; or
- any stage fails.

These limits catch severe regressions and pathological resource growth. The recorded timings are a
comparison baseline for equivalent hardware, not a real-time guarantee or an industrial-data
performance claim.
