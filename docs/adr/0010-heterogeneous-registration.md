# ADR-0010: heterogeneous registration with shared objectives

Status: accepted for PCAD-HET-001, 2026-09-13.

## Evidence and alternatives

- [Open3D tensor ICP](https://www.open3d.org/docs/latest/tutorial/t_pipelines/t_icp_registration.html)
  separates device placement from point-to-point / point-to-plane estimation and supports multiscale
  processing. We adopt the separation, without adding Open3D as a second geometry dependency.
- [small_gicp](https://github.com/koide3/small_gicp) separates preprocessing, search, factors and
  reduction. We adopt reusable per-solve search and covariance preparation.
- [fast_gicp](https://github.com/koide3/fast_gicp) exposes distinct CPU/GPU implementations and
  explicitly distinguishes CPU-tree, GPU brute-force and GPU voxel search. We likewise disclose the
  execution scope rather than presenting a hybrid solver as an entirely device-resident pipeline.
- [CUDA best practices](https://docs.nvidia.com/cuda/cuda-c-best-practices-guide/) motivates keeping
  the reference cloud resident and including transfers in timing.

## Decision

The public registration request separates objective (`point_to_plane`, `point_to_point`, `gicp`)
from device (`cpu`, `gpu`, `automatic`). Old configurations retain CPU point-to-plane behavior.
All objectives use the same millimetre, right-handed, scan-to-reference and Result<T> contracts.
The pipeline calls a backend-neutral dispatcher. PCL and Eigen remain under src/backends/pcl/;
CUDA includes, streams and allocations remain under src/backends/cuda/. Domain code knows neither.

CPU uses PCL's indexed search. GPU uses a tiled exact search, retaining the reference on the device
for the duration of a solve. This first GPU backend is a correctness baseline with O(N*M) work,
not a claimed million-point acceleration. Automatic selection stays on CPU until benchmarked size
thresholds exist. Explicit GPU never silently falls back. The GPU solve is hybrid: correspondence
search runs on CUDA; covariance preparation, robust accumulation and the small solve run on CPU.
Both modes support all three objectives. They do not imply GPU preprocessing/detection/I/O.

GICP estimates local covariances once, regularizes eigenvalues to (epsilon, 1, 1), rotates source
covariances at each iteration, and minimizes the Mahalanobis residual. Point-to-point uses vector
residuals; point-to-plane preserves its scalar residual. Huber weighting and ordered double
accumulation apply in both modes. Reports identify objective, requested and actual backend, and
the objective-specific RMSE metric; existing point-to-plane quality thresholds are not silently
reused as a claim of calibration for other objectives.

The fixed-seed ordering contract remains. Repeated runs on one build/device must be deterministic;
CPU/GPU comparisons use explicit numeric tolerances and identical business gates on acceptance
fixtures, not a cross-device bitwise promise. No fast-math or unordered floating-point atomics.

CUDA is opt-in through a vcpkg manifest feature pinned by the existing registry baseline and CMake
imported targets; no vendored sources or second dependency manager. A CPU package has no CUDA
runtime dependency. GPU packages require a compatible NVIDIA driver, separately from the Toolkit
needed to build. CPU-only builds return a structured unsupported-backend error for explicit GPU.

## Follow-up boundaries

NDT/VGICP, global RANSAC, multiscale schedules, device-resident covariance/reduction, cross-inspection
reference caches, SIMD and automatic performance thresholds are separate atomic tasks. They must
earn acceptance and measured benefit before becoming defaults. GPU support does not resolve the
existing real-industrial-sample acceptance gap.

## PCAD-HET-002 follow-up, 2026-09-13

The initial tiled CUDA search is superseded as the default by an exact flat BVH with deterministic
host median construction and stackless device traversal. Tiled search remains a private oracle.
This changes neither the dependency decision nor the public objective/device contract. See
[the current architecture](../architecture/HETEROGENEOUS_REGISTRATION.md) and
[verification and timings](../benchmarks/PCAD-HET-002.md). Cross-call reference reuse remains pending.

## PCAD-HET-003 follow-up, 2026-09-13

Cross-call reuse is now explicit through the owning `RegistrationContext` API. Full reference
snapshots and immutable algorithm/device/capacity settings avoid stale pointer-based cache keys.
CPU and CUDA preparation share the existing optimizer. This retains extra O(N) host state and
requires caller serialization per context; independent contexts are separate resources. The SDK
keeps its one-shot API through temporary contexts. See [ownership and concurrency](../SDK.md) and
[amortized measurements](../benchmarks/PCAD-HET-003.md). Inspection preprocessing caching is separate.
