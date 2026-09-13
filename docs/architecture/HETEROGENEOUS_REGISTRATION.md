# Heterogeneous registration

The public `register_surfaces` entry point and `InspectionPipeline` select a registration objective
independently of its execution device. Existing domain, error, frame and ownership contracts remain.

```text
CLI / InspectionPipeline / C++ registration consumer
                       |
      validated method + backend selection
                       |
           shared registration optimizer
          /                            \
 CPU indexed correspondences    CUDA indexed correspondences
          \                            /
       stable double objective accumulation / solve
                       |
        registration metrics -> quality gate
```

## Supported matrix

| Objective | CPU | GPU | Required data | Reported inlier RMSE |
|---|---|---|---|---|
| `point_to_plane` | PCL indexed search | CUDA indexed exact search | Reference normals | Scalar point-to-plane distance, mm |
| `point_to_point` | PCL indexed search | CUDA indexed exact search | XYZ | Euclidean correspondence distance, mm |
| `gicp` | PCL indexed search | CUDA indexed exact search | XYZ, at least 12 valid points by default | Euclidean correspondence distance, mm |

All methods are local registration and require an initial pose within a useful correspondence basin.
They are not global-pose discovery algorithms. Optional source/reference normals reject back-facing
correspondences when both are supplied. GICP estimates reference neighborhood covariance once per context and scan covariance per call on CPU, sorts neighbors before accumulation, regularizes the eigenvalues, and rotates source
covariances during optimization. Its covariance controls are exposed on `RegistrationParameters`.
Pipeline/JSON GICP currently uses the documented defaults (12 neighbors, epsilon 0.001).

The GPU mode is deliberately **hybrid**. Only correspondence search executes on GPU;
preprocessing, covariance estimation, Huber weighting, double accumulation, 6x6 solve, quality
gates and defect inspection execute on CPU. All three methods use that same split. CUDA headers
and resource handles remain private. Each reusable context owns its stream and device buffers;
the reference and its spatial index are uploaded once per context and queries/results transferred
per iteration. One-shot calls use a temporary context. Align requires the device current at context
creation; switching it is rejected. Resource destruction temporarily selects the owning device and
restores the thread's previous device. No persistent global thread/device policy is changed.
The shared CPU solver uses one worker. The existing execution thread-count field is not a promise
of parallel registration; explicit CPU task partitioning is a follow-up task.

## Prepared reference lifetime (PCAD-HET-003)

`RegistrationContext` is an owning, move-only public pimpl. Its implementation holds a validated
`OwnedSurface` snapshot and a private prepared-backend interface. PCL/Eigen types, reference
covariances and search objects remain in `src/backends/pcl/`; CUDA handles and allocations remain
in `src/backends/cuda/`. The optimizer is shared by prepared and one-shot calls.

Creation fixes the full algorithm parameters, backend and maximum logical scan size. Reference
mutation, parameter changes and larger scan requirements are handled by explicit new-context
creation, never by address-based cache heuristics or silent rebuilding. Prepared state is private;
caller mutation/destruction of the original reference has no effect. CPU and GPU contexts follow
the same ownership and capacity contracts. Memory stays allocated until move assignment/destruction.

Per-frame input validation, scan extraction/covariance and ordered optimization remain CPU work.
No scan covariance, pose or correspondence results are reused between frames. Same-context calls
require external serialization because GPU scratch is mutable; independent contexts can run
concurrently. Invalid scan input does not change prepared reference state. The public boundary
translates errors to Result and detects moved-from contexts. GPU runtime recovery is not guaranteed.

The initial implementation retains both the original reference snapshot and private compact/search
representations. This trades additional O(N) host storage for simple ownership and unchanged grid,
mask and frame contracts. Reference GICP covariance contributes nine doubles per valid point.
The explicit scan bound reserves CUDA scratch without reallocating on varying frame sizes.

## Selection, determinism and failure

- `cpu`: portable indexed CPU implementation; requires no CUDA installation.
- `gpu`: requires a CUDA-enabled build and available NVIDIA device. Never silently falls back.
- `auto`: explicitly resolves to CPU until measured crossover policies are added in a later task.
- Unknown enum/string values fail validation; CUDA runtime failures become public `Result` errors.
- Stable source ordering, lower-index nearest-distance ties and ordered double reductions are used.
  CPU/GPU acceptance compares pose and metrics within 1e-5 on the committed synthetic fixture;
  repeated runs on each backend require identical transforms. This is not a cross-compiler or
  cross-device bitwise guarantee.
- Point-to-plane retains the existing partial-observability behavior; GICP/point-to-point reject
  rank-deficient normal equations. Fewer than six accepted correspondences do not converge.
- The result records method, requested/actual backend, execution scope and RMSE metric. Gate
  thresholds must be calibrated for the chosen metric; different objectives do not imply equivalent
  threshold calibration. GPU selection never changes PASS/FAIL/INDETERMINATE semantics.

## Performance limits and next tasks

CPU uses a KD-tree with double rechecking of nearest-distance ties. Since PCAD-HET-002, the CUDA
backend builds a balanced binary bounding-volume hierarchy on CPU: deterministic median splits
along the widest axis, leaves of at most 16 points, and original reference indices preserved after
packing. Its flat preorder representation has escape links, allowing stackless GPU traversal.
Double point-to-box lower bounds prune whole subtrees. Inclusive distance thresholds and each
subtree's minimum original index preserve exact nearest-neighbor ties, including duplicate clouds.
The former tiled exhaustive kernel remains a private test/benchmark oracle; users select an
execution backend, not an internal search strategy.

Index construction uses O(N) storage and balanced median partitions. Device payload is O(N+Q),
including reference coordinates, nodes, original indices, queries and results. Pruning reduces
visited points for favorable geometry; worst-case search is still O(N*Q). A balanced tree is not a
guarantee of logarithmic query time. Host construction, upload, allocation and traversal divergence
remain material costs. No approximate matching or global device/thread settings are introduced.

[PCAD-HET-002 measurements](../benchmarks/PCAD-HET-002.md) include exact-search microbenchmarks through
1,048,576 reference/query points and the existing three-objective registration fixture. They separate
preparation, transferred query time and total lifetime. Million-point search is not million-point
registration or full-inspection acceptance. Automatic selection remains CPU until workload-specific
crossover and real industrial accuracy tests justify a policy.

Reference reuse is implemented and measured in [PCAD-HET-003](../benchmarks/PCAD-HET-003.md).
Next atomic tasks should introduce deterministic CPU task partitioning, and runtime-dispatched SIMD after profiling. GICP covariance
and the shared reduction can migrate to GPU behind these contracts later. NDT/VGICP, multiscale
registration and global feature registration need their own algorithm-specific contracts and tests.

Design references and dependency decision: [ADR-0010](../adr/0010-heterogeneous-registration.md).
