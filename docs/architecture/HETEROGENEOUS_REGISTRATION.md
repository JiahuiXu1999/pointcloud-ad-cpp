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
correspondences when both are supplied. GICP estimates reference neighborhood covariance once per context and scan covariance per call on the selected backend, sorts neighbors before accumulation, regularizes the eigenvalues, and rotates source
covariances during optimization. Its covariance controls are exposed on `RegistrationParameters`.
Pipeline/JSON GICP currently uses the documented defaults (12 neighbors, epsilon 0.001).

GPU mode is hybrid: resident transforms, correspondence search, covariance estimation, Huber
weighting, objective/metric accumulation and deterministic reduction run on GPU. Index construction,
the 6x6 solve, convergence, preprocessing, quality gates and defect inspection remain CPU work.
CUDA headers and resource handles remain private. Each reusable context owns its stream and device
buffers; reference data uploads once per context, compact scan data once per align call, and each
iteration returns a 360-byte equation/statistics packet. One-shot calls use a temporary context.
Align requires the device current at context
creation; switching it is rejected. Resource destruction temporarily selects the owning device and
restores the thread's previous device. No persistent global thread/device policy is changed.
M12 uses a context-owned bounded executor for CPU queries, fixed-block objective/metric reduction
and GICP covariance. `thread_count` is a positive maximum including the caller, capped at 64 and
the available work blocks. M13 dispatches transform and objective arithmetic to portable scalar or
AVX2 kernels after CPU/OS capability checks. Blocks and merge order do not depend on worker count;
see [numerical contract](NUMERICAL_REPRODUCIBILITY.md) and the SDK guide.

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

Per-frame input validation and scan extraction remain CPU work; covariance/objective arithmetic follows the selected backend.
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
- `auto`: GPU when reference logical count and scan capacity are both >=65,536 and CUDA is available; otherwise CPU. Selection is fixed at context creation and observable through `backend()` and inspection requested/actual fields. See the SDK for warm-start and hardware limitations.
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
M11-M13 add measured CPU task partitioning and runtime-dispatched SIMD. See
[performance plan](../PERFORMANCE_PLAN.md) and [results](../benchmarks/PCAD-SIMD-003.md). GICP covariance
and the shared reduction can migrate to GPU behind these contracts later. NDT/VGICP, multiscale
registration and global feature registration need their own algorithm-specific contracts and tests.

Design references and dependency decision: [ADR-0010](../adr/0010-heterogeneous-registration.md).

## M14 resident arithmetic

PCAD-GPU-001 keeps a validated compact scan and its normalized normals on the device for the duration of an align call. Point-to-plane and point-to-point transform/search/filter/Huber/objective and final residual metrics run on the context's stream. Each iteration returns a 360-byte equation/statistics packet, including an invalid-coordinate flag; the existing CPU 6x6 solve and convergence semantics remain unchanged. A fixed 128-lane binary tree and increasing block merge order replace floating-point atomics. CPU/GPU use the existing metric-specific parity tolerances; repeated GPU calls remain exact. The old host-query search API remains as a test oracle; GICP is also resident after PCAD-GPU-002.

Reference order is reconstructed on-device only when resident arithmetic is enabled; search-only users retain their original memory footprint. Context resources drain on errors and are released on their owning device. No GPU/PCL handles enter public headers. PCAD-GPU-002 extends this path to covariance and GICP.

PCAD-GPU-002 adds exact device KNN and deterministic 3x3 covariance eigendecomposition. Reference covariances stay resident; each new scan constructs its own spatial index and covariances once. Iterative GICP covariance rotation, 3x3 information inversion and weighted equations then run on-device. KNN workspace is bounded to 16,384 queries per batch and 1,024 neighbors; insufficient, coincident or collinear support remains an input error. The covariance validity transfer is one integer; dense covariance downloads exist only for the private test oracle. Spatial-index construction remains CPU work performed once per scan, not per iteration.
