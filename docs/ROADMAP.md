# Implementation roadmap

Each row is an atomic implementation task. A task is complete only when its evidence is automated
and the repository-level definition of done in `AGENTS.md` is satisfied.

## AI execution batches

Development proceeds by verified AI execution batches rather than calendar weeks. Each batch must
finish implementation, tests, Debug/Release verification, formatting, static analysis, relevant
documentation, and a reproducible acceptance command before the next batch starts.

| Batch | Milestone | Core delivery | Exit gate |
|---|---:|---|---|
| B00 | M0.1 | Default DLL build, exports, runtime layout | Installed DLL consumer runs independently |
| B01-B03 | M1 | Errors/results, units, frames, transforms, config | Core has no PCL; contract tests pass |
| B04-B07 | M2 | Surface ownership, PCL facade, PLY/PCD | Round-trip and public-include audit pass |
| B08-B10 | M3 | Validity, ROI, sampling, normals, boundaries | Deterministic output and traceable mapping |
| B11-B15 | M4 | Registration contracts, robust ICP, quality gate | Good alignment passes; bad alignment is rejected |
| B16-B18 | M5 | Deviation, coverage, invalid statistics | AC-003 through AC-006 pass |
| B19-B21 | M6 | Classification, clustering, measurement, severity | Synthetic defect labels and measurements pass |
| B22-B25 | M7 | Pipeline, verdict, schemas, artifacts, CLI | One `inspect` command completes the vertical slice |
| B26-B30 | M8 | Acceptance matrix, determinism, sanitizers, benchmark | AC-001 through AC-012 and CI pass |
| B31-B33 | M9 | DLL dependency packaging, CPack, examples, RC | Release package runs on a clean Windows system |

Real industrial samples are requested before M3 ends. If unavailable, development continues with
fixed-seed synthetic data and industrial acceptance remains explicitly unverified. Actual severity
thresholds are requested after M6; repository and maintainer identity are needed only before M9.

| ID | Milestone | Deliverable | Depends on | Required evidence | Status |
|---|---:|---|---|---|---|
| PCAD-ENG-001 | M0 | Targets, presets, install/export | — | Configure and installed consumer | Complete |
| PCAD-ENG-002 | M0 | Warnings, formatting, sanitizers, CI | ENG-001 | CI smoke paths | Complete |
| PCAD-ENG-003 | M0.1 | Default DLL, export audit, runtime packaging | ENG-002 | DLL install/consumer | Complete |
| PCAD-CORE-001 | M1 | `ErrorCode`, `Error`, `Result<T>` | ENG-003 | Unit tests | Complete |
| PCAD-CORE-002 | M1 | Units, `Vec3`, `FrameId`, `RigidTransform` | CORE-001 | Transform tests | Complete |
| PCAD-CFG-001 | M1 | Strongly typed validated configuration | CORE-001 | Invalid-config matrix | Complete |
| PCAD-GEO-001 | M2 | `SurfaceView` and `OwnedSurface` | CORE-002 | Ownership/validation tests | Complete |
| PCAD-GEO-002 | M2 | Unit and frame normalization | GEO-001 | Normalization tests | Complete |
| PCAD-BE-001 | M2 | PCL facade, conversions, include boundary | GEO-001 | Public include audit | Complete |
| PCAD-IO-001 | M2 | PLY reader/writer adapter | BE-001 | Round-trip tests | Complete |
| PCAD-IO-002 | M2 | PCD reader/writer adapter | BE-001 | Round-trip tests | Complete |
| PCAD-PRE-001 | M3 | Finite-value, ROI, and valid-mask stage | GEO-002 | Mask tests | Complete |
| PCAD-PRE-002 | M3 | Voxel sampling and source mapping | PRE-001, BE-001 | Sampling tests | Complete |
| PCAD-PRE-003 | M3 | Normal estimation, orientation, boundary | PRE-002 | Normal tests | Complete |
| PCAD-REG-001 | M4 | Registration input and metrics contracts | PRE-003, BE-001 | Contract tests | Complete |
| PCAD-REG-002 | M4 | Robust point-to-plane ICP | REG-001 | AC-002 | Complete |
| PCAD-REG-003 | M4 | Registration quality gate | REG-002 | AC-007 | Complete |
| PCAD-CMP-001 | M5 | Scan-to-reference deviation field | REG-003 | AC-003/004 | Complete |
| PCAD-CMP-002 | M5 | Reference-to-scan coverage field | CMP-001 | AC-005/006 | Complete |
| PCAD-CMP-003 | M5 | Invalid reasons and statistics | CMP-002 | Reason matrix | Complete |
| PCAD-DET-001 | M6 | Dent/bump classification and clustering | CMP-003 | AC-003/004 | Complete |
| PCAD-DET-002 | M6 | Missing-material clustering | DET-001 | AC-005 | Complete |
| PCAD-DET-003 | M6 | Region measurements and severity rules | DET-002 | Region tests | Complete |
| PCAD-PIPE-001 | M7 | Stage orchestration, short-circuit, verdict | DET-003 | AC-001..010 | Complete |
| PCAD-SER-001 | M7 | Config/result schemas and JSON | PIPE-001 | Schema/golden tests | Complete |
| PCAD-SER-002 | M7 | PLY fields, manifest, and hashes | SER-001 | Output tests | Complete |
| PCAD-CLI-001 | M7 | `inspect`, `validate-config`, `version` | SER-002 | CLI integration | Complete |
| PCAD-TEST-001 | M8 | Synthetic generator and acceptance matrix | CLI-001 | AC-001..012 | Complete |
| PCAD-TEST-002 | M8 | Consumer, sanitizers, determinism gates | TEST-001 | Release gates | Complete |
| PCAD-BENCH-001 | M8 | 100k/1M-point staged benchmarks | TEST-002 | Baseline report | Complete |
| PCAD-REL-001 | M9 | Open-source files and release package | BENCH-001 | Release checklist | Complete |

## M10: heterogeneous registration

| ID | Milestone | Deliverable | Depends on | Required evidence | Status |
|---|---:|---|---|---|---|
| PCAD-HET-001 | M10 | Device-independent CPU/CUDA registration with point-to-point, point-to-plane and GICP objectives | REL-001 | CPU/CUDA algorithm matrix, configuration round-trip, Debug/Release and installed consumer | Complete |
| PCAD-HET-002 | M10 | Exact indexed CUDA correspondence search shared by all registration objectives | HET-001 | Brute-force oracle, index invariants, GPU sanitizers, CPU/CUDA regression and scaling benchmark | Complete |
| PCAD-HET-003 | M10 | Owned reusable reference registration contexts for CPU/CUDA and all three objectives | HET-002 | Snapshot lifetime, repeat/frame/capacity tests, installed SDK consumer and amortized benchmark | Complete |

## M11-M13: measured CPU and SIMD optimization

See [performance plan](PERFORMANCE_PLAN.md) and the completed M14-M16 tasks below.

| ID | Milestone | Deliverable | Depends on | Required evidence | Status |
|---|---|---|---|---|---|
| PCAD-PERF-001 | M11 | End-to-end inspection and registration baseline | HET-003 | Tests, builds and measured evidence in task card | Complete |
| PCAD-PERF-002 | M11 | Numerical contract and scalar regression | PERF-001 | Tests, builds and measured evidence in task card | Complete |
| PCAD-PERF-003 | M11 | Reusable registration workspace | PERF-002 | Tests, builds and measured evidence in task card | Complete |
| PCAD-CPU-001 | M12 | Bounded executor and parallel queries | PERF-003 | Tests, builds and measured evidence in task card | Complete |
| PCAD-CPU-002 | M12 | Fixed-block objective and metric reduction | CPU-001 | Tests, builds and measured evidence in task card | Complete |
| PCAD-CPU-003 | M12 | Parallel GICP covariance | CPU-002 | Tests, builds and measured evidence in task card | Complete |
| PCAD-SIMD-001 | M13 | Runtime scalar and AVX2 dispatch | CPU-003 | Tests, builds and measured evidence in task card | Complete |
| PCAD-SIMD-002 | M13 | SIMD coordinate transformation | SIMD-001 | Tests, builds and measured evidence in task card | Complete |
| PCAD-SIMD-003 | M13 | SIMD objective arithmetic | SIMD-002 | Tests, builds and measured evidence in task card | Complete |

## M14-M16: resident GPU and complete inspection optimization

| ID | Milestone | Deliverable | Depends on | Required evidence | Status |
|---|---|---|---|---|---|
| PCAD-GPU-001 | M14 | Resident scan, GPU point-to-plane/point-to-point objectives and compact deterministic reduction | SIMD-003 | Task-card tests, builds and measured acceptance | Complete |
| PCAD-GPU-002 | M14 | Exact GPU KNN, covariance preparation and resident GICP objectives | GPU-001 | Task-card tests, builds and measured acceptance | Complete |
| PCAD-GPU-003 | M14 | Resident transfer accounting, sanitizer and scale acceptance | GPU-002 | Task-card tests, builds and measured acceptance | Complete |
| PCAD-PIPEPERF-001 | M15 | Owned reusable inspection reference and registration context | GPU-003 | Task-card tests, builds and measured acceptance | Complete |
| PCAD-PIPEPERF-002 | M15 | Bounded parallel normal and boundary preprocessing | PIPEPERF-001 | Task-card tests, builds and measured acceptance | Complete |
| PCAD-PIPEPERF-003 | M15 | Parallel comparison, measured voxel/clustering policy and complete inspection benchmark | PIPEPERF-002 | Task-card tests, builds and measured acceptance | Complete |
| PCAD-POLICY-001 | M16 | Measured backend selection with explicit overrides and observable decisions | PIPEPERF-003 | Task-card tests, builds and measured acceptance | Complete |
| PCAD-VALID-001 | M16 | Cross-platform/backend regression and generated ground-truth acceptance | POLICY-001 | Task-card tests, builds and measured acceptance | Complete |
| PCAD-REL-002 | M16 | SDK release verification and evidence-based acceptance report | VALID-001 | Task-card tests, builds and measured acceptance | Complete |

## Milestone themes

- **M0 Engineering foundation:** build, test, install, CI, and AI contribution contract.
- **M1 Core contracts:** backend-independent failure, geometry, transforms, and configuration.
- **M2 Geometry and I/O:** owned/borrowed surfaces, PCL isolation, PLY/PCD.
- **M3 Preprocessing:** validity, ROI, sampling, normals, and boundaries.
- **M4 Registration:** robust point-to-plane ICP and explicit quality gates.
- **M5 Comparison:** directional deviation and reverse coverage fields.
- **M6 Detection:** defect labels, clustering, measurement, and severity.
- **M7 Product slice:** pipeline, schemas, artifacts, and full CLI.
- **M8 Hardening:** acceptance matrix, determinism, sanitizers, and benchmarks.
- **M9 Release:** public documentation, packaging, and v0.1 release.

- **M10 Heterogeneous registration:** shared CPU/CUDA objectives, indexed GPU search and reusable reference contexts.
- **M11 Performance foundation:** end-to-end baselines, numerical contract and reusable iteration buffers.
- **M12 CPU execution:** bounded workers, deterministic block reductions and parallel GICP covariance.
- **M13 SIMD:** runtime scalar/AVX2 dispatch, coordinate/objective kernels and [measured verification](benchmarks/PCAD-SIMD-003.md).
- **M14-M16:** resident GPU objectives, inspection caching/parallelism, automatic backend policy, generated-data acceptance and verified SDK packages; see [release evidence](release/M16.md). Industrial scans and additional physical hardware remain deployment gates.
