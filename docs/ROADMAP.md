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
| PCAD-BENCH-001 | M8 | 100k/1M-point staged benchmarks | TEST-002 | Baseline report | Next |
| PCAD-REL-001 | M9 | Open-source files and release package | BENCH-001 | Release checklist | Pending |

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
