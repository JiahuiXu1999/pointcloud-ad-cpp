# Changelog

All notable changes are documented here. This project follows Semantic Versioning once the first
public API release is published.

## [Unreleased]

## [0.1.0] - 2026-08-26

### Added

- M0 CMake/Ninja/vcpkg engineering foundation.
- Installable `PointCloudAD::pointcloud_ad` CMake target.
- Minimal version API and `pcad` command-line shell.
- Dependency-free CTest unit and CLI smoke tests.
- Compiler warning, sanitizer, formatting, and clang-tidy integration.
- Windows, Linux, sanitizer, and installed-consumer CI definitions.
- Default DLL/shared-library output with hidden-by-default symbols and an explicit public export.
- Automated Windows export, dynamic-dependency, runtime-layout, and clean installed-consumer gates.
- M1 dependency-free `ErrorCode`, `PipelineStage`, owned `Error`, and move-only `Result<T>` contracts.
- Explicit length units, project-owned vectors, validated UTF-8 frame IDs, and right-handed
  source-to-target rigid transforms with millimetre translation semantics.
- Strongly typed raw/validated inspection configuration with required production gates,
  determinism provenance, measurement error budget checks, and field-level validation errors.
- Public-header dependency audit and installed-consumer compilation of all M1 headers.
- Backend-neutral borrowed `SurfaceView` and immutable `OwnedSurface` contracts with organized
  topology, row stride, validity masks, and masked non-finite sample preservation.
- Deterministic surface normalization to millimetres and explicit target frames, including correct
  point translation and rotation-only normal handling.
- ADR-0002 accepting a pinned vcpkg PCL dependency behind `src/backends/pcl/` for the remaining M2
  facade and PLY/PCD adapters.
- PCL 1.15.1#1 manifest integration and an opaque internal backend surface with deterministic
  project-to-PCL round-trip conversion for points, normals, masks, and organized topology.
- Private PCL facade tests and shared preset dependency installation without adding backend symbols
  or dependency types to the installed public API.
- Private binary/ASCII PLY reader-writer adapters with stable path/field errors, explicit unit/frame
  input, mask persistence or finite-value derivation, optional normals, and organized round trips.
- Matching binary/ASCII PCD adapters and a shared bounds-checked PCL field codec, including corrupt
  input handling and runtime-safe Release install/consumer verification.
- M3 finite-value and inclusive AABB ROI preprocessing with explicit output masks, organized-padding
  handling, stable invalid-reason precedence, and deterministic repeatability tests.
- Deterministic signed-key voxel sampling with double-precision centroids, normalized optional
  normals, lexicographic output, and CSR source-to-sample traceability.
- M3 normal preparation with sorted-neighbor PCA, minimum-support reasons, direction/viewpoint
  orientation evidence, explicit unproven status, and deterministic organized/radius boundaries.
- M4 backend-neutral registration input and metrics contracts with validated scan-to-reference
  transforms, solver parameters, neutral convergence status, and quantitative fitness/RMSE/delta
  metrics that remain independent of ICP implementation and quality-gate verdicts.
- M4 deterministic robust point-to-plane ICP behind the PCL backend with Huber weighting, per-pass
  correspondence and back-facing rejection, a once-per-call reference KD-tree, twist-based
  incremental updates, SO(3) re-orthonormalization, and double-precision transform accumulation.
- M4 backend-neutral registration quality gate evaluating convergence, pair count, overlap, inlier
  residual, and translation/rotation priors in a fixed specification order without coupling to
  verdicts or process exit codes.
- M5 scan-to-reference deviation field with a per-storage-index signed deviation, Euclidean
  distance, and normal angle, plus invalid reasons (input-invalid, no-neighbor, normal-missing,
  normal-mismatch, reference-boundary) computed over a once-per-call reference KD-tree.
- M5 reference-to-scan coverage field with a per-storage-index covered/no-neighbor/scan-boundary
  sample and an aggregate coverage ratio, computed over a once-per-call scan KD-tree.
- M5 deviation and coverage statistics with mean/RMS/max-abs/p95-abs deviation summaries, coverage
  ratio, and deterministic per-reason counts over both fields.
- M6 dent/bump classification and clustering with threshold-based candidate selection, separate
  3D Euclidean clustering per class, and minimum-cluster-size filtering.
- M6 missing-material clustering over `no_neighbor` reference points with the same 3D Euclidean
  clustering backend and minimum-cluster-size filtering.
- M6 defect-region measurement (centroid, AABB, deviation statistics, approximate area) and a
  configurable max-deviation severity rule mapping to info/warning/reject.
- M7 public `InspectionPipeline` entry point and `InspectionResult` report model orchestrating the
  normalize/preprocess/register/gate/transform/compare/coverage-gate/detect/finalize stages with
  short-circuit and PASS/FAIL/INDETERMINATE verdict semantics.
- M7 versioned config/result JSON serialization behind an internal facade (ADR-0008, nlohmann/json)
  with lowercase snake_case enums, UTC timestamps, and null for non-finite measurements.
- M7 output facade for attribute PLY, run manifest, and SHA-256 hashes with deterministic ASCII PLY
  fields (scalar/reason/region_id), a versioned manifest listing each output file and digest, and
  published-vector-verified hashing, all internal and independent of status/exit-code semantics.
- M7 `pcad` command-line vertical slice with `validate-config`, `inspect`, and `version` commands;
  `inspect` reads config and point-cloud files (by `.ply`/`.pcd` extension) and writes `result.json`
  and `manifest.json` while keeping inspection verdicts separate from the process exit code.
- M8 deterministic synthetic scene generator and acceptance matrix (`tests/synthetic/`) covering
  AC-001 through AC-012: identical/rigid/bump/dent/missing/dropout/bad-pose scenes with ground-truth
  metadata, plus flipped-normal, boundary-mask, NaN-input, and ten-run determinism checks. The
  pipeline now reports INDETERMINATE when no trustworthy deviation sample exists (for example when
  normals cannot be oriented), so an unproven comparison never produces a PASS.
- M8 release hardening gates for installed consumers, Linux ASan/UBSan, and serialized CLI
  determinism. The CLI integration now launches ten independent inspections and requires every
  business field (excluding UTC timestamps and wall-clock timings) to remain identical.
- M8 opt-in staged performance benchmark covering normalization, validity, and deterministic voxel
  sampling at 100k and 1M points, with JSON timings, peak RSS, checksums, scaling limits, and a
  recorded Windows MSVC Release baseline.
- M9 redistributable Windows SDK ZIP containing the shared library, CLI, public headers, CMake
  package, transitive runtime DLLs, runtime notices, and a standalone consumer example.
- Clean-package verification that rejects build-tree path leaks, rebuilds the bundled example from
  the extracted archive, and runs the packaged CLI and consumer on a restricted runtime path.
- Tag-driven GitHub Release automation for `v*` tags and v0.1 SDK integration, compatibility,
  security, and redistribution documentation.

[Unreleased]: https://github.com/JiahuiXu1999/pointcloud-ad-cpp/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/JiahuiXu1999/pointcloud-ad-cpp/releases/tag/v0.1.0
