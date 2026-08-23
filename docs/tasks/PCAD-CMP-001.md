# PCAD-CMP-001: scan-to-reference deviation field

## Identity

- Task ID: PCAD-CMP-001
- Milestone: M5 / B16
- Objective: Implement the scan-to-reference deviation field that measures signed surface deviation
  for every aligned scan point, without clustering, severity, or coverage.
- Dependencies: PCAD-REG-003, PCAD-BE-001

## Allowed scope

- Files/modules that may change: `src/comparison/deviation_field.{hpp,cpp}`, the PCL nearest
  neighbour backend `src/backends/pcl/pcl_comparison_backend.{hpp,cpp}`, their unit test, build
  files, and M5 documentation.
- Public API changes: none. The deviation field is an internal `pointcloud_ad::comparison` module
  consumed by later detection stages; no PCL/Eigen type enters the installed headers.
- New dependencies: none.

## Contract

- Inputs and ownership: `compute_deviation_field` borrows a millimetre reference, an optional
  reference boundary mask, and an already-aligned millimetre scan, plus a validated
  `ValidatedComparisonConfig`.
- Outputs and ownership: `DeviationField` owns per-storage-index samples aligned with the scan
  storage layout. Each sample carries `euclidean_mm`, `signed_mm`, `normal_angle_deg`, and a
  `DeviationReason`.
- Units and coordinate frames: distances in millimetres, angles in degrees, signed deviation along
  the reference normal.
- Determinism requirements: one KD-tree build, queries in storage order, no randomized step.
- Error cases: non-millimetre surfaces, missing reference normals, and a mismatched boundary mask
  return `Result` failures.
- Performance budget: one KD-tree build plus a linear nearest-neighbour pass over the scan.

## Exclusions

- Explicitly out of scope: reference-to-scan coverage (PCAD-CMP-002), unified invalid-reason
  statistics (PCAD-CMP-003), defect classification, clustering, and any verdict coupling.
- Architecture boundaries that must not move: PCL stays behind `src/backends/pcl/`; the module never
  decides a verdict or exit code.

## Acceptance

- Required unit tests: AC-003 (bump) and AC-004 (dent) assert the sign and depth of the signed
  deviation, identical surfaces yield zero deviation, and a scan outside the search radius reports
  `no_neighbor` instead of a false deviation.
- Required integration/golden tests: existing public-header audit, installed-consumer, I/O, and
  preprocessing gates remain green.
- Required build presets: Windows Debug and Release install verification.
- Required documentation updates: roadmap, changelog, and milestone status.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`.
- Test results: the new deviation-field test passes alongside the full suite.
- Remaining risks: the field is consumed by PCAD-CMP-002/003 and the M6 detection stage.
