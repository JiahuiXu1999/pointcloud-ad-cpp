# PCAD-DET-003: region measurements and severity rules

## Identity

- Task ID: PCAD-DET-003
- Milestone: M6 / B21
- Objective: Measure each defect cluster into a `DefectRegion` and apply a user-configurable
  severity rule, without verdict or output coupling.
- Dependencies: PCAD-DET-002, PCAD-DET-001

## Allowed scope

- Files/modules that may change: `src/detection/defect_region.{hpp,cpp}`, its unit test, build
  files, and M6 documentation.
- Public API changes: none. `DefectRegion` and `SeverityRule` are internal detection types.
- New dependencies: none.

## Contract

- Inputs and ownership: `measure_deviation_region` borrows a cluster, an aligned scan, and a
  deviation field; `measure_missing_region` borrows a cluster and the reference. Both return an
  owned `DefectRegion`.
- Outputs and ownership: `DefectRegion` carries id, type, point count, centroid, AABB, deviation
  statistics (mean/rms/max/p95), an optional approximate area, and an approximate flag.
- Units and coordinate frames: positions and statistics in millimetres.
- Determinism requirements: fixed traversal and deterministic percentile sorting.
- Error cases: non-millimetre surfaces, mismatched deviation field, and out-of-range cluster indices
  return `Result` failures.
- Performance budget: linear member pass plus an O(n^2) nearest-neighbour area estimate.

## Exclusions

- Explicitly out of scope: verdict formation, serialization, and any process exit-code coupling.
- Architecture boundaries that must not move: the module never includes PCL or Eigen.

## Acceptance

- Required unit tests: a synthetic bump verifies centroid, max/mean deviation, AABB, and approximate
  area, plus the three-way severity rule mapping.
- Required integration/golden tests: existing public-header audit, installed-consumer, I/O, and
  preprocessing gates remain green.
- Required build presets: Windows Debug and Release install verification.
- Required documentation updates: roadmap, changelog, and milestone status.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`.
- Test results: the new defect-region test passes alongside the full suite.
- Remaining risks: regions feed the M7 pipeline and serialization stages.
