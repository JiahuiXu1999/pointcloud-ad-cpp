# PCAD-CMP-002: reference-to-scan coverage field

## Identity

- Task ID: PCAD-CMP-002
- Milestone: M5 / B17
- Objective: Implement the reference-to-scan coverage field that decides, for every reference point,
  whether the aligned scan covers it, without clustering or missing-material detection.
- Dependencies: PCAD-CMP-001, PCAD-BE-001

## Allowed scope

- Files/modules that may change: `src/comparison/coverage_field.{hpp,cpp}`, the shared PCL nearest
  neighbour backend, its unit test, build files, and M5 documentation.
- Public API changes: none. The coverage field is an internal `pointcloud_ad::comparison` module.
- New dependencies: none.

## Contract

- Inputs and ownership: `compute_coverage_field` borrows a millimetre reference, an aligned
  millimetre scan, an optional scan boundary mask, and a validated `ValidatedComparisonConfig`.
- Outputs and ownership: `CoverageField` owns per-storage-index samples aligned with the reference
  storage layout, plus a covered count and coverage ratio in [0, 1].
- Units and coordinate frames: distances in millimetres.
- Determinism requirements: one KD-tree build over the scan, queries in reference storage order.
- Error cases: non-millimetre surfaces and a mismatched scan boundary mask return `Result` failures.
- Performance budget: one KD-tree build plus a linear nearest-neighbour pass over the reference.

## Exclusions

- Explicitly out of scope: unified invalid-reason statistics (PCAD-CMP-003), missing-material
  clustering, severity rules, and any verdict or coverage-gate decision (the ratio is produced, not
  judged).
- Architecture boundaries that must not move: PCL stays behind `src/backends/pcl/`.

## Acceptance

- Required unit tests: AC-005 (a missing block leaves the reference hole uncovered while the
  surrounding surface stays covered) and AC-006 (half coverage falls below the minimum coverage
  ratio) plus a full-scan control.
- Required integration/golden tests: existing public-header audit, installed-consumer, I/O, and
  preprocessing gates remain green.
- Required build presets: Windows Debug and Release install verification.
- Required documentation updates: roadmap, changelog, and milestone status.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`.
- Test results: the new coverage-field test passes alongside the full suite.
- Remaining risks: the coverage ratio feeds the M7 coverage gate; missing-material clustering is
  deferred to M6/PCAD-DET-002.
