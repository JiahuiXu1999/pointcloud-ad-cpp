# PCAD-CMP-003: invalid reasons and statistics

## Identity

- Task ID: PCAD-CMP-003
- Milestone: M5 / B18
- Objective: Implement deterministic aggregate statistics and invalid-reason counts over the
  deviation and coverage fields, without defect classification or verdict coupling.
- Dependencies: PCAD-CMP-002, PCAD-CMP-001

## Allowed scope

- Files/modules that may change: `src/comparison/comparison_statistics.{hpp,cpp}`, its unit test,
  build files, and M5 documentation.
- Public API changes: none. Statistics are internal `pointcloud_ad::comparison` value types.
- New dependencies: none.

## Contract

- Inputs and ownership: `summarize_deviation` and `summarize_coverage` borrow their field and return
  owned plain-data summaries.
- Outputs and ownership: `DeviationStatistics` (mean, RMS, max-abs, p95-abs, and per-reason counts)
  and `CoverageSummary` (covered/valid counts, coverage ratio, and per-reason counts).
- Units and coordinate frames: deviations in millimetres.
- Determinism requirements: fixed traversal order and a deterministic percentile sort.
- Error cases: none; the functions are total over well-formed fields.
- Performance budget: one linear pass per summary plus an optional sort for the p95 percentile.

## Exclusions

- Explicitly out of scope: defect classification, clustering, severity rules, and any verdict or
  coverage-gate decision (the summary is produced, not judged).
- Architecture boundaries that must not move: the module never includes PCL or Eigen.

## Acceptance

- Required unit tests: known sample sets assert mean, RMS, max-abs, p95, and per-reason counts for
  both summaries, plus all-invalid boundary fields.
- Required integration/golden tests: existing public-header audit, installed-consumer, I/O, and
  preprocessing gates remain green.
- Required build presets: Windows Debug and Release install verification.
- Required documentation updates: roadmap, changelog, and milestone status.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`.
- Test results: the new statistics test passes alongside the full suite.
- Remaining risks: summaries are consumed by the M6 detection and M7 pipeline stages.
