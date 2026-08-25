# PCAD-PIPE-001: stage orchestration, short-circuit, and verdict

## Identity

- Task ID: PCAD-PIPE-001
- Milestone: M7 / B22
- Objective: Introduce the public `InspectionPipeline` entry point and `InspectionResult` report
  model, orchestrating all stages with the specification's short-circuit and verdict semantics.
- Dependencies: PCAD-DET-003

## Allowed scope

- Files/modules that may change: public headers `include/pointcloud_ad/inspection_result.hpp` and
  `include/pointcloud_ad/inspection_pipeline.hpp`, the implementation `src/pipeline/`, the export
  audit script, build files, its unit test, and M7 documentation.
- Public API changes: new `InspectionPipeline`, `InspectionRequest`, `InspectionResult`, and the
  report value types; the shared library now exports `InspectionPipeline` in addition to
  `version_string`.
- New dependencies: none.

## Contract

- Inputs and ownership: `create` takes an `InspectionConfig`; `run` borrows reference and scan
  `SurfaceView`s and an optional `InspectionRequest`.
- Outputs and ownership: `run` returns an owned `InspectionResult` with verdict, registration,
  coverage, deviations, regions, timings, provenance, and diagnostics.
- Units and coordinate frames: millimetres; scan-to-reference transforms.
- Determinism requirements: fixed stage order and no randomized behavior.
- Error cases: validation or normalization failures return `Result` errors; a rejected registration
  gate returns an INDETERMINATE report without running comparison or detection.
- Performance budget: one pass through the pipeline stages.

## Exclusions

- Explicitly out of scope: JSON serialization (PCAD-SER-001), PLY/manifest output (PCAD-SER-002),
  and the CLI (PCAD-CLI-001).
- Architecture boundaries that must not move: no PCL/Eigen enters the installed headers; the public
  error model remains `Result<T>`.

## Acceptance

- Required unit tests: identical surfaces yield PASS with no regions; a bump yields FAIL with one
  bump region; a grossly wrong initial pose is rejected by the gate and yields INDETERMINATE without
  detection.
- Required integration/golden tests: existing public-header audit, installed-consumer, I/O, and
  preprocessing gates remain green; the export audit allows only the public ABI.
- Required build presets: Windows Debug and Release install verification.
- Required documentation updates: roadmap, changelog, and milestone status.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`.
- Test results: the new pipeline test passes alongside the full suite.
- Remaining risks: severity uses a conservative default rule until user-configured severity rules
  arrive; serialization and the CLI consume this report model next.
