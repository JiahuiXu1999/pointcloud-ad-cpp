# PCAD-BENCH-001: staged 100k/1M-point benchmarks

## Identity

- Task ID: PCAD-BENCH-001
- Milestone: M8 / B29-B30
- Objective: Establish reproducible 100k/1M-point timing and peak-memory baselines for the
  normalization, validity, and deterministic voxel-sampling stages.
- Dependencies: PCAD-TEST-002

## Allowed scope

- Files/modules that may change: `benchmarks/`, benchmark target/build wiring, M8 documentation,
  and formatting coverage.
- Public API changes: none.
- New dependencies: none.

## Contract

- Inputs and ownership: deterministic planar point clouds with normals and valid masks, generated
  before timing at exactly 100,000 and 1,000,000 points.
- Outputs and ownership: JSON on stdout containing stage milliseconds, total milliseconds, peak RSS,
  valid/sample counts, and a deterministic output checksum.
- Units and coordinate frames: millimetres in frame `benchmark`; no frame transform.
- Determinism requirements: fixed point generation, ordered voxel output, and stable checksums.
- Error cases: a failed stage, total runtime over 180 seconds, scaling over 25x, or peak RSS over 2
  GiB returns a non-zero process exit code.
- Performance budget: the 1M case stays within the explicit portable guardrails above; recorded
  machine baselines are observational rather than universal promises.

## Exclusions

- Explicitly out of scope: microbenchmark statistics, GPU/CUDA, multithreading comparisons, full
  1M-point ICP, and hard real-time guarantees.
- Architecture boundaries that must not move: benchmark-only code may compile private preprocess
  sources but cannot expose them through the SDK.

## Acceptance

- Required unit tests: existing stage unit tests remain green.
- Required integration/golden tests: benchmark executable reports both fixed sizes, non-zero stable
  checksums, stage timings, peak RSS, and `within_limits: true`.
- Required build presets: Windows MSVC Release benchmark target plus the normal Debug/Release gates.
- Required documentation updates: roadmap, changelog, README, baseline report, task card, and
  milestone status.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -RunBenchmark` and the
  standard Debug/Release verification workflows.
- Test results: recorded in `docs/benchmarks/PCAD-BENCH-001.md`.
- Remaining risks: industrial workloads may have different spatial distributions; the baseline does
  not replace profiling on customer data.
