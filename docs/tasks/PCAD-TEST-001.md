# PCAD-TEST-001: synthetic generator and acceptance matrix

## Identity

- Task ID: PCAD-TEST-001
- Milestone: M8 / B26
- Objective: Implement a deterministic synthetic scene generator and an automated acceptance matrix
  covering AC-001 through AC-012.
- Dependencies: PCAD-CLI-001

## Allowed scope

- Files/modules that may change: `tests/synthetic/` (generator and acceptance-matrix test), build
  files, the pipeline's finalize verdict rule for unproven comparisons, and M8 documentation.
- Public API changes: none beyond the pipeline verdict rule for zero valid deviation samples, which
  aligns with AC-008 and preserves PASS/FAIL/INDETERMINATE semantics.
- New dependencies: none.

## Contract

- Inputs and ownership: the generator produces owned reference/scan surfaces plus ground truth
  (pose, defect centre/depth, missing-block geometry); inputs are always millimetres with reference
  in frame `fixture` and scan in frame `scanner`.
- Outputs and ownership: each scene factory returns a `SyntheticScene` with deterministic geometry
  (no randomness), and the acceptance matrix asserts the spec's pass criteria per AC.
- Units and coordinate frames: millimetres; scan-to-reference transforms.
- Determinism requirements: fixed seeds and fully deterministic geometry; the determinism scene
  re-runs the same pipeline 10 times and asserts identical business fields.
- Error cases: NaN/Inf input must fail validation with `invalid_input` and never crash.
- Performance budget: each scene is a few thousand points; the whole matrix runs in well under a
  second of pipeline time.

## Exclusions

- Explicitly out of scope: sanitizer/consumer/install release gates (PCAD-TEST-002), staged
  benchmarks (PCAD-BENCH-001), and golden-file comparison harnesses.
- Architecture boundaries that must not move: PCL/Eigen never enter installed headers; the generator
  uses only public project types.

## Acceptance

- Required unit tests: AC-001 identical PASS with zero regions, AC-002 rigid pose recovery within
  0.02 mm/0.02°, AC-003 bump depth within `max(0.02, 5%)`, AC-004 dent sign/centre/depth,
  AC-005 missing-material recall >= 0.95, AC-006 dropout never PASS, AC-007 bad pose yields
  INDETERMINATE without detection, AC-008 flipped normals never PASS, AC-009 boundary-adjacent
  anomalies produce zero regions, AC-010 NaN input fails with `invalid_input`, AC-011 ten identical
  runs keep business fields stable, AC-012 is covered by the Release install consumer.
- Required integration/golden tests: existing public-header audit, installed-consumer, I/O, and
  preprocessing gates remain green.
- Required build presets: Windows Debug and Release install verification.
- Required documentation updates: roadmap, changelog, and milestone status.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`.
- Test results: the new acceptance-matrix test passes alongside the full suite (28 tests).
- Remaining risks: industrial acceptance remains unverified pending real sample clouds; golden files
  and staged benchmarks arrive in PCAD-TEST-002/BENCH-001.
