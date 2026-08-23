# PCAD-REG-001: registration input and metrics contracts

## Identity

- Task ID: PCAD-REG-001
- Milestone: M4 / B11
- Objective: Introduce backend-neutral registration input and metrics contracts without
  implementing ICP, changing status/exit-code semantics, or exposing PCL/Eigen.
- Dependencies: PCAD-PRE-003, PCAD-BE-001

## Allowed scope

- Files/modules that may change: public contract header `include/pointcloud_ad/registration.hpp`,
  its unit tests, build files, and M4 documentation.
- Public API changes: new `RegistrationParameters`, `RegistrationInput`, `RegistrationConvergence`,
  and `RegistrationMetrics` types in the existing `pointcloud_ad` namespace.
- New dependencies: none.

## Contract

- Inputs and ownership: `RegistrationInput::create` borrows a reference `SurfaceView`, a scan
  `SurfaceView`, and takes a `RigidTransform` initial guess plus validated `RegistrationParameters`
  by value. The caller keeps the surface storage alive for the input lifetime.
- Outputs and ownership: `RegistrationMetrics` owns the final scan-to-reference transform and the
  quantitative fitness values; it carries no backend storage and is produced by a validating factory.
- Units and coordinate frames: surfaces and all lengths are millimetres; the initial and final
  transforms are scan-to-reference with validated frame direction; rotation deltas are degrees.
- Determinism requirements: the contracts are pure value types with no randomized behavior.
- Error cases: invalid units, empty surfaces, mismatched transform frame direction, and out-of-range
  parameters/metrics return `Result` failures with stable field-level context.
- Performance budget: contract construction is constant-time per field and performs no I/O.

## Exclusions

- Explicitly out of scope: ICP and correspondence search (PCAD-REG-002), the registration quality
  gate and its verdicts (PCAD-REG-003), and any inspection status or process exit-code coupling.
- Architecture boundaries that must not move: no PCL/Eigen include may enter
  `include/pointcloud_ad/`; the public error model remains `Result<T>`.

## Acceptance

- Required unit tests: input acceptance/accessors, empty surfaces, non-millimetre units, transform
  frame-direction mismatch, parameter range rejection, metrics acceptance/accessors, convergence
  enumerator rejection, and metric range/boundary rejection.
- Required integration/golden tests: existing public-header audit, installed-consumer, I/O, and
  preprocessing gates remain green.
- Required build presets: Windows Debug and Release install verification.
- Required documentation updates: roadmap, architecture, changelog, README, and milestone status.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`, and the Release
  `format`/`format-check` targets.
- Test results: Debug and Release each pass all CTest tests; the new registration contract tests,
  the public-header audit, and the installed consumer pass.
- Remaining risks: the metrics fields are populated by the PCAD-REG-002 solver and consumed by the
  PCAD-REG-003 gate, both of which are not yet implemented.
