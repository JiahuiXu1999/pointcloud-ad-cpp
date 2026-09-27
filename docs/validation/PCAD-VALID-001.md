# M16 generated ground-truth acceptance

The user authorized generated inputs because no industrial scans are available. This report verifies the implementation against labelled synthetic geometry; it does not establish factory accuracy, a production cycle time or portability to untested physical GPUs/CPUs.

## Dataset and checks

`scripts/validate-generated.py` uses only Python's standard library. Seed 5489, 0.25 mm grid spacing, supplied +Z normals and explicit scan/reference frames. The default 65×65 reference contains 4,225 points. A second 257×257 dataset contains 66,049 points and exercises automatic GPU selection. ASCII float32 coordinates with magnitude below 1e-8 mm are emitted as zero: otherwise Gaussian tails far below float32 range can become invalid values in the third-party ASCII parser. This floor is part of the recorded generator contract, far below the measurement tolerances. The validator rejects any discarded finite input sample.

| Scene | Recorded truth | Required result |
|---|---|---|
| Normal | Identical planar surfaces | PASS, no regions, max deviation ≤0.03 mm |
| Bump / dent | Signed ±0.7 mm Gaussian, sigma 1.5 mm, centre (0,0) | FAIL, correct type/sign, peak within 0.06 mm, centroid within 0.5 mm |
| Missing | Remove radius-3 mm disk | FAIL, missing-material region, at least 100 no-neighbour reference samples in the guaranteed core |
| Noise | Fixed-seed uniform Z noise ±0.015 mm | PASS, no regions, max deviation ≤0.03 mm |
| Density | Half the samples on positive-X side | PASS, no regions, coverage gate retained |
| Translated | Scan Z offset −0.1 mm | PASS, recovered scan-to-reference Z translation within 0.002 mm |
| Bad pose | Scan displaced +20 mm in Z | INDETERMINATE, process exits successfully |

Every run validates business verdict independently from process exit status. CPU worker counts and repeated same-backend results must be exactly equal after excluding timings/provenance and backend descriptors. Cross-backend floating quantities use an absolute 1e-5 budget; integral counts, defect types, ordering and verdicts must match exactly. No production threshold was relaxed. Wrong verdict, sign, peak, position and discarded-input mutations were independently checked to ensure the validator rejects them. Two independently generated directories produced identical manifests and input hashes.

The existing C++ AC-001–AC-012 suite now runs at one/four workers for CPU, automatic and available GPU backends. It additionally covers non-planar known rigid poses, boundaries, normal orientation, invalid inputs and coverage failures. Existing scalar/AVX2 objective, tail, dispatch, fixed-reference numerical and reusable-context tests remain enabled; cached contexts are covered by the SDK tests, while generated files exercise the CLI/I/O path.

## Reproduce

```powershell
python scripts/validate-generated.py --output out/validation/windows-cpu --pcad out/build/windows-msvc-release/bin/pcad.exe
python scripts/validate-generated.py --output out/validation/windows-cuda --pcad out/build/windows-cuda-release/bin/pcad.exe --backends cpu auto gpu
python scripts/validate-generated.py --output out/validation/windows-cuda-large --pcad out/build/windows-cuda-release/bin/pcad.exe --side 257 --backends cpu auto gpu --threads 4
```

Omit `--pcad` to generate inputs without running inspections. Output includes PLY files, `ground-truth.json` (SHA-256, dimensions, seed and labels), configurations, per-run result/manifest files and `validation-summary.json` with executable and ground-truth hashes. A failure writes `status: failed` and exits nonzero. CPU/GPU are explicit overrides; the small auto fixture uses CPU. At the large size auto uses GPU except the lower-density scan, whose actual point count falls below the capacity threshold.

Windows CPU/CUDA and Linux CPU CI run the generated validator through the same public executable. CUDA CI still requires a suitable self-hosted device; a generic hosted runner is not evidence of GPU execution.

## Recorded runs

- Windows CPU Release: [32 cases](PCAD-VALID-001-windows-cpu.json), CPU/auto × one/four workers × eight scenes.
- Windows CUDA Release: [48 cases](PCAD-VALID-001-windows-cuda.json), CPU/auto/GPU × one/four workers × eight scenes.
- Windows CUDA large dataset: [24 cases](PCAD-VALID-001-windows-cuda-large.json), CPU/auto/GPU × four workers × eight scenes.
- Windows CPU Release with AVX2 disabled: [32 cases](PCAD-VALID-001-windows-scalar.json); all 36 CTest tests passed. AVX2 was restored and the complete suite passed again.
- Final Windows CPU Debug/Release: 36/36 each; CUDA Debug/Release: 38/38 each, including the expanded multi-backend AC suite. Compute Sanitizer memcheck (resident and inspection contexts), racecheck and synccheck: zero errors, zero leaks/hazards. Configured static analysis and formatting passed (reviewed advisory diagnostics only).
- Default dataset truth and hashes: [manifest](PCAD-VALID-001-ground-truth.json).

The Windows host is an i5-13400F with RTX 5070 Ti 16 GiB, MSVC 19.51, CUDA 13.2.78 and driver 616.64. WSL2 tests use the same physical CPU, so they establish a different compiler/OS environment, not a second physical hardware result. Industrial labelled scans, other GPUs and deployment cold-start/tail-latency acceptance remain deployment gates.

## Linux environment

Ubuntu 22.04 under WSL2 kernel 6.6.87.2, GCC 11.4.0 and Clang 21.1.8 (the initial Clang 14.0.0 runtime was unsuitable); the vcpkg checkout is pinned to `45f9f39362a4c52e2b1fbe57b7e649db7f3d96d4`, matching the manifest/CI baseline. Dependencies were built with the existing vcpkg manager in a separate Linux cache, not replaced by distro PCL. Configure commands use the committed presets with a local `VCPKG_INSTALLED_DIR` override to avoid sharing a lock with Windows builds. GCC's strict warnings exposed signed KNN conversion and reduction alias shadowing; fixes preserve arithmetic and all warning flags. Clang requires its OpenMP compiler runtime for the existing static PCL dependency; CMake now reports a missing runtime during configuration.

GCC Release passed 36/36 tests, [32 generated cases](PCAD-VALID-001-linux-gcc.json), and the installed standalone consumer. All eight CPU business reports matched Windows within the declared absolute 1e-5 floating budget with exact counts/verdicts; dataset manifests and hashes matched exactly.

The initial Clang 14 sanitizer runtime failed nondeterministically before test execution on this WSL kernel (`vm.mmap_rnd_bits=32`). A standalone sanitized `int main(){return 0;}` reproduced SIGSEGV in 10/40 starts. Those failed runs are not counted as acceptance. [LLVM's upstream allocator-layout fix](https://reviews.llvm.org/D147984) explains the old-runtime conflict; validation uses a newer Clang/runtime from the [official LLVM package repository](https://apt.llvm.org/) instead. System ASLR, leak detection, sanitizer instrumentation and test thresholds remain enabled. The repository preset keeps a generic Clang name; this local environment selects a versioned compiler through CMake cache overrides.

Clang 21.1.8/runtime validation: 100/100 empty-program startups passed and an intentional heap overrun was diagnosed as `heap-buffer-overflow`. No ASLR or sanitizer option was disabled to obtain these results.

Clang 21.1.8 Debug with ASan+UBSan passed **36/36 tests** and [32 generated cases](PCAD-VALID-001-linux-clang.json), with `ASAN_OPTIONS=detect_leaks=1` and `UBSAN_OPTIONS=halt_on_error=1`. All eight business results match Windows under the same 1e-5 floating/exact-count contract, and all input hashes match. Sanitizers instrument project targets; third-party vcpkg archives keep their port build settings. Reproduce on a machine whose default Clang is older using:

```bash
cmake --fresh --preset linux-clang-asan -DCMAKE_CXX_COMPILER=clang++-21 -DCMAKE_C_COMPILER=clang-21
cmake --build --preset linux-clang-asan --parallel 3
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 ctest --preset linux-clang-asan
```

Set `VCPKG_ROOT` to the pinned checkout first; a separate installed-cache directory can be passed with `-DVCPKG_INSTALLED_DIR=...`. Local verification used a separate cache to avoid Windows/Linux lock contention. This does not change the dependency manager or pinned versions.
