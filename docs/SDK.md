# PointCloudAD SDK 0.1

## CPU and GPU registration (M10–M16)

Include `pointcloud_ad/registration_engine.hpp` for direct registration. Set
`RegistrationParameters::method` to `point_to_plane`, `point_to_point` or `gicp`, and pass
`ComputeBackend::cpu`, `gpu` or `automatic` to `register_surfaces`. Inputs are borrowed for the
synchronous call; outputs own their transform/metrics. Units are millimetres and poses map scan to
reference. The direct API validates frame direction, units, geometry and solver settings.

Existing configuration files retain CPU point-to-plane behavior. Optional JSON fields are:

```json
{
  "registration": { "method": "gicp" },
  "execution": { "backend": "gpu" }
}
```

This is a fragment to merge into a complete validated configuration, not a standalone config.
The corresponding C++ fields are `config.registration.method` and `config.execution.backend`.
`auto` uses the size/device policy below; explicit GPU returns an error if unavailable. GICP and
point-to-point accept XYZ without normals through the direct registration API. The inspection
pipeline still prepares normals for the downstream deviation/classification stages.

Build CUDA variants with an installed CUDA Toolkit 13.2+ and a compatible C++20 host compiler:

```powershell
pwsh ./scripts/build.ps1 -Preset windows-cuda-debug
pwsh ./scripts/build.ps1 -Preset windows-cuda-release -VerifyInstall
```

The convenience CUDA presets compile for the local GPU (`CMAKE_CUDA_ARCHITECTURES=native`). Release
distributors must explicitly select/test their deployment architectures instead. The optional
vcpkg `cuda` feature validates the system Toolkit; its port is pinned by the manifest registry
baseline. It does not download a second dependency toolchain. The CUDA runtime is statically linked;
a compatible NVIDIA driver is still required at execution. CPU presets have no CUDA dependency.

GPU mode executes resident transforms, search, robust objectives, covariance and reductions, with
CPU index construction and the 6x6 solve. Results report `gpu_resident_objective_cpu_solve`. CUDA exact search uses a
balanced spatial index built on CPU and uploaded once per solve; it preserves inclusive-radius and
original-index tie semantics. Use `RegistrationContext` to reuse the index and reference covariance across separate scans.
See [the design and supported matrix](architecture/HETEROGENEOUS_REGISTRATION.md) for metric
semantics and repeatability, and [measured preparation/search costs](benchmarks/PCAD-HET-002.md). Adding public fields requires rebuilding 0.1 C++
consumers; this release line does not promise binary ABI compatibility.

### Repeated registration against one reference

`RegistrationContext::create(reference, parameters, max_scan_points, backend)` returns a move-only
owning context. It copies points, optional normals, validity mask, grid and frame before returning,
then prepares CPU search or resident CUDA search and reference GICP covariance once. Caller reference
storage may be mutated or destroyed afterward. Its full solver parameters, method, backend and
capacity are fixed; create a new context to change any of them. There is no implicit pointer-keyed
cache and no automatic invalidation to manage.

```cpp
// reference and scan are normalized SurfaceViews; initial maps scan.frame() -> reference.frame().
auto prepared = pointcloud_ad::RegistrationContext::create(
    reference, parameters, maximum_scan_points, pointcloud_ad::ComputeBackend::gpu);
if (prepared) {
  auto context = std::move(prepared).value();
  auto result = context.align(scan, initial);
  // Handle Result, then call context.align(next_scan, next_initial) for subsequent frames.
} else {
  // Handle prepared.error(), including unavailable GPU or invalid reference/parameters.
}
```

Capacity is a positive number of **logical** scan points, including masked points, excluding grid
padding, at most INT32_MAX. Choose the maximum expected frame size; CUDA allocates query buffers
once at preparation. Smaller frames are allowed; larger frames return an error, with no buffer
growth or reference rebuild. CPU enforces the same limit. Retained memory includes the reference
snapshot and private geometry/search copies, plus reference covariance for GICP and GPU capacity
buffers where selected. There is no cross-context sharing or cache eviction.

Every align call validates current scan data, units, frame direction and capacity. It computes fresh
scan covariance for GICP and starts from the supplied initial pose; previous poses and scans are
not retained as hidden initial guesses. Invalid inputs leave the context usable. Moved-from align
returns an error; moving transfers resource ownership, and destruction releases it.

Calls, moves and destruction of the **same context must be serialized by the caller**. Independent
contexts can execute concurrently. GPU calls must run with the CUDA device selected at creation;
a changed device is rejected. Destruction releases device resources on their owning device and
restores the thread's prior device selection. Runtime device failures may require context recreation;
no device reset or fallback is performed. Scan buffers must remain valid and unchanged during align.

`register_surfaces` keeps its existing signature and uses a temporary context internally.
`InspectionPipeline::run` continues to use one-shot registration; cross-run preprocessing and
inspection caching are separate work. See [context benchmark](benchmarks/PCAD-HET-003.md) for
preparation, first-frame and subsequent-frame costs. Public method exports and out-of-line move /
destruction are checked by the standalone installed consumer.

PointCloudAD is delivered as a C++20 shared-library SDK. On Windows the public runtime is
`pointcloud_ad.dll`, consumers link `pointcloud_ad.lib`, and CMake consumers use the imported target
`PointCloudAD::pointcloud_ad`.

## Package layout

```text
bin/                         pointcloud_ad.dll, pcad, and runtime DLL dependencies
include/pointcloud_ad/       public C++ headers
lib/                         import/shared libraries
lib/cmake/PointCloudAD/      find_package configuration
examples/sdk_consumer/       minimal standalone SDK consumer
share/licenses/              third-party copyright and license files
```

Only headers under `include/pointcloud_ad/` are public. They do not expose PCL, Eigen, Boost, VTK,
filesystem handles, JSON types, or CLI types.

## Consume with CMake

Extract the SDK and configure the bundled example with the SDK root as the prefix:

```powershell
cmake -S examples/sdk_consumer -B build -G Ninja `
  -DCMAKE_BUILD_TYPE=Release `
  -DCMAKE_PREFIX_PATH=C:/path/to/pointcloud-ad-sdk-0.1.0-windows-x86_64
cmake --build build
```

At runtime, keep the SDK `bin/` directory on `PATH` or deploy its contents beside the consumer
executable. The release archive includes the PointCloudAD DLL, PCL/Boost runtime closure, and the
matching app-local MSVC/UCRT libraries.

## ABI and compatibility

- Version 0.1 exposes a C++ ABI and requires C++20.
- Windows consumers should use an ABI-compatible MSVC toolchain and the architecture named by the
  package.
- Patch releases in the 0.1 line preserve documented source compatibility where practical, but the
  project does not promise a stable C++ ABI until 1.0.
- Cross-language consumers require a future C ABI or language binding; the 0.1 DLL is a C++ SDK.

## Contracts

- Errors cross the public boundary as `Result<T>`; exceptions do not.
- Geometry is expressed in millimetres, right-handed frames, and column vectors.
- Registration transforms map scan coordinates to reference coordinates.
- Inspection verdicts (`PASS`, `FAIL`, `INDETERMINATE`) are domain results and are not process exit
  codes.
- Deterministic execution uses explicit configuration and a recorded random seed. CPU registration uses fixed blocks and merge order independent of its worker budget; see [the numerical contract](architecture/NUMERICAL_REPRODUCIBILITY.md).

See the public headers for ownership and failure contracts, and use `pcad validate-config` before a
CLI inspection.

## CPU execution budget (M12)

`RegistrationParameters::thread_count` defaults to 1 and is a positive maximum including the calling thread. A context owns at most 64 workers and at most one worker per 256-point block of its reference/capacity. Independent contexts have independent budgets; callers must budget concurrent contexts. The pipeline forwards `execution.thread_count`. Worker exceptions are collected, all jobs finish, and the lowest failing block is translated at the public Result boundary. No process-wide thread policy changes. Public struct layout changed: rebuild C++ consumers with matching headers/DLL.

## CPU arithmetic selection (M13)

`RegistrationParameters::cpu_kernel` selects `CpuKernel::automatic`, `scalar`, or `avx2` independently of correspondence backend. The resident GPU path does not execute these CPU arithmetic kernels. Automatic point-to-plane objective arithmetic stays scalar based on the measured baseline; transforms and the other objectives use AVX2 when available. Forced AVX2 also covers point-to-plane for explicit comparisons. Automatic dispatch checks the compiled implementation, CPUID AVX/AVX2/XSAVE/OSXSAVE and XCR0 XMM/YMM state. Forced unsupported AVX2 returns an error; auto safely uses scalar. `POINTCLOUDAD_ENABLE_AVX2=OFF` builds the portable fallback. The pipeline uses automatic arithmetic selection; explicit kernel overrides are currently a registration SDK control, not a JSON schema field. Rebuild C++ consumers after the parameter-layout addition.

## Repeated inspection (M15)

`InspectionContext::create(config, reference, max_scan_points)` owns a normalized reference snapshot, normal/boundary preparation and reusable registration context. `run(scan, request)` returns the same domain result as one-shot inspection with those inputs; timestamps and timings describe each new run. Reference preparation is paid at creation and excluded from per-scan normalize/preprocess timings. Callers may release/mutate original reference/configuration storage after successful creation. Scan inputs are borrowed only for the synchronous call. Capacity is logical scan point count, including organized layouts; wrong capacity/frame and moved-from use return Result errors. Calls on one context must be serialized, while independent contexts may run concurrently with independent thread budgets. Changing reference or configuration requires a new context; there is no hidden pointer/global cache. File I/O and serialization remain caller responsibilities.

## Automatic backend policy (M16)

Explicit CPU/GPU overrides are unchanged. Automatic selects GPU only when both reference logical point count and declared scan capacity are at least 65,536 and a CUDA device is available; otherwise it selects CPU. The one-shot API uses the actual scan logical count as capacity. Context selection is immutable, so smaller subsequent frames retain that selection. Masked logical points count toward the threshold, row padding does not. This conservative warm-registration policy is based on [M14](benchmarks/PCAD-GPU-003.md) and [complete inspection measurements](benchmarks/PCAD-PIPEPERF-003.md) on one host; cold startup, masks, hardware and geometry can change the crossover. It does not guarantee every full inspection is faster. Use explicit CPU for latency-sensitive small frames or explicit GPU after deployment measurements.

`RegistrationContext::backend()` reports the selected CPU/GPU backend (`automatic` only for a moved-from context). Inspection JSON reports both requested/actual backends. Missing CUDA uses CPU for auto; preparation/allocation/runtime failures after selection return `Result` errors without silent retry or backend changes. This makes resource failures observable. There is no device-memory budget promise. The GPU execution-scope string changed from the historical search-only value to `gpu_resident_objective_cpu_solve`; consumers matching this descriptive string must accept the new value. JSON schema version and quantitative metric meanings remain unchanged.

Linux builds require the OpenMP runtime used by the C++ compiler because static PCL/FLANN references it. For Clang on Debian/Ubuntu, install the matching `libomp-<clang-major>-dev` compiler runtime (CI uses `libomp-dev`). CMake resolves `OpenMP::OpenMP_CXX`; no compiler/library paths are hard-coded. GCC provides its runtime through the normal compiler installation.
