# PointCloudAD SDK 0.1

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
- Deterministic execution uses explicit configuration, one thread, and a recorded random seed.

See the public headers for ownership and failure contracts, and use `pcad validate-config` before a
CLI inspection.
