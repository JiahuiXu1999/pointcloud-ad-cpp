# Third-party notices

PointCloudAD uses the dependencies pinned by `vcpkg.json` and `vcpkg-configuration.json`, including
PCL and nlohmann/json and their transitive dependencies. They remain under their respective
licenses; Apache-2.0 applies only to PointCloudAD's own code.

Binary SDK archives contain the corresponding vcpkg copyright/license files under
`share/licenses/<port>/copyright`. Recipients should retain that directory when redistributing the
SDK or applications that bundle its third-party runtime libraries.

Optional CUDA builds also link NVIDIA's CUDA runtime under its own license. These SDK installations
include the Toolkit's `EULA.txt` under `share/licenses/nvidia-cuda/`. The vcpkg `cuda` port copyright
describes the port helper, not NVIDIA's runtime license. NVIDIA drivers are supplied separately.
