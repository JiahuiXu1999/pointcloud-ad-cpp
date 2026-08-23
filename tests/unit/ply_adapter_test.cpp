#include "ply_adapter.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <pointcloud_ad/geometry.hpp>
#include <pointcloud_ad/status.hpp>
#include <pointcloud_ad/surface.hpp>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

bool expect(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}

pointcloud_ad::FrameId frame(std::string_view name) {
  return std::move(pointcloud_ad::FrameId::create(std::string(name))).value();
}

class TemporaryFiles final {
public:
  explicit TemporaryFiles(std::filesystem::path directory) : directory_(std::move(directory)) {
    std::filesystem::create_directories(directory_);
  }

  TemporaryFiles(const TemporaryFiles&) = delete;
  TemporaryFiles& operator=(const TemporaryFiles&) = delete;

  ~TemporaryFiles() {
    std::error_code ignored;
    std::filesystem::remove_all(directory_, ignored);
  }

  [[nodiscard]] std::filesystem::path path(std::string_view name) const {
    return directory_ / name;
  }

private:
  std::filesystem::path directory_;
};

} // namespace

int main() {
  using pointcloud_ad::ErrorCode;
  using pointcloud_ad::GridTopology;
  using pointcloud_ad::LengthUnit;
  using pointcloud_ad::OwnedSurface;
  using pointcloud_ad::Vec3f;
  using pointcloud_ad::io::PlyEncoding;
  using pointcloud_ad::io::read_ply;
  using pointcloud_ad::io::write_ply;

  const auto unique_suffix = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto test_directory = std::filesystem::temp_directory_path() /
                              ("pointcloud_ad_ply_adapter_tests_" + std::to_string(unique_suffix));
  TemporaryFiles temporary_files(test_directory);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  std::vector<Vec3f> points{{1.0F, 2.0F, 3.0F},
                            {4.0F, 5.0F, 6.0F},
                            {99.0F, 99.0F, 99.0F},
                            {nan, nan, nan},
                            {7.0F, 8.0F, 9.0F}};
  std::vector<Vec3f> normals{{0.0F, 0.0F, 1.0F},
                             {0.0F, 1.0F, 0.0F},
                             {1.0F, 0.0F, 0.0F},
                             {nan, nan, nan},
                             {1.0F, 0.0F, 0.0F}};
  std::vector<std::uint8_t> valid{1U, 1U, 0U, 0U, 1U};
  auto surface =
      OwnedSurface::create(std::move(points), std::move(normals), std::move(valid),
                           GridTopology{2U, 2U, 3U}, LengthUnit::millimeter, frame("scanner"));
  bool passed = expect(static_cast<bool>(surface), "PLY fixture must be valid");

  const auto binary_path = temporary_files.path("organized.PLY").string();
  auto written = write_ply(binary_path, surface.value().view());
  passed &= expect(written && written.value() == 4U, "binary PLY write must report logical size");
  auto restored = read_ply(binary_path, LengthUnit::millimeter, frame("scanner"));
  passed &= expect(static_cast<bool>(restored), "binary PLY must round trip");
  if (restored) {
    const auto view = restored.value().view();
    passed &= expect(view.storage_size() == 4U, "PLY must contain only logical row-major points");
    passed &= expect(view.grid() == std::optional{GridTopology{2U, 2U, 2U}},
                     "PLY camera metadata must preserve organized dimensions");
    passed &= expect(view.normals().size() == 4U, "PLY must preserve normals");
    passed &= expect(view.valid().size() == 4U && view.valid()[0] == 1U && view.valid()[1] == 1U &&
                         view.valid()[2] == 0U && view.valid()[3] == 1U,
                     "PLY must preserve the logical validity mask");
    passed &= expect(std::isnan(view.points()[2].x), "PLY must preserve masked NaN coordinates");
    passed &= expect(view.points()[3].x == 7.0F && view.normals()[3].x == 1.0F,
                     "PLY order and normal values must be deterministic");
    passed &= expect(view.unit() == LengthUnit::millimeter && view.frame() == frame("scanner"),
                     "caller-supplied PLY semantics must be retained");
  }

  std::vector<Vec3f> ascii_points{{-1.0F, 0.5F, 3.0F}, {2.0F, 4.0F, 6.0F}};
  auto ascii_surface = OwnedSurface::create(std::move(ascii_points), {}, {}, std::nullopt,
                                            LengthUnit::meter, frame("fixture"));
  const auto ascii_path = temporary_files.path("small.ply").string();
  auto ascii_written = write_ply(ascii_path, ascii_surface.value().view(), PlyEncoding::ascii);
  auto ascii_restored = read_ply(ascii_path, LengthUnit::meter, frame("fixture"));
  passed &= expect(ascii_written && ascii_restored, "ASCII PLY must round trip");
  if (ascii_restored) {
    const auto point = ascii_restored.value().view().points()[1];
    passed &= expect(point.x == 2.0F && point.y == 4.0F && point.z == 6.0F,
                     "ASCII PLY must preserve coordinates");
  }

  const auto missing_xyz_path = temporary_files.path("missing_xyz.ply");
  {
    std::ofstream file(missing_xyz_path);
    file << "ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\n"
            "end_header\n1 2\n";
  }
  auto missing_xyz = read_ply(missing_xyz_path.string(), LengthUnit::millimeter, frame("fixture"));
  passed &= expect(!missing_xyz && missing_xyz.error().code == ErrorCode::unsupported_format,
                   "PLY without XYZ must return unsupported_format");

  const auto derived_mask_path = temporary_files.path("derived_mask.ply");
  {
    std::ofstream file(derived_mask_path);
    file << "ply\nformat ascii 1.0\nelement vertex 2\nproperty float x\nproperty float y\n"
            "property float z\nend_header\n1 2 3\nnan nan nan\n";
  }
  auto derived_mask =
      read_ply(derived_mask_path.string(), LengthUnit::millimeter, frame("fixture"));
  passed &= expect(derived_mask && derived_mask.value().view().valid().size() == 2U &&
                       derived_mask.value().view().valid()[0] == 1U &&
                       derived_mask.value().view().valid()[1] == 0U,
                   "PLY without a mask must derive validity from finite XYZ values");

  const auto corrupt_path = temporary_files.path("corrupt.ply");
  {
    std::ofstream file(corrupt_path);
    file << "not a PLY file\n";
  }
  auto corrupt = read_ply(corrupt_path.string(), LengthUnit::millimeter, frame("fixture"));
  passed &= expect(!corrupt && corrupt.error().code == ErrorCode::io_error,
                   "corrupt PLY must return io_error");

  auto empty_path = read_ply("", LengthUnit::millimeter, frame("fixture"));
  auto wrong_extension = write_ply("surface.pcd", surface.value().view());
  passed &= expect(!empty_path && empty_path.error().code == ErrorCode::invalid_argument,
                   "empty PLY path must be rejected");
  passed &=
      expect(!wrong_extension && wrong_extension.error().code == ErrorCode::unsupported_format,
             "non-PLY extension must be rejected");

  return passed ? 0 : 1;
}
