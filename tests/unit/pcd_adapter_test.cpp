#include "pcd_adapter.hpp"

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
  using pointcloud_ad::io::PcdEncoding;
  using pointcloud_ad::io::read_pcd;
  using pointcloud_ad::io::write_pcd;

  const auto unique_suffix = std::chrono::steady_clock::now().time_since_epoch().count();
  TemporaryFiles files(std::filesystem::temp_directory_path() /
                       ("pointcloud_ad_pcd_adapter_tests_" + std::to_string(unique_suffix)));
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
  bool passed = expect(static_cast<bool>(surface), "PCD fixture must be valid");

  const auto binary_path = files.path("organized.PCD").string();
  auto written = write_pcd(binary_path, surface.value().view());
  auto restored = read_pcd(binary_path, LengthUnit::millimeter, frame("scanner"));
  passed &= expect(written && written.value() == 4U && restored, "binary PCD must round trip");
  if (restored) {
    const auto view = restored.value().view();
    passed &= expect(view.storage_size() == 4U, "PCD must contain only logical row-major points");
    passed &= expect(view.grid() == std::optional{GridTopology{2U, 2U, 2U}},
                     "PCD must preserve organized dimensions");
    passed &= expect(view.normals().size() == 4U, "PCD must preserve normals");
    passed &= expect(view.valid().size() == 4U && view.valid()[0] == 1U && view.valid()[1] == 1U &&
                         view.valid()[2] == 0U && view.valid()[3] == 1U,
                     "PCD must preserve the logical validity mask");
    passed &= expect(std::isnan(view.points()[2].x), "PCD must preserve masked NaN coordinates");
    passed &= expect(view.points()[3].x == 7.0F && view.normals()[3].x == 1.0F,
                     "PCD order and normals must be deterministic");
    passed &= expect(view.unit() == LengthUnit::millimeter && view.frame() == frame("scanner"),
                     "caller-supplied PCD semantics must be retained");
  }

  std::vector<Vec3f> ascii_points{{-1.0F, 0.5F, 3.0F}, {2.0F, 4.0F, 6.0F}};
  auto ascii_surface = OwnedSurface::create(std::move(ascii_points), {}, {}, std::nullopt,
                                            LengthUnit::meter, frame("fixture"));
  const auto ascii_path = files.path("small.pcd").string();
  auto ascii_written = write_pcd(ascii_path, ascii_surface.value().view(), PcdEncoding::ascii);
  auto ascii_restored = read_pcd(ascii_path, LengthUnit::meter, frame("fixture"));
  passed &= expect(ascii_written && ascii_restored, "ASCII PCD must round trip");
  if (ascii_restored) {
    const auto point = ascii_restored.value().view().points()[1];
    passed &= expect(point.x == 2.0F && point.y == 4.0F && point.z == 6.0F,
                     "ASCII PCD must preserve coordinates");
  }

  const auto missing_xyz_path = files.path("missing_xyz.pcd");
  {
    std::ofstream file(missing_xyz_path);
    file << "# .PCD v0.7\nVERSION .7\nFIELDS x y\nSIZE 4 4\nTYPE F F\nCOUNT 1 1\n"
            "WIDTH 1\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 1\nDATA ascii\n1 2\n";
  }
  auto missing_xyz = read_pcd(missing_xyz_path.string(), LengthUnit::millimeter, frame("fixture"));
  passed &= expect(!missing_xyz && missing_xyz.error().code == ErrorCode::unsupported_format,
                   "PCD without XYZ must return unsupported_format");

  const auto derived_mask_path = files.path("derived_mask.pcd");
  {
    std::ofstream file(derived_mask_path);
    file << "# .PCD v0.7\nVERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
            "COUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 2\n"
            "DATA ascii\n1 2 3\nnan nan nan\n";
  }
  auto derived_mask =
      read_pcd(derived_mask_path.string(), LengthUnit::millimeter, frame("fixture"));
  passed &= expect(derived_mask && derived_mask.value().view().valid().size() == 2U &&
                       derived_mask.value().view().valid()[0] == 1U &&
                       derived_mask.value().view().valid()[1] == 0U,
                   "PCD without a mask must derive validity from finite XYZ values");

  const auto corrupt_path = files.path("corrupt.pcd");
  {
    std::ofstream file(corrupt_path);
    file << "not a PCD file\n";
  }
  auto corrupt = read_pcd(corrupt_path.string(), LengthUnit::millimeter, frame("fixture"));
  passed &= expect(!corrupt && corrupt.error().code == ErrorCode::io_error,
                   "corrupt PCD must return io_error");

  auto empty_path = read_pcd("", LengthUnit::millimeter, frame("fixture"));
  auto wrong_extension = write_pcd("surface.ply", surface.value().view());
  passed &= expect(!empty_path && empty_path.error().code == ErrorCode::invalid_argument,
                   "empty PCD path must be rejected");
  passed &=
      expect(!wrong_extension && wrong_extension.error().code == ErrorCode::unsupported_format,
             "non-PCD extension must be rejected");

  return passed ? 0 : 1;
}
