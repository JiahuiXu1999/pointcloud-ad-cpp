#include "pcl_backend.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <optional>
#include <pointcloud_ad/geometry.hpp>
#include <pointcloud_ad/surface.hpp>
#include <string>
#include <string_view>
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

} // namespace

int main() {
  using pointcloud_ad::GridTopology;
  using pointcloud_ad::LengthUnit;
  using pointcloud_ad::OwnedSurface;
  using pointcloud_ad::Vec3f;
  using pointcloud_ad::backends::pcl_backend::from_pcl_surface;
  using pointcloud_ad::backends::pcl_backend::to_pcl_surface;

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
  auto owned =
      OwnedSurface::create(std::move(points), std::move(normals), std::move(valid),
                           GridTopology{2U, 2U, 3U}, LengthUnit::millimeter, frame("scanner"));

  bool passed = expect(static_cast<bool>(owned), "organized padded fixture must be valid");
  auto backend = to_pcl_surface(owned.value().view());
  passed &= expect(static_cast<bool>(backend), "project-to-PCL conversion must succeed");
  auto restored = from_pcl_surface(backend.value(), LengthUnit::millimeter, frame("scanner"));
  passed &= expect(static_cast<bool>(restored), "PCL-to-project conversion must succeed");
  if (restored) {
    const auto view = restored.value().view();
    passed &= expect(view.storage_size() == 4U, "organized padding must be compacted");
    passed &= expect(view.grid() == std::optional{GridTopology{2U, 2U, 2U}},
                     "organized topology must be retained with packed rows");
    passed &= expect(view.normals().size() == 4U, "normal presence must be retained");
    passed &= expect(view.valid().size() == 4U && view.valid()[0] == 1U && view.valid()[1] == 1U &&
                         view.valid()[2] == 0U && view.valid()[3] == 1U,
                     "logical valid mask must be retained in row-major order");
    passed &= expect(std::isnan(view.points()[2].x), "masked non-finite point must be retained");
    passed &= expect(view.points()[3].x == 7.0F && view.normals()[3].x == 1.0F,
                     "logical point and normal order must be deterministic");
    passed &= expect(view.unit() == LengthUnit::millimeter && view.frame() == frame("scanner"),
                     "explicit semantic metadata must be retained");
  }

  return passed ? 0 : 1;
}
