#include <iostream>
#include <optional>
#include <pointcloud_ad/geometry.hpp>
#include <pointcloud_ad/normalization.hpp>
#include <pointcloud_ad/surface.hpp>
#include <pointcloud_ad/version.hpp>
#include <utility>

int main() {
  auto frame = pointcloud_ad::FrameId::create("sdk_example");
  if (!frame) {
    return 1;
  }
  auto surface = pointcloud_ad::OwnedSurface::create(
      {pointcloud_ad::Vec3f{1.0F, 2.0F, 3.0F}}, {}, {}, std::nullopt,
      pointcloud_ad::LengthUnit::millimeter, std::move(frame).value());
  if (!surface) {
    return 1;
  }
  auto target_frame = pointcloud_ad::FrameId::create("sdk_example");
  if (!target_frame) {
    return 1;
  }
  auto normalized = pointcloud_ad::normalize_surface(surface.value().view(),
                                                      std::move(target_frame).value());
  if (!normalized) {
    return 1;
  }
  std::cout << "PointCloudAD SDK " << pointcloud_ad::version_string() << ": "
            << normalized.value().size() << " point\n";
  return 0;
}
