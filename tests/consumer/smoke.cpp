#include <iostream>
#include <optional>
#include <pointcloud_ad/config.hpp>
#include <pointcloud_ad/geometry.hpp>
#include <pointcloud_ad/normalization.hpp>
#include <pointcloud_ad/result.hpp>
#include <pointcloud_ad/status.hpp>
#include <pointcloud_ad/surface.hpp>
#include <pointcloud_ad/version.hpp>
#include <utility>

int main() {
  auto frame = pointcloud_ad::FrameId::create("consumer");
  if (!frame) {
    return 1;
  }
  auto surface = pointcloud_ad::OwnedSurface::create({pointcloud_ad::Vec3f{}}, {}, {}, std::nullopt,
                                                     pointcloud_ad::LengthUnit::millimeter,
                                                     std::move(frame).value());
  if (!surface || surface.value().size() != 1U) {
    return 1;
  }
  std::cout << pointcloud_ad::version_string() << '\n';
  return pointcloud_ad::version().major == 0 ? 0 : 1;
}
