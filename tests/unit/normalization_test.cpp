#include <array>
#include <cmath>
#include <iostream>
#include <pointcloud_ad/normalization.hpp>
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

bool near(float actual, float expected, float tolerance = 1.0e-4F) {
  return std::abs(actual - expected) <= tolerance;
}

pointcloud_ad::FrameId frame(std::string_view value) {
  return std::move(pointcloud_ad::FrameId::create(std::string(value))).value();
}

} // namespace

int main() {
  using pointcloud_ad::GridTopology;
  using pointcloud_ad::LengthUnit;
  using pointcloud_ad::OwnedSurface;
  using pointcloud_ad::RigidTransform;
  using pointcloud_ad::Vec3f;

  bool passed = true;

  auto metre_surface = OwnedSurface::create({Vec3f{1.0F, 0.5F, -0.25F}}, {}, {}, std::nullopt,
                                            LengthUnit::meter, frame("fixture"));
  auto millimetres =
      pointcloud_ad::normalize_surface(metre_surface.value().view(), frame("fixture"));
  passed &= expect(static_cast<bool>(millimetres), "same-frame unit normalization must succeed");
  if (millimetres) {
    const auto view = millimetres.value().view();
    passed &= expect(view.unit() == LengthUnit::millimeter && view.frame().value() == "fixture",
                     "normalized metadata must use millimetres and the target frame");
    passed &= expect(near(view.points()[0].x, 1000.0F) && near(view.points()[0].y, 500.0F) &&
                         near(view.points()[0].z, -250.0F),
                     "point coordinates must be scaled exactly once");
  }

  auto scan_surface =
      OwnedSurface::create({Vec3f{1.0F, 2.0F, 3.0F}}, {Vec3f{1.0F, 0.0F, 0.0F}}, {1U},
                           GridTopology{1U, 1U, 1U}, LengthUnit::millimeter, frame("scanner"));
  constexpr std::array<double, 16> scan_to_fixture{0.0, -1.0, 0.0, 10.0, 1.0, 0.0, 0.0, 20.0,
                                                   0.0, 0.0,  1.0, 30.0, 0.0, 0.0, 0.0, 1.0};
  auto transform = RigidTransform::create(scan_to_fixture, frame("scanner"), frame("fixture"));
  auto aligned = pointcloud_ad::normalize_surface(scan_surface.value().view(), frame("fixture"),
                                                  std::move(transform).value());
  passed &= expect(static_cast<bool>(aligned), "matching frame transform must normalize");
  if (aligned) {
    const auto view = aligned.value().view();
    passed &= expect(near(view.points()[0].x, 8.0F) && near(view.points()[0].y, 21.0F) &&
                         near(view.points()[0].z, 33.0F),
                     "scan points must map into the reference frame");
    passed &= expect(near(view.normals()[0].x, 0.0F) && near(view.normals()[0].y, 1.0F) &&
                         near(view.normals()[0].z, 0.0F),
                     "normals must rotate without translation");
    passed &= expect(view.valid()[0] == 1U && view.grid() == GridTopology{1U, 1U, 1U},
                     "normalization must preserve mask and topology");
  }

  auto missing_transform =
      pointcloud_ad::normalize_surface(scan_surface.value().view(), frame("fixture"));
  passed &= expect(!missing_transform, "different frames require an explicit transform");

  auto wrong_transform = RigidTransform::create(scan_to_fixture, frame("camera"), frame("fixture"));
  auto wrong_direction = pointcloud_ad::normalize_surface(
      scan_surface.value().view(), frame("fixture"), std::move(wrong_transform).value());
  passed &= expect(!wrong_direction, "transform source must match the surface frame");

  return passed ? 0 : 1;
}
