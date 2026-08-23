#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <pointcloud_ad/surface.hpp>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

bool expect(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}

pointcloud_ad::FrameId frame(std::string_view name = "fixture") {
  return std::move(pointcloud_ad::FrameId::create(std::string(name))).value();
}

} // namespace

int main() {
  using pointcloud_ad::GridTopology;
  using pointcloud_ad::LengthUnit;
  using pointcloud_ad::OwnedSurface;
  using pointcloud_ad::SurfaceView;
  using pointcloud_ad::Vec3f;

  static_assert(!std::is_copy_constructible_v<OwnedSurface>);
  static_assert(std::is_move_constructible_v<OwnedSurface>);

  bool passed = true;

  auto empty = OwnedSurface::create({}, {}, {}, std::nullopt, LengthUnit::millimeter, frame());
  passed &= expect(!empty, "empty point storage must be rejected");

  std::vector<Vec3f> points{{1.0F, 2.0F, 3.0F}, {4.0F, 5.0F, 6.0F}};
  std::vector<Vec3f> short_normals{{0.0F, 0.0F, 1.0F}};
  auto mismatched = OwnedSurface::create(points, std::move(short_normals), {}, std::nullopt,
                                         LengthUnit::millimeter, frame());
  passed &= expect(!mismatched, "normal storage length must match point storage");

  const float nan = std::numeric_limits<float>::quiet_NaN();
  std::vector<Vec3f> masked_points{{1.0F, 2.0F, 3.0F}, {nan, nan, nan}};
  auto masked = OwnedSurface::create(std::move(masked_points), {}, {1U, 0U}, std::nullopt,
                                     LengthUnit::millimeter, frame());
  passed &= expect(static_cast<bool>(masked), "masked non-finite points must be retained");
  if (masked) {
    const auto view = masked.value().view();
    passed &= expect(view.size() == 2U && view.storage_size() == 2U,
                     "unorganized view size must match point storage");
    passed &= expect(view.valid()[1] == 0U && std::isnan(view.points()[1].x),
                     "OwnedSurface must preserve the original validity mask and invalid sample");
  }

  std::vector<Vec3f> unmasked_nonfinite{{nan, 0.0F, 0.0F}};
  auto invalid_point = OwnedSurface::create(std::move(unmasked_nonfinite), {}, {}, std::nullopt,
                                            LengthUnit::millimeter, frame());
  passed &= expect(!invalid_point, "unmasked non-finite points must be rejected");

  std::vector<Vec3f> padded_grid{{0.0F, 0.0F, 0.0F},
                                 {1.0F, 0.0F, 0.0F},
                                 {nan, nan, nan},
                                 {0.0F, 1.0F, 0.0F},
                                 {1.0F, 1.0F, 0.0F}};
  auto organized = OwnedSurface::create(std::move(padded_grid), {}, {}, GridTopology{2U, 2U, 3U},
                                        LengthUnit::meter, frame("scanner"));
  passed &= expect(static_cast<bool>(organized), "organized storage may contain row padding");
  if (organized) {
    const auto view = organized.value().view();
    passed &= expect(view.size() == 4U && view.storage_size() == 5U,
                     "organized logical size must exclude row padding");
    passed &= expect(view.grid() == GridTopology{2U, 2U, 3U} && view.unit() == LengthUnit::meter &&
                         view.frame().value() == "scanner",
                     "view must preserve topology, unit, and borrowed frame metadata");
  }

  auto bad_grid = SurfaceView::create(points, {}, {}, GridTopology{2U, 2U, 2U},
                                      LengthUnit::millimeter, frame());
  passed &= expect(!bad_grid, "grid dimensions must match storage size");

  auto bad_mask =
      OwnedSurface::create(points, {}, {1U, 2U}, std::nullopt, LengthUnit::millimeter, frame());
  passed &= expect(!bad_mask, "validity mask elements other than 0 and 1 must be rejected");

  return passed ? 0 : 1;
}
