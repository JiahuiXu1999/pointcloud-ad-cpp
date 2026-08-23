#include "normal_boundary.hpp"

#include <cmath>
#include <cstdint>
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

} // namespace

int main() {
  using pointcloud_ad::ErrorCode;
  using pointcloud_ad::GridTopology;
  using pointcloud_ad::LengthUnit;
  using pointcloud_ad::OwnedSurface;
  using pointcloud_ad::PipelineStage;
  using pointcloud_ad::Vec3f;
  using pointcloud_ad::preprocess::NormalOrientationHint;
  using pointcloud_ad::preprocess::NormalOrientationMode;
  using pointcloud_ad::preprocess::NormalReason;
  using pointcloud_ad::preprocess::prepare_normals_and_boundaries;

  std::vector<Vec3f> plane_points;
  for (int y = -2; y <= 2; ++y) {
    for (int x = -2; x <= 2; ++x) {
      plane_points.push_back(Vec3f{static_cast<float>(x), static_cast<float>(y), 0.0F});
    }
  }
  auto plane = OwnedSurface::create(std::move(plane_points), {}, {}, std::nullopt,
                                    LengthUnit::millimeter, frame("fixture"));
  auto unproven = prepare_normals_and_boundaries(plane.value().view(), 1.5, 5U, 1.5);
  bool passed = expect(static_cast<bool>(unproven), "planar PCA normal estimation must succeed");
  if (unproven) {
    const auto output = unproven.value().surface();
    const auto center = std::size_t{12};
    passed &= expect(output.valid()[center] == 1U &&
                         std::abs(std::abs(output.normals()[center].z) - 1.0F) < 1.0e-5F,
                     "planar center normal must be a finite unit Z vector");
    passed &= expect(output.valid()[0] == 0U &&
                         unproven.value().reasons()[0] == NormalReason::normal_missing,
                     "points below the minimum neighbor count must be marked normal_missing");
    passed &= expect(!unproven.value().orientation_proven() &&
                         unproven.value().orientation_valid()[center] == 0U,
                     "PCA sign without a hint must remain explicitly unproven");
    passed &= expect(unproven.value().boundary()[center] == 0U,
                     "well-supported planar center must not be a radius boundary");
  }

  const NormalOrientationHint upward{NormalOrientationMode::align_direction,
                                     Vec3f{0.0F, 0.0F, 1.0F}};
  auto oriented = prepare_normals_and_boundaries(plane.value().view(), 1.5, 5U, 1.5, upward);
  passed &= expect(oriented && oriented.value().orientation_proven(),
                   "direction hint must prove every available normal orientation");
  if (oriented) {
    passed &= expect(oriented.value().surface().normals()[12].z > 0.99F &&
                         oriented.value().orientation_valid()[12] == 1U,
                     "direction hint must flip planar normals consistently");
  }

  std::vector<Vec3f> grid_points;
  std::vector<Vec3f> grid_normals;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      const float z = row == 1 && column == 2 ? 5.0F : 0.0F;
      grid_points.push_back(Vec3f{static_cast<float>(column), static_cast<float>(row), z});
      grid_normals.push_back(Vec3f{0.0F, 0.0F, -2.0F});
    }
  }
  auto grid =
      OwnedSurface::create(std::move(grid_points), std::move(grid_normals), {},
                           GridTopology{3U, 3U, 3U}, LengthUnit::millimeter, frame("fixture"));
  auto grid_result = prepare_normals_and_boundaries(grid.value().view(), 1.5, 3U, 1.5, upward);
  passed &= expect(grid_result && grid_result.value().surface().normals()[4].z == 1.0F,
                   "existing normals must be normalized and oriented");
  if (grid_result) {
    passed &=
        expect(grid_result.value().boundary()[0] == 1U && grid_result.value().boundary()[4] == 1U,
               "grid edges and depth discontinuities must be boundary flagged");
    passed &= expect(grid_result.value().reasons()[4] == NormalReason::none,
                     "boundary flags must not invalidate otherwise usable normals");
  }

  std::vector<Vec3f> view_points{{0.0F, 0.0F, 0.0F}};
  std::vector<Vec3f> view_normals{{0.0F, 0.0F, -3.0F}};
  auto view_surface = OwnedSurface::create(std::move(view_points), std::move(view_normals), {},
                                           std::nullopt, LengthUnit::millimeter, frame("scanner"));
  const NormalOrientationHint viewpoint{NormalOrientationMode::toward_viewpoint,
                                        Vec3f{0.0F, 0.0F, 10.0F}};
  auto view_result =
      prepare_normals_and_boundaries(view_surface.value().view(), 1.0, 3U, 1.0, viewpoint);
  passed &= expect(view_result && view_result.value().orientation_proven() &&
                       view_result.value().surface().normals()[0].z == 1.0F,
                   "viewpoint evidence must orient existing normals toward the viewpoint");

  auto repeated = prepare_normals_and_boundaries(plane.value().view(), 1.5, 5U, 1.5, upward);
  passed &=
      expect(repeated && oriented &&
                 repeated.value().surface().valid()[12] == oriented.value().surface().valid()[12] &&
                 repeated.value().surface().normals()[12].z ==
                     oriented.value().surface().normals()[12].z &&
                 repeated.value().boundary()[12] == oriented.value().boundary()[12],
             "normal, orientation, and boundary outputs must be repeatable");

  auto invalid_radius = prepare_normals_and_boundaries(plane.value().view(), 0.0, 5U, 1.5);
  auto invalid_neighbors = prepare_normals_and_boundaries(plane.value().view(), 1.5, 2U, 1.5);
  const NormalOrientationHint zero_direction{NormalOrientationMode::align_direction, Vec3f{}};
  auto invalid_hint =
      prepare_normals_and_boundaries(plane.value().view(), 1.5, 5U, 1.5, zero_direction);
  passed &= expect(!invalid_radius && invalid_radius.error().code == ErrorCode::invalid_argument &&
                       invalid_radius.error().stage == PipelineStage::preprocess,
                   "invalid normal radius must be rejected");
  passed &=
      expect(!invalid_neighbors && invalid_neighbors.error().code == ErrorCode::invalid_argument,
             "fewer than three requested neighbors must be rejected");
  passed &= expect(!invalid_hint && invalid_hint.error().code == ErrorCode::invalid_argument,
                   "zero direction hints must be rejected");

  std::vector<Vec3f> meter_points{{0.0F, 0.0F, 0.0F}};
  std::vector<Vec3f> meter_normals{{0.0F, 0.0F, 1.0F}};
  auto meters = OwnedSurface::create(std::move(meter_points), std::move(meter_normals), {},
                                     std::nullopt, LengthUnit::meter, frame("fixture"));
  auto wrong_unit = prepare_normals_and_boundaries(meters.value().view(), 1.0, 3U, 1.0);
  passed &= expect(!wrong_unit && wrong_unit.error().code == ErrorCode::invalid_input,
                   "normal preprocessing must reject non-millimetre input");

  return passed ? 0 : 1;
}
