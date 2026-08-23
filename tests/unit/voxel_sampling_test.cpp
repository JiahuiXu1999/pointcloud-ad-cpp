#include "voxel_sampling.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <optional>
#include <pointcloud_ad/geometry.hpp>
#include <pointcloud_ad/status.hpp>
#include <pointcloud_ad/surface.hpp>
#include <span>
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

bool same_indices(std::span<const std::size_t> actual,
                  std::initializer_list<std::size_t> expected) {
  return actual.size() == expected.size() &&
         std::equal(actual.begin(), actual.end(), expected.begin(), expected.end());
}

} // namespace

int main() {
  using pointcloud_ad::ErrorCode;
  using pointcloud_ad::GridTopology;
  using pointcloud_ad::LengthUnit;
  using pointcloud_ad::OwnedSurface;
  using pointcloud_ad::PipelineStage;
  using pointcloud_ad::Vec3f;
  using pointcloud_ad::preprocess::voxel_sample;

  std::vector<Vec3f> points{{-1.2F, 0.0F, 0.0F},   {-0.8F, 0.0F, 0.0F}, {0.1F, 0.0F, 0.0F},
                            {99.0F, 99.0F, 99.0F}, {0.9F, 0.0F, 0.0F},  {1.1F, 0.0F, 0.0F},
                            {1.8F, 0.0F, 0.0F}};
  std::vector<Vec3f> normals(points.size(), Vec3f{0.0F, 0.0F, 2.0F});
  std::vector<std::uint8_t> valid{1U, 1U, 1U, 0U, 1U, 0U, 1U};
  auto surface =
      OwnedSurface::create(std::move(points), std::move(normals), std::move(valid),
                           GridTopology{3U, 2U, 4U}, LengthUnit::millimeter, frame("fixture"));
  auto sampled = voxel_sample(surface.value().view(), 1.0);
  bool passed = expect(static_cast<bool>(sampled), "voxel sampling must accept a masked surface");
  if (sampled) {
    const auto output = sampled.value().surface();
    passed &= expect(output.size() == 4U && !output.grid(),
                     "sampling must emit one unorganized point per occupied voxel");
    passed &= expect(std::abs(output.points()[0].x - -1.2F) < 1.0e-6F &&
                         std::abs(output.points()[1].x - -0.8F) < 1.0e-6F &&
                         std::abs(output.points()[2].x - 0.5F) < 1.0e-6F &&
                         std::abs(output.points()[3].x - 1.8F) < 1.0e-6F,
                     "voxel centroids must use floor keys and lexicographic ordering");
    passed &= expect(output.normals().size() == 4U && output.normals()[2].z == 1.0F,
                     "sampled normals must be accumulated and normalized");
    passed &= expect(output.unit() == LengthUnit::millimeter && output.frame() == frame("fixture"),
                     "sampling must preserve unit and frame metadata");
    passed &=
        expect(same_indices(sampled.value().source_offsets(), {0U, 1U, 2U, 4U, 5U}) &&
                   same_indices(sampled.value().source_indices(), {0U, 1U, 2U, 4U, 6U}),
               "CSR mapping must retain stable source storage indices and skip padding/masks");
    passed &= expect(same_indices(sampled.value().sources_for(2U), {2U, 4U}),
                     "per-voxel source lookup must match CSR membership");
  }

  auto repeated = voxel_sample(surface.value().view(), 1.0);
  passed &= expect(repeated && sampled &&
                       same_indices(repeated.value().source_offsets(), {0U, 1U, 2U, 4U, 5U}) &&
                       same_indices(repeated.value().source_indices(), {0U, 1U, 2U, 4U, 6U}),
                   "voxel ordering and source mapping must be repeatable");
  if (repeated && sampled) {
    for (std::size_t index = 0; index < sampled.value().surface().size(); ++index) {
      passed &= expect(repeated.value().surface().points()[index].x ==
                           sampled.value().surface().points()[index].x,
                       "repeated voxel centroids must be byte-stable");
    }
  }

  auto zero_size = voxel_sample(surface.value().view(), 0.0);
  auto nan_size = voxel_sample(surface.value().view(), std::numeric_limits<double>::quiet_NaN());
  passed &= expect(!zero_size && zero_size.error().code == ErrorCode::invalid_argument &&
                       zero_size.error().stage == PipelineStage::preprocess,
                   "zero voxel size must be rejected");
  passed &= expect(!nan_size && nan_size.error().code == ErrorCode::invalid_argument,
                   "non-finite voxel size must be rejected");

  std::vector<Vec3f> meter_points{{0.0F, 0.0F, 0.0F}};
  auto meters = OwnedSurface::create(std::move(meter_points), {}, {}, std::nullopt,
                                     LengthUnit::meter, frame("fixture"));
  auto wrong_unit = voxel_sample(meters.value().view(), 1.0);
  passed &= expect(!wrong_unit && wrong_unit.error().code == ErrorCode::invalid_input,
                   "sampling must reject non-millimetre input");

  std::vector<Vec3f> masked_points{{0.0F, 0.0F, 0.0F}};
  std::vector<std::uint8_t> masked_valid{0U};
  auto all_masked = OwnedSurface::create(std::move(masked_points), {}, std::move(masked_valid),
                                         std::nullopt, LengthUnit::millimeter, frame("fixture"));
  auto no_samples = voxel_sample(all_masked.value().view(), 1.0);
  passed &= expect(!no_samples && no_samples.error().code == ErrorCode::invalid_input,
                   "sampling must reject an all-masked input");

  std::vector<Vec3f> huge_points{{std::numeric_limits<float>::max(), 0.0F, 0.0F}};
  auto huge = OwnedSurface::create(std::move(huge_points), {}, {}, std::nullopt,
                                   LengthUnit::millimeter, frame("fixture"));
  auto key_overflow = voxel_sample(huge.value().view(), std::numeric_limits<double>::denorm_min());
  passed &= expect(!key_overflow && key_overflow.error().code == ErrorCode::invalid_input,
                   "coordinate-to-key overflow must be rejected");

  return passed ? 0 : 1;
}
