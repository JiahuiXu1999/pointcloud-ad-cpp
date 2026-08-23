#include "validity.hpp"

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
  using pointcloud_ad::preprocess::apply_validity_mask;
  using pointcloud_ad::preprocess::AxisAlignedRoi;
  using pointcloud_ad::preprocess::InvalidReason;

  const float nan = std::numeric_limits<float>::quiet_NaN();
  std::vector<Vec3f> points{{0.0F, 0.0F, 0.0F},
                            {1.0F, 1.0F, 1.0F},
                            {nan, nan, nan},
                            {3.0F, 3.0F, 3.0F},
                            {2.0F, 2.0F, 2.0F}};
  std::vector<Vec3f> normals{
      {0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 1.0F}, {nan, nan, nan}, {0.0F, 0.0F, 1.0F}, {nan, nan, nan}};
  std::vector<std::uint8_t> input_mask{1U, 0U, 0U, 1U, 0U};
  auto input = OwnedSurface::create(std::move(points), std::move(normals), std::move(input_mask),
                                    std::nullopt, LengthUnit::millimeter, frame("fixture"));
  const AxisAlignedRoi roi{{0.0F, 0.0F, 0.0F}, {2.0F, 2.0F, 2.0F}};
  auto filtered = apply_validity_mask(input.value().view(), roi);
  bool passed = expect(static_cast<bool>(filtered), "validity stage must accept normalized input");
  if (filtered) {
    const auto output = filtered.value().surface();
    const auto reasons = filtered.value().reasons();
    passed &=
        expect(filtered.value().valid_count() == 1U, "only one logical sample must remain valid");
    passed &=
        expect(output.valid().size() == 5U && output.valid()[0] == 1U && output.valid()[1] == 0U &&
                   output.valid()[2] == 0U && output.valid()[3] == 0U && output.valid()[4] == 0U,
               "validity stage must emit an explicit stable mask");
    passed &=
        expect(reasons[0] == InvalidReason::none && reasons[1] == InvalidReason::input_masked &&
                   reasons[2] == InvalidReason::non_finite_point &&
                   reasons[3] == InvalidReason::outside_roi &&
                   reasons[4] == InvalidReason::non_finite_normal,
               "reason precedence must be finite point, finite normal, input mask, then ROI");
    passed &= expect(output.points()[3].x == 3.0F && std::isnan(output.points()[2].x),
                     "masking must not rewrite source geometry");
  }

  std::vector<Vec3f> organized_points{{0.0F, 0.0F, 0.0F},
                                      {2.0F, 0.0F, 0.0F},
                                      {99.0F, 99.0F, 99.0F},
                                      {0.0F, 2.0F, 0.0F},
                                      {2.0F, 2.0F, 0.0F}};
  auto organized =
      OwnedSurface::create(std::move(organized_points), {}, {}, GridTopology{2U, 2U, 3U},
                           LengthUnit::millimeter, frame("fixture"));
  auto organized_result = apply_validity_mask(organized.value().view(), roi);
  passed &= expect(organized_result && organized_result.value().valid_count() == 4U,
                   "inclusive ROI must retain boundary points");
  if (organized_result) {
    const auto output = organized_result.value().surface();
    passed &= expect(output.grid() == std::optional{GridTopology{2U, 2U, 3U}},
                     "validity stage must preserve organized row stride");
    passed &= expect(output.valid()[2] == 0U &&
                         organized_result.value().reasons()[2] == InvalidReason::row_padding,
                     "organized padding must be explicitly masked and traceable");
  }

  auto repeated = apply_validity_mask(input.value().view(), roi);
  passed &= expect(repeated && filtered &&
                       repeated.value().surface().valid().size() ==
                           filtered.value().surface().valid().size(),
                   "repeated validity runs must have identical shape");
  if (repeated && filtered) {
    for (std::size_t index = 0; index < repeated.value().reasons().size(); ++index) {
      passed &= expect(repeated.value().reasons()[index] == filtered.value().reasons()[index] &&
                           repeated.value().surface().valid()[index] ==
                               filtered.value().surface().valid()[index],
                       "repeated validity runs must be byte-stable");
    }
  }

  std::vector<Vec3f> meter_points{{0.0F, 0.0F, 0.0F}};
  auto meters = OwnedSurface::create(std::move(meter_points), {}, {}, std::nullopt,
                                     LengthUnit::meter, frame("fixture"));
  auto wrong_unit = apply_validity_mask(meters.value().view());
  passed &= expect(!wrong_unit && wrong_unit.error().code == ErrorCode::invalid_input &&
                       wrong_unit.error().stage == PipelineStage::preprocess,
                   "preprocessing must reject surfaces that are not normalized to millimetres");

  const AxisAlignedRoi reversed{{2.0F, 0.0F, 0.0F}, {1.0F, 2.0F, 2.0F}};
  const AxisAlignedRoi non_finite{{0.0F, 0.0F, 0.0F}, {nan, 2.0F, 2.0F}};
  auto reversed_result = apply_validity_mask(input.value().view(), reversed);
  auto non_finite_result = apply_validity_mask(input.value().view(), non_finite);
  passed &= expect(!reversed_result && reversed_result.error().code == ErrorCode::invalid_argument,
                   "reversed ROI bounds must be rejected");
  passed &=
      expect(!non_finite_result && non_finite_result.error().code == ErrorCode::invalid_argument,
             "non-finite ROI bounds must be rejected");

  return passed ? 0 : 1;
}
