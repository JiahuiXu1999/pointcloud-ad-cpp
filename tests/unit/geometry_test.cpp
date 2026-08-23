#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <pointcloud_ad/geometry.hpp>
#include <string>
#include <string_view>
#include <utility>

namespace {

bool expect(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}

bool near(double actual, double expected, double tolerance = 1.0e-12) {
  return std::abs(actual - expected) <= tolerance;
}

} // namespace

int main() {
  using pointcloud_ad::ErrorCode;
  using pointcloud_ad::FrameId;
  using pointcloud_ad::LengthUnit;
  using pointcloud_ad::RigidTransform;
  using pointcloud_ad::Vec3d;

  bool passed = true;

  auto empty_frame = FrameId::create("");
  passed &= expect(!empty_frame && empty_frame.error().code == ErrorCode::invalid_input,
                   "an empty frame id must be rejected");
  auto invalid_utf8 = FrameId::create(std::string("\xF0\x28\x8C\x28", 4));
  passed &= expect(!invalid_utf8, "invalid UTF-8 must be rejected");
  auto scan = FrameId::create("scanner");
  auto reference = FrameId::create("fixture");
  passed &= expect(scan && reference, "non-empty UTF-8 frame ids must be accepted");
  passed &= expect(scan.value().value() == "scanner", "frame ids must preserve case and text");

  auto metres = pointcloud_ad::to_millimeters(1.25, LengthUnit::meter);
  auto micrometres = pointcloud_ad::to_millimeters(250.0, LengthUnit::micrometer);
  passed &= expect(metres && near(metres.value(), 1250.0), "metres must convert to millimetres");
  passed &= expect(micrometres && near(micrometres.value(), 0.25),
                   "micrometres must convert to millimetres");
  passed &= expect(!pointcloud_ad::to_millimeters(std::numeric_limits<double>::infinity(),
                                                  LengthUnit::millimeter),
                   "non-finite lengths must be rejected");
  passed &= expect(!pointcloud_ad::millimeters_per_unit(static_cast<LengthUnit>(255)),
                   "unknown unit enumerators must be rejected");

  constexpr std::array<double, 16> scan_to_reference{0.0, -1.0, 0.0, 10.0, 1.0, 0.0, 0.0, 20.0,
                                                     0.0, 0.0,  1.0, 30.0, 0.0, 0.0, 0.0, 1.0};
  auto transform = RigidTransform::create(scan_to_reference, std::move(scan).value(),
                                          std::move(reference).value());
  passed &=
      expect(static_cast<bool>(transform), "a valid right-handed rigid transform must be accepted");
  if (transform) {
    const Vec3d mapped = transform.value().apply(Vec3d{1.0, 2.0, 3.0});
    passed &=
        expect(near(mapped.x, 8.0) && near(mapped.y, 21.0) && near(mapped.z, 33.0),
               "row-major storage must apply to column vectors in source-to-target direction");
    passed &= expect(transform.value().source_frame().value() == "scanner" &&
                         transform.value().target_frame().value() == "fixture",
                     "transform frame direction must be retained");
  }

  auto source = FrameId::create("source");
  auto target = FrameId::create("target");
  auto scaled = scan_to_reference;
  scaled[0] = 2.0;
  passed &=
      expect(!RigidTransform::create(scaled, std::move(source).value(), std::move(target).value()),
             "a scaled rotation must be rejected");

  auto reflected_source = FrameId::create("source");
  auto reflected_target = FrameId::create("target");
  constexpr std::array<double, 16> reflection{-1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                              0.0,  0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  passed &= expect(!RigidTransform::create(reflection, std::move(reflected_source).value(),
                                           std::move(reflected_target).value()),
                   "a left-handed reflection must be rejected");

  return passed ? 0 : 1;
}
