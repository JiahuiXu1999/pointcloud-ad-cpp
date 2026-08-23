#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <pointcloud_ad/geometry.hpp>
#include <pointcloud_ad/registration.hpp>
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

bool near(double actual, double expected, double tolerance = 1.0e-12) {
  return std::abs(actual - expected) <= tolerance;
}

constexpr std::array<double, 16> kIdentity{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                           0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};

pointcloud_ad::RegistrationParameters valid_parameters() {
  pointcloud_ad::RegistrationParameters parameters;
  parameters.max_iterations = 60U;
  parameters.max_correspondence_distance_mm = 1.0;
  parameters.huber_delta_mm = 0.3;
  parameters.translation_epsilon_mm = 1.0e-4;
  parameters.rotation_epsilon_rad = 1.0e-5;
  parameters.residual_epsilon_mm = 1.0e-5;
  return parameters;
}

} // namespace

int main() {
  using pointcloud_ad::ErrorCode;
  using pointcloud_ad::FrameId;
  using pointcloud_ad::LengthUnit;
  using pointcloud_ad::RegistrationConvergence;
  using pointcloud_ad::RegistrationInput;
  using pointcloud_ad::RegistrationMetrics;
  using pointcloud_ad::RegistrationParameters;
  using pointcloud_ad::RigidTransform;
  using pointcloud_ad::SurfaceView;
  using pointcloud_ad::Vec3f;

  bool passed = true;

  auto reference_frame = FrameId::create("fixture");
  auto scan_frame = FrameId::create("scanner");
  passed &= expect(static_cast<bool>(reference_frame) && static_cast<bool>(scan_frame),
                   "frame ids must be accepted");

  std::vector<Vec3f> reference_points{
      {0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, {1.0F, 1.0F, 0.0F}};
  std::vector<Vec3f> scan_points{{0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}};

  auto reference = SurfaceView::create(reference_points, {}, {}, std::nullopt,
                                       LengthUnit::millimeter, reference_frame.value());
  auto scan = SurfaceView::create(scan_points, {}, {}, std::nullopt, LengthUnit::millimeter,
                                  scan_frame.value());
  auto reference_metres = SurfaceView::create(reference_points, {}, {}, std::nullopt,
                                              LengthUnit::meter, reference_frame.value());
  passed &= expect(static_cast<bool>(reference) && static_cast<bool>(scan) &&
                       static_cast<bool>(reference_metres),
                   "surface views must be created for the contract tests");

  auto scan_to_reference =
      RigidTransform::create(kIdentity, scan_frame.value(), reference_frame.value());
  auto reversed = RigidTransform::create(kIdentity, reference_frame.value(), scan_frame.value());
  passed &= expect(static_cast<bool>(scan_to_reference) && static_cast<bool>(reversed),
                   "rigid transforms must be created for the contract tests");

  // --- Input contract: acceptance and accessors ---
  auto input = RegistrationInput::create(reference.value(), scan.value(), scan_to_reference.value(),
                                         valid_parameters());
  passed &= expect(static_cast<bool>(input), "a valid registration input must be accepted");
  if (input) {
    passed &= expect(input.value().reference().size() == 4U && input.value().scan().size() == 3U,
                     "registration input must retain borrowed surface sizes");
    passed &= expect(input.value().reference().frame().value() == "fixture" &&
                         input.value().scan().frame().value() == "scanner",
                     "registration input must retain surface frames");
    passed &= expect(input.value().initial_transform().source_frame().value() == "scanner" &&
                         input.value().initial_transform().target_frame().value() == "fixture",
                     "registration input must retain the scan-to-reference transform");
    passed &= expect(input.value().parameters().max_iterations == 60U &&
                         near(input.value().parameters().max_correspondence_distance_mm, 1.0),
                     "registration input must retain validated parameters");
  }

  // --- Input contract: rejected units ---
  auto metres_input = RegistrationInput::create(reference_metres.value(), scan.value(),
                                                scan_to_reference.value(), valid_parameters());
  passed &= expect(!metres_input && metres_input.error().code == ErrorCode::invalid_input &&
                       metres_input.error().stage == pointcloud_ad::PipelineStage::registration,
                   "a non-millimetre reference must be rejected with a registration-stage error");

  // --- Input contract: rejected transform frame direction ---
  auto reversed_input = RegistrationInput::create(reference.value(), scan.value(), reversed.value(),
                                                  valid_parameters());
  passed &= expect(!reversed_input, "a reversed transform direction must be rejected");

  // --- Input contract: rejected parameters ---
  auto zero_iterations = valid_parameters();
  zero_iterations.max_iterations = 0U;
  auto zero_iterations_input = RegistrationInput::create(
      reference.value(), scan.value(), scan_to_reference.value(), zero_iterations);
  passed &= expect(!zero_iterations_input && zero_iterations_input.error().context.at("field") ==
                                                 "parameters.max_iterations",
                   "zero iterations must be rejected with a field-level error");

  auto negative_distance = valid_parameters();
  negative_distance.max_correspondence_distance_mm = -1.0;
  passed &= expect(!RegistrationInput::create(reference.value(), scan.value(),
                                              scan_to_reference.value(), negative_distance),
                   "a negative correspondence distance must be rejected");

  auto nan_huber = valid_parameters();
  nan_huber.huber_delta_mm = std::numeric_limits<double>::quiet_NaN();
  passed &= expect(!RegistrationInput::create(reference.value(), scan.value(),
                                              scan_to_reference.value(), nan_huber),
                   "a non-finite Huber delta must be rejected");

  // --- Metrics contract: acceptance and accessors ---
  auto metrics =
      RegistrationMetrics::create(scan_to_reference.value(), RegistrationConvergence::converged,
                                  12U, 500U, 0.85, 0.12, 1.5, 2.0);
  passed &= expect(static_cast<bool>(metrics), "valid registration metrics must be accepted");
  if (metrics) {
    passed &= expect(metrics.value().final_transform().source_frame().value() == "scanner" &&
                         metrics.value().final_transform().target_frame().value() == "fixture",
                     "metrics must retain the final scan-to-reference transform frames");
    passed &=
        expect(metrics.value().convergence() == RegistrationConvergence::converged &&
                   metrics.value().iterations() == 12U && metrics.value().valid_pairs() == 500U,
               "metrics must retain convergence, iteration, and pair counts");
    passed &= expect(near(metrics.value().fitness(), 0.85) &&
                         near(metrics.value().inlier_rmse_mm(), 0.12) &&
                         near(metrics.value().translation_delta_mm(), 1.5) &&
                         near(metrics.value().rotation_delta_deg(), 2.0),
                     "metrics must retain quantitative fitness values");
  }

  // --- Metrics contract: rejected convergence enumerator ---
  auto invalid_convergence = RegistrationMetrics::create(scan_to_reference.value(),
                                                         static_cast<RegistrationConvergence>(255),
                                                         1U, 1U, 0.5, 0.0, 0.0, 0.0);
  passed &= expect(!invalid_convergence, "an unknown convergence enumerator must be rejected");

  // --- Metrics contract: rejected fitness range ---
  passed &= expect(!RegistrationMetrics::create(scan_to_reference.value(),
                                                RegistrationConvergence::converged, 1U, 1U, -0.1,
                                                0.0, 0.0, 0.0),
                   "a negative fitness must be rejected");
  passed &= expect(!RegistrationMetrics::create(scan_to_reference.value(),
                                                RegistrationConvergence::converged, 1U, 1U, 1.1,
                                                0.0, 0.0, 0.0),
                   "a fitness above one must be rejected");
  passed &= expect(
      !RegistrationMetrics::create(scan_to_reference.value(), RegistrationConvergence::converged,
                                   1U, 1U, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0),
      "a non-finite fitness must be rejected");

  // --- Metrics contract: rejected residuals and deltas ---
  passed &= expect(!RegistrationMetrics::create(scan_to_reference.value(),
                                                RegistrationConvergence::converged, 1U, 1U, 0.5,
                                                -0.01, 0.0, 0.0),
                   "a negative inlier RMSE must be rejected");
  passed &= expect(!RegistrationMetrics::create(scan_to_reference.value(),
                                                RegistrationConvergence::converged, 1U, 1U, 0.5,
                                                0.0, -0.01, 0.0),
                   "a negative translation delta must be rejected");
  passed &= expect(!RegistrationMetrics::create(scan_to_reference.value(),
                                                RegistrationConvergence::converged, 1U, 1U, 0.5,
                                                0.0, 0.0, 181.0),
                   "a rotation delta above 180 degrees must be rejected");
  passed &= expect(!RegistrationMetrics::create(scan_to_reference.value(),
                                                RegistrationConvergence::converged, 1U, 1U, 0.5,
                                                0.0, 0.0, -1.0),
                   "a negative rotation delta must be rejected");

  // --- Metrics contract: accepted boundary values ---
  passed &= expect(static_cast<bool>(RegistrationMetrics::create(
                       scan_to_reference.value(), RegistrationConvergence::degenerate_input, 0U, 0U,
                       0.0, 0.0, 0.0, 0.0)),
                   "boundary fitness, residual, and delta values must be accepted");
  passed &= expect(static_cast<bool>(RegistrationMetrics::create(
                       scan_to_reference.value(), RegistrationConvergence::not_converged, 1000U, 1U,
                       1.0, 0.0, 0.0, 180.0)),
                   "boundary fitness of one and rotation of 180 degrees must be accepted");

  return passed ? 0 : 1;
}
