#include "registration_gate.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <pointcloud_ad/config.hpp>
#include <pointcloud_ad/geometry.hpp>
#include <pointcloud_ad/registration.hpp>
#include <string_view>
#include <utility>

namespace {

constexpr std::array<double, 16> kIdentity{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                           0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};

using pointcloud_ad::FrameId;
using pointcloud_ad::RegistrationConvergence;
using pointcloud_ad::RegistrationMetrics;
using pointcloud_ad::RigidTransform;
using pointcloud_ad::ValidatedRegistrationGateConfig;
using pointcloud_ad::registration::evaluate_gate;
using pointcloud_ad::registration::GateFailure;

bool expect(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}

ValidatedRegistrationGateConfig gate_config() {
  return ValidatedRegistrationGateConfig{0.7, 0.2, 500, 5.0, 5.0};
}

RigidTransform make_transform() {
  auto source = FrameId::create("scanner").value();
  auto target = FrameId::create("fixture").value();
  return RigidTransform::create(kIdentity, source, target).value();
}

RegistrationMetrics make_metrics(RigidTransform transform, RegistrationConvergence convergence,
                                 std::uint64_t valid_pairs, double fitness, double inlier_rmse,
                                 double translation_delta, double rotation_delta) {
  return RegistrationMetrics::create(std::move(transform), convergence, 10U, valid_pairs, fitness,
                                     inlier_rmse, translation_delta, rotation_delta)
      .value();
}

} // namespace

int main() {
  bool passed = true;

  // A fully passing solve.
  {
    auto result = evaluate_gate(make_metrics(make_transform(), RegistrationConvergence::converged,
                                             800U, 0.85, 0.12, 1.5, 2.0),
                                gate_config());
    passed &= expect(static_cast<bool>(result) && result.value().passed,
                     "a well-formed solve must pass every gate");
    passed &= expect(result.value().failure == GateFailure::none,
                     "a passing solve must report no failure");
  }

  // Convergence gate.
  {
    auto result =
        evaluate_gate(make_metrics(make_transform(), RegistrationConvergence::not_converged, 800U,
                                   0.85, 0.12, 1.5, 2.0),
                      gate_config());
    passed &= expect(static_cast<bool>(result) && !result.value().passed &&
                         result.value().failure == GateFailure::not_converged,
                     "a non-converged solve must be rejected");
  }

  // Pair-count gate.
  {
    auto result = evaluate_gate(make_metrics(make_transform(), RegistrationConvergence::converged,
                                             100U, 0.85, 0.12, 1.5, 2.0),
                                gate_config());
    passed &= expect(static_cast<bool>(result) && !result.value().passed &&
                         result.value().failure == GateFailure::insufficient_pairs,
                     "too few valid pairs must be rejected");
  }

  // Overlap gate.
  {
    auto result = evaluate_gate(make_metrics(make_transform(), RegistrationConvergence::converged,
                                             800U, 0.5, 0.12, 1.5, 2.0),
                                gate_config());
    passed &= expect(static_cast<bool>(result) && !result.value().passed &&
                         result.value().failure == GateFailure::insufficient_overlap,
                     "insufficient overlap must be rejected");
  }

  // Residual gate.
  {
    auto result = evaluate_gate(make_metrics(make_transform(), RegistrationConvergence::converged,
                                             800U, 0.85, 0.4, 1.5, 2.0),
                                gate_config());
    passed &= expect(static_cast<bool>(result) && !result.value().passed &&
                         result.value().failure == GateFailure::excessive_residual,
                     "an excessive residual must be rejected");
  }

  // Translation-prior gate.
  {
    auto result = evaluate_gate(make_metrics(make_transform(), RegistrationConvergence::converged,
                                             800U, 0.85, 0.12, 8.0, 2.0),
                                gate_config());
    passed &= expect(static_cast<bool>(result) && !result.value().passed &&
                         result.value().failure == GateFailure::translation_prior,
                     "excessive translation from the initial pose must be rejected");
  }

  // Rotation-prior gate.
  {
    auto result = evaluate_gate(make_metrics(make_transform(), RegistrationConvergence::converged,
                                             800U, 0.85, 0.12, 1.5, 12.0),
                                gate_config());
    passed &= expect(static_cast<bool>(result) && !result.value().passed &&
                         result.value().failure == GateFailure::rotation_prior,
                     "excessive rotation from the initial pose must be rejected");
  }

  // A degenerate solve is reported through the convergence gate, never as a Result error.
  {
    auto result =
        evaluate_gate(make_metrics(make_transform(), RegistrationConvergence::degenerate_input, 0U,
                                   0.0, 0.0, 0.0, 0.0),
                      gate_config());
    passed &= expect(static_cast<bool>(result) && !result.value().passed &&
                         result.value().failure == GateFailure::not_converged,
                     "a degenerate solve must be rejected through the convergence gate");
  }

  // Boundary pass: every configured threshold is met exactly.
  {
    auto result = evaluate_gate(make_metrics(make_transform(), RegistrationConvergence::converged,
                                             500U, 0.7, 0.2, 5.0, 5.0),
                                gate_config());
    passed &= expect(static_cast<bool>(result) && result.value().passed,
                     "metrics exactly at every threshold must pass");
  }

  return passed ? 0 : 1;
}
