#include "registration_gate.hpp"

#include <exception>
#include <string>
#include <utility>

namespace pointcloud_ad::registration {
namespace {

[[nodiscard]] RegistrationGateResult rejected(GateFailure failure, std::string message) {
  return RegistrationGateResult{false, failure, std::move(message)};
}

} // namespace

Result<RegistrationGateResult> evaluate_gate(const RegistrationMetrics& metrics,
                                             const ValidatedRegistrationGateConfig& gate) noexcept {
  try {
    // The gate order mirrors the specification: convergence, pair count, overlap, residual, then
    // the translation and rotation priors. The first failing item is reported deterministically.
    if (metrics.convergence() != RegistrationConvergence::converged) {
      return Result<RegistrationGateResult>::success(
          rejected(GateFailure::not_converged, "registration did not converge"));
    }
    if (metrics.valid_pairs() < gate.min_valid_pairs) {
      return Result<RegistrationGateResult>::success(rejected(
          GateFailure::insufficient_pairs, "valid correspondence count is below the minimum"));
    }
    if (metrics.fitness() < gate.min_overlap_ratio) {
      return Result<RegistrationGateResult>::success(
          rejected(GateFailure::insufficient_overlap, "overlap ratio is below the minimum"));
    }
    if (metrics.inlier_rmse_mm() > gate.max_inlier_rmse_mm) {
      return Result<RegistrationGateResult>::success(
          rejected(GateFailure::excessive_residual, "inlier RMS residual exceeds the maximum"));
    }
    if (metrics.translation_delta_mm() > gate.max_translation_from_initial_mm) {
      return Result<RegistrationGateResult>::success(rejected(
          GateFailure::translation_prior, "translation from the initial pose exceeds the bound"));
    }
    if (metrics.rotation_delta_deg() > gate.max_rotation_from_initial_deg) {
      return Result<RegistrationGateResult>::success(rejected(
          GateFailure::rotation_prior, "rotation from the initial pose exceeds the bound"));
    }
    return Result<RegistrationGateResult>::success(
        RegistrationGateResult{true, GateFailure::none, {}});
  } catch (const std::exception& exception) {
    return Result<RegistrationGateResult>::failure(
        Error{ErrorCode::internal_error,
              PipelineStage::registration_gate,
              "unexpected exception while evaluating the registration gate",
              {{"exception", exception.what()}}});
  } catch (...) {
    return Result<RegistrationGateResult>::failure(
        Error{ErrorCode::internal_error,
              PipelineStage::registration_gate,
              "unknown exception while evaluating the registration gate",
              {}});
  }
}

} // namespace pointcloud_ad::registration
