#pragma once

#include <cstdint>
#include <pointcloud_ad/config.hpp>
#include <pointcloud_ad/registration.hpp>
#include <string>

namespace pointcloud_ad::registration {

// Machine-readable reason a registration gate rejected a solve. The enumeration is ordered by the
// specification's gate table so the first failure encountered is stable and deterministic.
enum class GateFailure : std::uint8_t {
  none,                 // Every gate passed.
  not_converged,        // The solver reported a non-converged termination.
  insufficient_pairs,   // valid_pairs fell below the configured minimum.
  insufficient_overlap, // The fitness/overlap ratio fell below the configured minimum.
  excessive_residual,   // The inlier RMS residual exceeded the configured maximum.
  translation_prior,    // Movement from the initial pose exceeded the translation bound.
  rotation_prior,       // Movement from the initial pose exceeded the rotation bound.
};

struct RegistrationGateResult final {
  bool passed{false};
  GateFailure failure{GateFailure::none};
  std::string message;
};

// Applies the registration quality gates without deciding a verdict or process exit code. A
// rejected solve is reported through the value's `passed` flag, not through a Result error, so the
// caller keeps the distinction between a completed-but-untrustworthy solve and a failed call.
[[nodiscard]] Result<RegistrationGateResult>
evaluate_gate(const RegistrationMetrics& metrics,
              const ValidatedRegistrationGateConfig& gate) noexcept;

} // namespace pointcloud_ad::registration
