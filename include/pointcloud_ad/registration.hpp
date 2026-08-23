#pragma once

#include <cmath>
#include <cstdint>
#include <pointcloud_ad/geometry.hpp>
#include <pointcloud_ad/result.hpp>
#include <pointcloud_ad/surface.hpp>
#include <string>
#include <utility>

namespace pointcloud_ad {

namespace detail {

[[nodiscard]] inline Error registration_error(std::string field, std::string reason) {
  return Error{ErrorCode::invalid_input,
               PipelineStage::registration,
               "invalid registration contract",
               {{"field", std::move(field)}, {"reason", std::move(reason)}}};
}

[[nodiscard]] inline bool finite_nonnegative(double value) noexcept {
  return std::isfinite(value) && value >= 0.0;
}

} // namespace detail

// Solver parameters supplied with every registration request. Values are already validated
// millimetre/radian quantities mirroring the validated registration configuration. The solver
// treats the maximum correspondence distance as the nearest-neighbour search bound and the
// remaining fields as robust-loss and convergence controls.
struct RegistrationParameters final {
  std::uint32_t max_iterations{};
  double max_correspondence_distance_mm{};
  double huber_delta_mm{};
  double translation_epsilon_mm{};
  double rotation_epsilon_rad{};
  double residual_epsilon_mm{};

  friend bool operator==(const RegistrationParameters&, const RegistrationParameters&) = default;
};

namespace detail {

[[nodiscard]] inline Result<RegistrationParameters>
validate_registration_parameters(RegistrationParameters parameters) {
  if (parameters.max_iterations == 0U || parameters.max_iterations > 1000U) {
    return Result<RegistrationParameters>::failure(
        registration_error("parameters.max_iterations", "must be in [1, 1000]"));
  }
  if (!std::isfinite(parameters.max_correspondence_distance_mm) ||
      parameters.max_correspondence_distance_mm <= 0.0) {
    return Result<RegistrationParameters>::failure(registration_error(
        "parameters.max_correspondence_distance_mm", "must be finite and positive"));
  }
  if (!std::isfinite(parameters.huber_delta_mm) || parameters.huber_delta_mm <= 0.0) {
    return Result<RegistrationParameters>::failure(
        registration_error("parameters.huber_delta_mm", "must be finite and positive"));
  }
  if (!std::isfinite(parameters.translation_epsilon_mm) ||
      parameters.translation_epsilon_mm <= 0.0) {
    return Result<RegistrationParameters>::failure(
        registration_error("parameters.translation_epsilon_mm", "must be finite and positive"));
  }
  if (!std::isfinite(parameters.rotation_epsilon_rad) || parameters.rotation_epsilon_rad <= 0.0) {
    return Result<RegistrationParameters>::failure(
        registration_error("parameters.rotation_epsilon_rad", "must be finite and positive"));
  }
  if (!std::isfinite(parameters.residual_epsilon_mm) || parameters.residual_epsilon_mm <= 0.0) {
    return Result<RegistrationParameters>::failure(
        registration_error("parameters.residual_epsilon_mm", "must be finite and positive"));
  }
  return Result<RegistrationParameters>::success(std::move(parameters));
}

} // namespace detail

// Borrows the two normalized millimetre surfaces and the scan-to-reference initial transform that
// a registration run consumes. The caller keeps the surface storage alive for the full lifetime of
// the input. Validation enforces non-empty surfaces, millimetre units, scan-to-reference transform
// frame direction, and finite positive parameters.
class RegistrationInput final {
public:
  [[nodiscard]] static Result<RegistrationInput> create(SurfaceView reference, SurfaceView scan,
                                                        RigidTransform initial_transform,
                                                        RegistrationParameters parameters) {
    if (reference.size() == 0U) {
      return Result<RegistrationInput>::failure(
          detail::registration_error("reference", "must contain at least one point"));
    }
    if (scan.size() == 0U) {
      return Result<RegistrationInput>::failure(
          detail::registration_error("scan", "must contain at least one point"));
    }
    if (reference.unit() != LengthUnit::millimeter) {
      return Result<RegistrationInput>::failure(
          detail::registration_error("reference.unit", "must be normalized to millimetres"));
    }
    if (scan.unit() != LengthUnit::millimeter) {
      return Result<RegistrationInput>::failure(
          detail::registration_error("scan.unit", "must be normalized to millimetres"));
    }
    if (initial_transform.source_frame() != scan.frame() ||
        initial_transform.target_frame() != reference.frame()) {
      return Result<RegistrationInput>::failure(detail::registration_error(
          "initial_transform", "frame direction must match scan -> reference"));
    }
    auto validated = detail::validate_registration_parameters(parameters);
    if (!validated) {
      return Result<RegistrationInput>::failure(std::move(validated).error());
    }
    return Result<RegistrationInput>::success(
        RegistrationInput(reference, scan, std::move(initial_transform), validated.value()));
  }

  [[nodiscard]] SurfaceView reference() const noexcept {
    return reference_;
  }
  [[nodiscard]] SurfaceView scan() const noexcept {
    return scan_;
  }
  [[nodiscard]] const RigidTransform& initial_transform() const noexcept {
    return initial_transform_;
  }
  [[nodiscard]] const RegistrationParameters& parameters() const noexcept {
    return parameters_;
  }

private:
  RegistrationInput(SurfaceView reference, SurfaceView scan, RigidTransform initial_transform,
                    RegistrationParameters parameters) noexcept
      : reference_(reference), scan_(scan), initial_transform_(std::move(initial_transform)),
        parameters_(parameters) {}

  SurfaceView reference_;
  SurfaceView scan_;
  RigidTransform initial_transform_;
  RegistrationParameters parameters_;
};

// Solver-level termination status. This is deliberately neutral: it reports whether the solver's
// own convergence criteria were met and never encodes an inspection verdict or a process exit code.
enum class RegistrationConvergence : std::uint8_t {
  converged,
  not_converged,
  degenerate_input,
};

namespace detail {

[[nodiscard]] inline bool is_valid_convergence(RegistrationConvergence value) noexcept {
  switch (value) {
  case RegistrationConvergence::converged:
  case RegistrationConvergence::not_converged:
  case RegistrationConvergence::degenerate_input:
    return true;
  }
  return false;
}

} // namespace detail

// Owns the quantitative outcome of a registration run. The final transform is scan-to-reference and
// carries its own validated frames. Fitness and the inlier root-mean-square residual describe the
// accepted correspondences in millimetres; the translation and rotation deltas are measured
// relative to the initial transform so a downstream quality gate can bound movement without
// re-deriving it.
class RegistrationMetrics final {
public:
  [[nodiscard]] static Result<RegistrationMetrics>
  create(RigidTransform final_transform, RegistrationConvergence convergence,
         std::uint32_t iterations, std::uint64_t valid_pairs, double fitness, double inlier_rmse_mm,
         double translation_delta_mm, double rotation_delta_deg) {
    if (!detail::is_valid_convergence(convergence)) {
      return Result<RegistrationMetrics>::failure(
          detail::registration_error("convergence", "enumerator is not supported"));
    }
    if (!std::isfinite(fitness) || fitness < 0.0 || fitness > 1.0) {
      return Result<RegistrationMetrics>::failure(
          detail::registration_error("fitness", "must be in [0, 1]"));
    }
    if (!detail::finite_nonnegative(inlier_rmse_mm)) {
      return Result<RegistrationMetrics>::failure(
          detail::registration_error("inlier_rmse_mm", "must be finite and non-negative"));
    }
    if (!detail::finite_nonnegative(translation_delta_mm)) {
      return Result<RegistrationMetrics>::failure(
          detail::registration_error("translation_delta_mm", "must be finite and non-negative"));
    }
    if (!std::isfinite(rotation_delta_deg) || rotation_delta_deg < 0.0 ||
        rotation_delta_deg > 180.0) {
      return Result<RegistrationMetrics>::failure(
          detail::registration_error("rotation_delta_deg", "must be in [0, 180]"));
    }
    return Result<RegistrationMetrics>::success(
        RegistrationMetrics(std::move(final_transform), convergence, iterations, valid_pairs,
                            fitness, inlier_rmse_mm, translation_delta_mm, rotation_delta_deg));
  }

  [[nodiscard]] const RigidTransform& final_transform() const noexcept {
    return final_transform_;
  }
  [[nodiscard]] RegistrationConvergence convergence() const noexcept {
    return convergence_;
  }
  [[nodiscard]] std::uint32_t iterations() const noexcept {
    return iterations_;
  }
  [[nodiscard]] std::uint64_t valid_pairs() const noexcept {
    return valid_pairs_;
  }
  [[nodiscard]] double fitness() const noexcept {
    return fitness_;
  }
  [[nodiscard]] double inlier_rmse_mm() const noexcept {
    return inlier_rmse_mm_;
  }
  [[nodiscard]] double translation_delta_mm() const noexcept {
    return translation_delta_mm_;
  }
  [[nodiscard]] double rotation_delta_deg() const noexcept {
    return rotation_delta_deg_;
  }

private:
  RegistrationMetrics(RigidTransform final_transform, RegistrationConvergence convergence,
                      std::uint32_t iterations, std::uint64_t valid_pairs, double fitness,
                      double inlier_rmse_mm, double translation_delta_mm,
                      double rotation_delta_deg) noexcept
      : final_transform_(std::move(final_transform)), convergence_(convergence),
        iterations_(iterations), valid_pairs_(valid_pairs), fitness_(fitness),
        inlier_rmse_mm_(inlier_rmse_mm), translation_delta_mm_(translation_delta_mm),
        rotation_delta_deg_(rotation_delta_deg) {}

  RigidTransform final_transform_;
  RegistrationConvergence convergence_;
  std::uint32_t iterations_;
  std::uint64_t valid_pairs_;
  double fitness_;
  double inlier_rmse_mm_;
  double translation_delta_mm_;
  double rotation_delta_deg_;
};

} // namespace pointcloud_ad
