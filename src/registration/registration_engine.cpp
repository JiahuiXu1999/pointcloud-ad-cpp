#include "backend_policy.hpp"
#include "pcl_registration_backend.hpp"

#include <exception>
#include <limits>
#include <pointcloud_ad/registration_engine.hpp>
#include <stdexcept>
#include <utility>

#ifdef POINTCLOUDAD_WITH_CUDA
#include "cuda_correspondence.hpp"
#endif

namespace pointcloud_ad {

namespace {
Error context_error(std::string message, ErrorCode code = ErrorCode::invalid_input) {
  return Error{code, PipelineStage::registration, std::move(message), {}};
}
} // namespace

struct RegistrationContext::Impl {
  OwnedSurface reference;
  RegistrationParameters parameters;
  std::size_t capacity;
  ComputeBackend actual_backend;
  std::unique_ptr<backends::pcl_backend::PreparedRegistration> prepared;

  Impl(OwnedSurface snapshot, RegistrationParameters options, std::size_t max_scan_points,
       ComputeBackend backend)
      : reference(std::move(snapshot)), parameters(options), capacity(max_scan_points),
        actual_backend(backend), prepared(backends::pcl_backend::prepare_registration(
                                     reference.view(), parameters, capacity, backend)) {}
};

RegistrationContext::RegistrationContext(std::unique_ptr<Impl> impl) noexcept
    : impl_(std::move(impl)) {}
RegistrationContext::RegistrationContext(RegistrationContext&&) noexcept = default;
RegistrationContext& RegistrationContext::operator=(RegistrationContext&&) noexcept = default;
RegistrationContext::~RegistrationContext() = default;

Result<RegistrationContext> RegistrationContext::create(SurfaceView reference,
                                                        RegistrationParameters parameters,
                                                        std::size_t max_scan_points,
                                                        ComputeBackend backend) noexcept {
  try {
    auto options = detail::validate_registration_parameters(parameters);
    if (!options) {
      return Result<RegistrationContext>::failure(std::move(options).error());
    }
    if (reference.unit() != LengthUnit::millimeter || max_scan_points == 0U ||
        max_scan_points > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max())) {
      return Result<RegistrationContext>::failure(
          context_error("reference must use millimetres; scan capacity must be in [1, INT32_MAX]"));
    }
    if (!registration_backend_available(backend)) {
      return Result<RegistrationContext>::failure(context_error(
          "requested registration backend is unavailable", ErrorCode::invalid_argument));
    }
    auto snapshot = OwnedSurface::create(
        std::vector<Vec3f>(reference.points().begin(), reference.points().end()),
        std::vector<Vec3f>(reference.normals().begin(), reference.normals().end()),
        std::vector<std::uint8_t>(reference.valid().begin(), reference.valid().end()),
        reference.grid(), reference.unit(), reference.frame());
    if (!snapshot) {
      return Result<RegistrationContext>::failure(std::move(snapshot).error());
    }
    const auto selected = registration::select_backend(
        backend, reference.size(), max_scan_points,
        backend == ComputeBackend::automatic &&
            registration::gpu_size_eligible(reference.size(), max_scan_points) &&
            registration_backend_available(ComputeBackend::gpu));
    return Result<RegistrationContext>::success(RegistrationContext(std::make_unique<Impl>(
        std::move(snapshot).value(), parameters, max_scan_points, selected)));
  } catch (const std::invalid_argument& exception) {
    return Result<RegistrationContext>::failure(context_error(exception.what()));
  } catch (const std::exception& exception) {
    return Result<RegistrationContext>::failure(
        context_error(exception.what(), ErrorCode::internal_error));
  } catch (...) {
    return Result<RegistrationContext>::failure(
        context_error("unknown context preparation exception", ErrorCode::internal_error));
  }
}

ComputeBackend RegistrationContext::backend() const noexcept {
  return impl_ ? impl_->actual_backend : ComputeBackend::automatic;
}

Result<RegistrationMetrics>
RegistrationContext::align(SurfaceView scan, const RigidTransform& initial_transform) noexcept {
  try {
    if (!impl_) {
      return Result<RegistrationMetrics>::failure(
          context_error("registration context was moved from"));
    }
    if (scan.size() > impl_->capacity) {
      return Result<RegistrationMetrics>::failure(
          context_error("scan exceeds context logical-point capacity"));
    }
    auto input = RegistrationInput::create(impl_->reference.view(), scan, initial_transform,
                                           impl_->parameters);
    if (!input) {
      return Result<RegistrationMetrics>::failure(std::move(input).error());
    }
    // A borrowed view can outlive a caller mutation. Revalidate current scan data before search.
    auto checked = SurfaceView::create(scan.points(), scan.normals(), scan.valid(), scan.grid(),
                                       scan.unit(), scan.frame());
    if (!checked) {
      return Result<RegistrationMetrics>::failure(std::move(checked).error());
    }
    return impl_->prepared->align(scan, initial_transform);
  } catch (const std::exception& exception) {
    return Result<RegistrationMetrics>::failure(
        context_error(exception.what(), ErrorCode::internal_error));
  } catch (...) {
    return Result<RegistrationMetrics>::failure(
        context_error("unknown context alignment exception", ErrorCode::internal_error));
  }
}

bool registration_backend_available(ComputeBackend backend) noexcept {
#ifdef POINTCLOUDAD_WITH_CUDA
  return backend == ComputeBackend::cpu || backend == ComputeBackend::automatic ||
         (backend == ComputeBackend::gpu && backends::cuda_backend::available());
#else
  return backend == ComputeBackend::cpu || backend == ComputeBackend::automatic;
#endif
}

Result<RegistrationMetrics> register_surfaces(SurfaceView reference, SurfaceView scan,
                                              const RigidTransform& initial_transform,
                                              RegistrationParameters parameters,
                                              ComputeBackend backend) noexcept {
  try {
    auto input = RegistrationInput::create(reference, scan, initial_transform, parameters);
    if (!input) {
      return Result<RegistrationMetrics>::failure(std::move(input).error());
    }
    auto context = RegistrationContext::create(reference, parameters, scan.size(), backend);
    if (!context) {
      return Result<RegistrationMetrics>::failure(std::move(context).error());
    }
    return context.value().align(scan, initial_transform);
  } catch (const std::exception& exception) {
    return Result<RegistrationMetrics>::failure(Error{.code = ErrorCode::internal_error,
                                                      .stage = PipelineStage::registration,
                                                      .message = exception.what(),
                                                      .context = {}});
  } catch (...) {
    return Result<RegistrationMetrics>::failure(Error{.code = ErrorCode::internal_error,
                                                      .stage = PipelineStage::registration,
                                                      .message = "unknown registration exception",
                                                      .context = {}});
  }
}

} // namespace pointcloud_ad
