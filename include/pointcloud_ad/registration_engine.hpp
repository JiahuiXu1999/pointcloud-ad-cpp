#pragma once

#include <cstddef>
#include <memory>
#include <pointcloud_ad/export.hpp>
#include <pointcloud_ad/registration.hpp>

namespace pointcloud_ad {

// Owns a snapshot of a millimetre reference and prepared search/covariance resources. The
// reference, parameters, backend and maximum logical scan size are immutable. Create a new
// context to change them. CPU/automatic select CPU; explicit GPU requires an available device.
// Calls on one context (including moves/destruction) must be serialized by the caller. Separate
// contexts are independent. GPU align must use the CUDA device current at creation.
class RegistrationContext final {
public:
  RegistrationContext(const RegistrationContext&) = delete;
  RegistrationContext& operator=(const RegistrationContext&) = delete;
  POINTCLOUD_AD_EXPORT RegistrationContext(RegistrationContext&&) noexcept;
  POINTCLOUD_AD_EXPORT RegistrationContext& operator=(RegistrationContext&&) noexcept;
  POINTCLOUD_AD_EXPORT ~RegistrationContext();

  // Copies reference data during this call; caller storage/frame may then be released or changed.
  // max_scan_points bounds logical points (including masked points), must be in [1, INT32_MAX],
  // and reserves iteration workspace (including GPU query storage) once. Invalid
  // inputs/allocation/device failures return errors.
  [[nodiscard]] static POINTCLOUD_AD_EXPORT Result<RegistrationContext>
  create(SurfaceView reference, RegistrationParameters parameters, std::size_t max_scan_points,
         ComputeBackend backend = ComputeBackend::cpu) noexcept;

  // Borrows scan only for this synchronous call. Initial/output pose maps scan to reference.
  // Over-capacity, wrong frames/units and moved-from contexts return errors. Rejected input does
  // not invalidate the context. GPU runtime failures may require recreating a device/context.
  // Scan extraction and GICP scan covariance are computed per call; reference data is reused.
  [[nodiscard]] POINTCLOUD_AD_EXPORT Result<RegistrationMetrics>
  align(SurfaceView scan, const RigidTransform& initial_transform) noexcept;

private:
  struct Impl;
  explicit RegistrationContext(std::unique_ptr<Impl> impl) noexcept;
  std::unique_ptr<Impl> impl_;
};

// Synchronous registration. Borrows normalized millimetre surfaces for the duration of the call.
// The initial and returned transforms map scan to reference. Explicit GPU never falls back;
// automatic currently selects CPU until performance crossover thresholds have been established.
// GPU accelerates correspondence search; preparation and ordered solving remain on CPU.
// Failures (including unavailable devices) are owned Result errors; no exception crosses this API.
[[nodiscard]] POINTCLOUD_AD_EXPORT Result<RegistrationMetrics>
register_surfaces(SurfaceView reference, SurfaceView scan, const RigidTransform& initial_transform,
                  RegistrationParameters parameters,
                  ComputeBackend backend = ComputeBackend::cpu) noexcept;

// Reports whether this binary can currently execute on the requested backend. GPU requires both
// an enabled build and an accessible device. Does not alter the requested execution policy.
[[nodiscard]] POINTCLOUD_AD_EXPORT bool
registration_backend_available(ComputeBackend backend) noexcept;

} // namespace pointcloud_ad
