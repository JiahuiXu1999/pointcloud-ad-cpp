#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <pointcloud_ad/result.hpp>
#include <string>
#include <string_view>
#include <utility>

namespace pointcloud_ad {

enum class LengthUnit : std::uint8_t { millimeter, meter, micrometer };

struct Vec3f final {
  float x{};
  float y{};
  float z{};
};

struct Vec3d final {
  double x{};
  double y{};
  double z{};
};

namespace detail {

[[nodiscard]] inline bool is_valid_utf8(std::string_view text) noexcept {
  std::size_t index = 0;
  while (index < text.size()) {
    const auto lead = static_cast<unsigned char>(text[index]);
    std::size_t continuation_count = 0;
    std::uint32_t code_point = 0;
    if (lead <= 0x7FU) {
      continuation_count = 0;
      code_point = lead;
    } else if (lead >= 0xC2U && lead <= 0xDFU) {
      continuation_count = 1;
      code_point = static_cast<std::uint32_t>(lead & 0x1FU);
    } else if (lead >= 0xE0U && lead <= 0xEFU) {
      continuation_count = 2;
      code_point = static_cast<std::uint32_t>(lead & 0x0FU);
    } else if (lead >= 0xF0U && lead <= 0xF4U) {
      continuation_count = 3;
      code_point = static_cast<std::uint32_t>(lead & 0x07U);
    } else {
      return false;
    }

    if (index + continuation_count >= text.size()) {
      return false;
    }
    for (std::size_t offset = 1; offset <= continuation_count; ++offset) {
      const auto byte = static_cast<unsigned char>(text[index + offset]);
      if ((byte & 0xC0U) != 0x80U) {
        return false;
      }
      code_point = (code_point << 6U) | static_cast<std::uint32_t>(byte & 0x3FU);
    }

    if ((continuation_count == 2 && code_point < 0x800U) ||
        (continuation_count == 3 && code_point < 0x10000U) ||
        (code_point >= 0xD800U && code_point <= 0xDFFFU) || code_point > 0x10FFFFU) {
      return false;
    }
    index += continuation_count + 1;
  }
  return true;
}

[[nodiscard]] inline Error geometry_error(std::string field, std::string reason) {
  return Error{ErrorCode::invalid_input,
               PipelineStage::validate,
               "invalid geometry contract",
               {{"field", std::move(field)}, {"reason", std::move(reason)}}};
}

} // namespace detail

class FrameId final {
public:
  FrameId(const FrameId&) = default;
  FrameId& operator=(const FrameId&) = default;
  FrameId(FrameId&&) noexcept = default;
  FrameId& operator=(FrameId&&) noexcept = default;
  ~FrameId() = default;

  [[nodiscard]] static Result<FrameId> create(std::string value) {
    if (value.empty()) {
      return Result<FrameId>::failure(detail::geometry_error("frame", "must not be empty"));
    }
    if (!detail::is_valid_utf8(value)) {
      return Result<FrameId>::failure(detail::geometry_error("frame", "must be valid UTF-8"));
    }
    return Result<FrameId>::success(FrameId(std::move(value)));
  }

  [[nodiscard]] std::string_view value() const noexcept {
    return value_;
  }

  friend bool operator==(const FrameId&, const FrameId&) = default;

private:
  explicit FrameId(std::string value) : value_(std::move(value)) {}

  std::string value_;
};

using CoordinateFrameId = FrameId;

[[nodiscard]] inline Result<double> millimeters_per_unit(LengthUnit unit) {
  switch (unit) {
  case LengthUnit::millimeter:
    return Result<double>::success(1.0);
  case LengthUnit::meter:
    return Result<double>::success(1000.0);
  case LengthUnit::micrometer:
    return Result<double>::success(0.001);
  }
  return Result<double>::failure(detail::geometry_error("unit", "enumerator is not supported"));
}

[[nodiscard]] inline Result<double> to_millimeters(double value, LengthUnit unit) {
  if (!std::isfinite(value)) {
    return Result<double>::failure(detail::geometry_error("length", "must be a finite number"));
  }
  auto scale = millimeters_per_unit(unit);
  if (!scale) {
    return Result<double>::failure(std::move(scale).error());
  }
  return Result<double>::success(value * scale.value());
}

class RigidTransform final {
public:
  RigidTransform(const RigidTransform&) = default;
  RigidTransform& operator=(const RigidTransform&) = default;
  RigidTransform(RigidTransform&&) noexcept = default;
  RigidTransform& operator=(RigidTransform&&) noexcept = default;
  ~RigidTransform() = default;

  // Storage is row-major, while points are mathematically column vectors. Translation entries
  // and transformed coordinates are millimetres. Direction is source_frame -> target_frame.
  [[nodiscard]] static Result<RigidTransform> create(std::array<double, 16> matrix,
                                                     FrameId source_frame, FrameId target_frame,
                                                     double tolerance = 1.0e-9) {
    if (!std::isfinite(tolerance) || tolerance <= 0.0) {
      return Result<RigidTransform>::failure(
          detail::geometry_error("tolerance", "must be finite and positive"));
    }
    for (double element : matrix) {
      if (!std::isfinite(element)) {
        return Result<RigidTransform>::failure(
            detail::geometry_error("matrix", "all elements must be finite"));
      }
    }
    if (std::abs(matrix[12]) > tolerance || std::abs(matrix[13]) > tolerance ||
        std::abs(matrix[14]) > tolerance || std::abs(matrix[15] - 1.0) > tolerance) {
      return Result<RigidTransform>::failure(
          detail::geometry_error("matrix", "last row must be [0, 0, 0, 1]"));
    }

    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column = 0; column < 3; ++column) {
        double dot = 0.0;
        for (std::size_t index = 0; index < 3; ++index) {
          dot += matrix[index * 4 + row] * matrix[index * 4 + column];
        }
        const double expected = row == column ? 1.0 : 0.0;
        if (std::abs(dot - expected) > tolerance) {
          return Result<RigidTransform>::failure(
              detail::geometry_error("matrix", "rotation must be orthonormal"));
        }
      }
    }

    const double determinant = matrix[0] * (matrix[5] * matrix[10] - matrix[6] * matrix[9]) -
                               matrix[1] * (matrix[4] * matrix[10] - matrix[6] * matrix[8]) +
                               matrix[2] * (matrix[4] * matrix[9] - matrix[5] * matrix[8]);
    if (std::abs(determinant - 1.0) > tolerance) {
      return Result<RigidTransform>::failure(
          detail::geometry_error("matrix", "rotation determinant must be +1"));
    }

    return Result<RigidTransform>::success(
        RigidTransform(std::move(matrix), std::move(source_frame), std::move(target_frame)));
  }

  [[nodiscard]] const std::array<double, 16>& matrix() const noexcept {
    return matrix_;
  }
  [[nodiscard]] const FrameId& source_frame() const noexcept {
    return source_frame_;
  }
  [[nodiscard]] const FrameId& target_frame() const noexcept {
    return target_frame_;
  }

  [[nodiscard]] Vec3d apply(Vec3d point) const noexcept {
    return Vec3d{matrix_[0] * point.x + matrix_[1] * point.y + matrix_[2] * point.z + matrix_[3],
                 matrix_[4] * point.x + matrix_[5] * point.y + matrix_[6] * point.z + matrix_[7],
                 matrix_[8] * point.x + matrix_[9] * point.y + matrix_[10] * point.z + matrix_[11]};
  }

private:
  RigidTransform(std::array<double, 16> matrix, FrameId source_frame, FrameId target_frame)
      : matrix_(std::move(matrix)), source_frame_(std::move(source_frame)),
        target_frame_(std::move(target_frame)) {}

  std::array<double, 16> matrix_;
  FrameId source_frame_;
  FrameId target_frame_;
};

} // namespace pointcloud_ad
