#pragma once

#include <pointcloud_ad/config.hpp>
#include <pointcloud_ad/inspection_result.hpp>
#include <pointcloud_ad/result.hpp>
#include <string>
#include <string_view>

namespace pointcloud_ad::serialization {

// Serializes a raw inspection configuration to a versioned JSON document. Optional fields are
// emitted as null. Enum values use lowercase snake_case.
[[nodiscard]] Result<std::string> serialize_config(const InspectionConfig& config) noexcept;

// Parses a versioned JSON configuration document back into an `InspectionConfig`. A missing or
// unknown schema major version is rejected.
[[nodiscard]] Result<InspectionConfig> parse_config(std::string_view json) noexcept;

// Serializes a completed inspection report to a versioned JSON document. Non-finite numbers are
// never emitted; unavailable measurements are written as null.
[[nodiscard]] Result<std::string> serialize_result(const InspectionResult& result) noexcept;

} // namespace pointcloud_ad::serialization
