#pragma once

#include <cstdint>
#include <map>
#include <string>

namespace pointcloud_ad {

// Program-level failure categories. These are deliberately independent from
// inspection verdicts and process exit codes.
enum class ErrorCode : std::uint16_t {
  invalid_argument,
  invalid_input,
  unsupported_format,
  io_error,
  registration_failed,
  insufficient_coverage,
  serialization_failed,
  internal_error
};

// The stable pipeline stage at which an error was detected.
enum class PipelineStage : std::uint8_t {
  none,
  validate,
  normalize,
  preprocess,
  registration,
  registration_gate,
  transform,
  compare,
  coverage_gate,
  detect,
  finalize,
  serialization
};

// Owns all diagnostic data. std::map keeps machine-readable context ordered.
struct Error final {
  ErrorCode code{ErrorCode::internal_error};
  PipelineStage stage{PipelineStage::none};
  std::string message;
  std::map<std::string, std::string> context;
};

} // namespace pointcloud_ad
