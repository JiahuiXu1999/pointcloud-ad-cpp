#include <iostream>
#include <memory>
#include <pointcloud_ad/result.hpp>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

namespace {

bool expect(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}

} // namespace

int main() {
  using pointcloud_ad::Error;
  using pointcloud_ad::ErrorCode;
  using pointcloud_ad::PipelineStage;
  using pointcloud_ad::Result;

  static_assert(!std::is_copy_constructible_v<Result<int>>);
  static_assert(!std::is_copy_assignable_v<Result<int>>);
  static_assert(std::is_move_constructible_v<Result<int>>);
  static_assert(!std::is_move_assignable_v<Result<int>>);

  bool passed = true;

  auto value = Result<int>::success(42);
  passed &= expect(value.has_value(), "success must contain a value");
  passed &= expect(static_cast<bool>(value), "success must convert to true");
  passed &= expect(value.value() == 42, "value must be preserved");

  Error diagnostic{ErrorCode::invalid_input,
                   PipelineStage::validate,
                   "point array contains an unmasked non-finite value",
                   {{"index", "17"}, {"field", "points"}}};
  auto failure = Result<int>::failure(std::move(diagnostic));
  passed &= expect(!failure.has_value(), "failure must not contain a value");
  passed &= expect(!static_cast<bool>(failure), "failure must convert to false");
  passed &=
      expect(failure.error().code == ErrorCode::invalid_input, "error code must be preserved");
  passed &=
      expect(failure.error().stage == PipelineStage::validate, "pipeline stage must be preserved");
  passed &= expect(failure.error().context.begin()->first == "field",
                   "error context ordering must be deterministic");

  auto fallback = Result<std::string>::failure(
      Error{ErrorCode::io_error, PipelineStage::validate, "missing file", {}});
  passed &= expect(std::move(fallback).value_or("fallback") == "fallback",
                   "value_or must return its fallback for an error");

  auto move_only = Result<std::unique_ptr<int>>::success(std::make_unique<int>(7));
  auto moved = std::move(move_only).value();
  passed &= expect(moved != nullptr && *moved == 7,
                   "Result must transport move-only values without copying");

  return passed ? 0 : 1;
}
