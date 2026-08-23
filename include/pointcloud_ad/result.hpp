#pragma once

#include <pointcloud_ad/status.hpp>
#include <type_traits>
#include <utility>
#include <variant>

namespace pointcloud_ad {

// A small move-only value-or-error contract for public C++20 APIs.
// Calling value() or error() for the inactive alternative violates the stated
// precondition; the accessors themselves never throw.
template <class T> class [[nodiscard]] Result final {
  static_assert(!std::is_reference_v<T>, "Result<T> cannot hold a reference");
  static_assert(!std::is_same_v<std::remove_cv_t<T>, Error>,
                "Result<Error> would make value and error ambiguous");

public:
  explicit Result(T value) : storage_(std::in_place_index<0>, std::move(value)) {}
  explicit Result(Error error) : storage_(std::in_place_index<1>, std::move(error)) {}

  Result(const Result&) = delete;
  Result& operator=(const Result&) = delete;
  Result(Result&&) noexcept(std::is_nothrow_move_constructible_v<T>) = default;
  Result& operator=(Result&&) = delete;
  ~Result() = default;

  [[nodiscard]] static Result success(T value) {
    return Result(std::move(value));
  }
  [[nodiscard]] static Result failure(Error error) {
    return Result(std::move(error));
  }

  [[nodiscard]] bool has_value() const noexcept {
    return storage_.index() == 0U;
  }
  [[nodiscard]] explicit operator bool() const noexcept {
    return has_value();
  }

  // Precondition: has_value() is true.
  [[nodiscard]] T& value() & noexcept {
    return *std::get_if<0>(&storage_);
  }
  [[nodiscard]] const T& value() const& noexcept {
    return *std::get_if<0>(&storage_);
  }
  [[nodiscard]] T&& value() && noexcept {
    return std::move(*std::get_if<0>(&storage_));
  }

  // Precondition: has_value() is false.
  [[nodiscard]] Error& error() & noexcept {
    return *std::get_if<1>(&storage_);
  }
  [[nodiscard]] const Error& error() const& noexcept {
    return *std::get_if<1>(&storage_);
  }
  [[nodiscard]] Error&& error() && noexcept {
    return std::move(*std::get_if<1>(&storage_));
  }

  [[nodiscard]] T value_or(T fallback) && {
    if (has_value()) {
      return std::move(*this).value();
    }
    return fallback;
  }

private:
  std::variant<T, Error> storage_;
};

} // namespace pointcloud_ad
