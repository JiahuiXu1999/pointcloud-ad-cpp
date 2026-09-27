#pragma once

#include <algorithm>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace pointcloud_ad::registration {

// Context-owned synchronous executor. One caller at a time; callbacks must not call run again.
// Blocks and exception selection are independent of scheduling and worker count.
class CpuExecutor final {
public:
  static constexpr std::size_t block_size = 256;
  static std::size_t blocks(std::size_t count) noexcept {
    return count / block_size + (count % block_size != 0 ? 1U : 0U);
  }
  CpuExecutor(std::uint32_t budget, std::size_t capacity)
      : count_(std::min({static_cast<std::size_t>(budget), std::size_t{64},
                         std::max(std::size_t{1}, blocks(capacity))})) {
    if (budget == 0) {
      throw std::invalid_argument("thread budget must be positive");
    }
    try {
      for (std::size_t worker = 1; worker < count_; ++worker) {
        threads_.emplace_back([this, worker] { worker_loop(worker); });
      }
    } catch (...) {
      stop();
      throw;
    }
  }
  ~CpuExecutor() {
    stop();
  }
  CpuExecutor(const CpuExecutor&) = delete;
  CpuExecutor& operator=(const CpuExecutor&) = delete;
  std::size_t concurrency() const noexcept {
    return count_;
  }

  template <typename Function> void run(std::size_t points, Function&& function) {
    const auto total = blocks(points);
    if (total == 0) {
      return;
    }
    // Prepare throwing allocations before notifying any worker.
    std::function<void(std::size_t, std::size_t)> job(std::forward<Function>(function));
    errors_.assign(total, nullptr);
    {
      std::lock_guard lock(mutex_);
      job_ = std::move(job);
      blocks_ = total;
      pending_ = threads_.size();
      ++generation_;
    }
    ready_.notify_all();
    execute(0);
    {
      std::unique_lock lock(mutex_);
      finished_.wait(lock, [this] { return pending_ == 0; });
      job_ = {};
    }
    for (const auto& error : errors_) {
      if (error) {
        std::rethrow_exception(error);
      }
    }
  }

private:
  void execute(std::size_t worker) noexcept {
    for (std::size_t block = worker; block < blocks_; block += count_) {
      try {
        job_(block, worker);
      } catch (...) {
        errors_[block] = std::current_exception();
      }
    }
  }
  void worker_loop(std::size_t worker) {
    std::size_t observed = 0;
    std::unique_lock lock(mutex_);
    for (;;) {
      ready_.wait(lock, [&] { return stopping_ || generation_ != observed; });
      if (stopping_) {
        return;
      }
      observed = generation_;
      lock.unlock();
      execute(worker);
      lock.lock();
      if (--pending_ == 0) {
        finished_.notify_one();
      }
    }
  }
  void stop() noexcept {
    {
      std::lock_guard lock(mutex_);
      stopping_ = true;
    }
    ready_.notify_all();
    for (auto& thread : threads_) {
      thread.join();
    }
  }
  std::size_t count_;
  std::mutex mutex_;
  std::condition_variable ready_, finished_;
  std::vector<std::thread> threads_;
  std::vector<std::exception_ptr> errors_;
  std::function<void(std::size_t, std::size_t)> job_;
  std::size_t blocks_{}, pending_{}, generation_{};
  bool stopping_{};
};
} // namespace pointcloud_ad::registration
