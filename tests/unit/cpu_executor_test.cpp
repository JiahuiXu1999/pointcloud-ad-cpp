#include "cpu_executor.hpp"

#include <atomic>
#include <iostream>
#include <string>

int main() {
  using pointcloud_ad::registration::CpuExecutor;
  bool passed = true;
  for (unsigned int budget : {1U, 2U, 4U, 8U}) {
    CpuExecutor executor(budget, 4096);
    passed &= executor.concurrency() == budget;
    for (std::size_t count : {0U, 1U, 255U, 256U, 257U, 1009U, 4096U}) {
      for (int run = 0; run < 20; ++run) {
        std::vector<unsigned int> visits(count);
        executor.run(count, [&](std::size_t block, std::size_t worker) {
          if (worker >= budget) {
            throw std::runtime_error("worker exceeds budget");
          }
          for (std::size_t i = block * CpuExecutor::block_size;
               i < std::min(count, (block + 1) * CpuExecutor::block_size); ++i) {
            ++visits[i];
          }
        });
        passed &= std::all_of(visits.begin(), visits.end(), [](auto n) { return n == 1; });
      }
    }
    std::atomic<int> completed{};
    try {
      executor.run(4096, [&](std::size_t block, std::size_t) {
        ++completed;
        if (block < 2) {
          throw std::runtime_error(std::to_string(block));
        }
      });
      passed = false;
    } catch (const std::runtime_error& error) {
      passed &= std::string(error.what()) == "0" && completed == 16;
    }
    executor.run(256, [&](std::size_t, std::size_t) { ++completed; });
    passed &= completed == 17;
  }
  bool rejected_zero = false;
  try {
    CpuExecutor invalid(0, 1000);
    passed = false;
  } catch (const std::invalid_argument&) {
    rejected_zero = true;
  }
  passed &= rejected_zero;
  CpuExecutor bounded(10000, 1);
  passed &= bounded.concurrency() == 1;
  if (!passed) {
    std::cerr << "executor coverage/budget/recovery failed\n";
  }
  return passed ? 0 : 1;
}
