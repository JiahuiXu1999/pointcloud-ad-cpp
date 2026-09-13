#include "cuda_correspondence.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <string_view>

int main() {
  using namespace pointcloud_ad;
  using namespace pointcloud_ad::backends::cuda_backend;
  using Clock = std::chrono::steady_clock;
  if (!available()) {
    std::cerr << "CUDA search benchmark requires a GPU\n";
    return 1;
  }
  std::cout << "workload,points,queries,strategy,prepare_median_ms,query_median_ms,total_median_ms,"
               "device_payload_mib\n";
  for (std::string_view workload : {"lattice", "plane", "duplicates"}) {
    for (std::size_t count : {1024U, 16384U, 131072U, 1048576U}) {
      std::vector<Vec3f> reference, queries;
      reference.reserve(count);
      queries.reserve(count);
      const double radius = workload == "plane" ? 0.01 : 0.5;
      for (std::size_t i = 0; i < count; ++i) {
        Vec3f point{static_cast<float>(i % 128U), static_cast<float>((i / 128U) % 128U),
                    static_cast<float>(i / 16384U)};
        if (workload == "plane") {
          point = {0.02F * static_cast<float>(i % 1024U), 0.02F * static_cast<float>(i / 1024U), 0};
        }
        if (workload == "duplicates") {
          point = {1, -2, 3};
        }
        reference.push_back(point);
        point.z += workload == "plane" ? 0.0025F : 0.125F;
        queries.push_back(point);
      }
      for (auto strategy : {SearchStrategy::indexed, SearchStrategy::tiled}) {
        // Exhaustive GPU work at larger N would dominate this benchmark quadratically.
        // Larger rows verify the indexed backend only, with analytically known correspondences.
        if (strategy == SearchStrategy::tiled && count > 16384U) {
          continue;
        }
        std::array<double, 3> prepare{}, query{}, total{};
        std::size_t bytes = 0;
        for (int run = -1; run < 3; ++run) {
          const auto start = Clock::now();
          double prepare_ms = 0, query_ms = 0;
          std::vector<std::int32_t> result;
          {
            CorrespondenceSearch search(reference, queries.size(), strategy);
            const auto ready = Clock::now();
            bytes = search.device_storage_bytes();
            result = search.query(queries, radius);
            const auto finished = Clock::now();
            prepare_ms = std::chrono::duration<double, std::milli>(ready - start).count();
            query_ms = std::chrono::duration<double, std::milli>(finished - ready).count();
          }
          const double total_ms =
              std::chrono::duration<double, std::milli>(Clock::now() - start).count();
          for (std::size_t i = 0; i < result.size(); ++i) {
            if (result[i] != (workload == "duplicates" ? 0 : static_cast<std::int32_t>(i))) {
              std::cerr << "search benchmark correspondence gate failed\n";
              return 1;
            }
          }
          if (run >= 0) {
            const auto sample = static_cast<std::size_t>(run);
            prepare[sample] = prepare_ms;
            query[sample] = query_ms;
            total[sample] = total_ms;
          }
        }
        std::sort(prepare.begin(), prepare.end());
        std::sort(query.begin(), query.end());
        std::sort(total.begin(), total.end());
        std::cout << workload << ',' << count << ',' << count << ','
                  << (strategy == SearchStrategy::indexed ? "indexed" : "tiled") << ','
                  << prepare[1] << ',' << query[1] << ',' << total[1] << ','
                  << static_cast<double>(bytes) / (1024.0 * 1024.0) << '\n';
      }
    }
  }
}
