#include "cuda_correspondence.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <random>
#include <string_view>

namespace {
using namespace pointcloud_ad;
using namespace pointcloud_ad::backends::cuda_backend;
bool expect(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}
template <typename Function> bool rejects(Function&& function) {
  try {
    function();
  } catch (const std::exception&) {
    return true;
  }
  return false;
}
std::vector<std::int32_t> oracle(const std::vector<Vec3f>& reference,
                                 const std::vector<Vec3f>& queries, double bound) {
  std::vector<std::int32_t> result(queries.size(), -1);
  for (std::size_t i = 0; i < queries.size(); ++i) {
    double best = bound * bound;
    for (std::size_t j = 0; j < reference.size(); ++j) {
      const double dx = static_cast<double>(queries[i].x) - reference[j].x;
      const double dy = static_cast<double>(queries[i].y) - reference[j].y;
      const double dz = static_cast<double>(queries[i].z) - reference[j].z;
      const double distance = dx * dx + dy * dy + dz * dz;
      if (distance < best || (distance == best && result[i] == -1)) {
        best = distance;
        result[i] = static_cast<std::int32_t>(j);
      }
    }
  }
  return result;
}
bool compare(const std::vector<Vec3f>& reference, const std::vector<Vec3f>& queries,
             const std::vector<double>& radii) {
  CorrespondenceSearch indexed(reference, queries.size());
  CorrespondenceSearch tiled(reference, queries.size(), SearchStrategy::tiled);
  bool passed = true;
  for (double radius : radii) {
    const auto expected = oracle(reference, queries, radius);
    passed &= expect(indexed.query(queries, radius) == expected,
                     "indexed search equals exhaustive double oracle");
    passed &= expect(tiled.query(queries, radius) == expected,
                     "tiled search equals exhaustive double oracle");
    passed &= expect(indexed.query(queries, radius) == expected, "indexed search repeats exactly");
    const std::span<const Vec3f> shorter(queries.data(), queries.size() / 2U);
    const auto short_result = indexed.query(shorter, radius);
    passed &=
        expect(short_result == std::vector<std::int32_t>(
                                   expected.begin(),
                                   expected.begin() + static_cast<std::ptrdiff_t>(shorter.size())),
               "changing query length preserves results");
    passed &= expect(indexed.query({}, radius).empty(), "empty query valid");
  }
  return passed;
}
} // namespace

int main() {
  if (!available()) {
    std::cerr << "CUDA test requires an accessible GPU\n";
    return 1;
  }
  bool passed = true;
  std::mt19937 generator(713U);
  for (std::size_t count : {1U, 16U, 17U, 129U, 2053U}) {
    std::vector<Vec3f> points, queries;
    for (std::size_t i = 0; i < count; ++i) {
      points.push_back({0.1F * static_cast<float>(generator() % 1000U) - 50.0F,
                        0.1F * static_cast<float>(generator() % 500U) - 25.0F,
                        0.01F * static_cast<float>(generator() % 10U)});
    }
    for (std::size_t i = 0; i < 259U; ++i) {
      const auto p = points[i % count];
      queries.push_back({p.x + (i % 3U == 0U ? 0.0F : 0.125F), p.y, p.z});
    }
    queries.push_back({1000, -1000, 1000});
    passed &= compare(points, queries, {0.0, 0.125, 0.5, 10.0, 10000.0});
  }
  // Uneven 3D clusters, isolated outliers and reordered duplicates exercise pruning beyond
  // the near-planar fixture. The fixed PRNG sequence keeps the exhaustive comparison repeatable.
  std::vector<Vec3f> clustered, cluster_queries;
  for (std::size_t i = 0; i < 4099U; ++i) {
    const float center = i % 13U == 0U ? 500.0F : (i % 3U == 0U ? -5.0F : 5.0F);
    const auto noise = [&] { return 0.01F * static_cast<float>(generator() % 200U) - 1.0F; };
    clustered.push_back({center + noise(), center * 0.2F + noise(), noise()});
  }
  for (std::size_t i = 0; i < 299U; ++i) {
    const auto p = clustered[(i * 17U) % clustered.size()];
    cluster_queries.push_back({p.x + 0.02F, p.y - 0.03F, p.z + 0.04F});
  }
  clustered[1] = clustered.back();
  cluster_queries.push_back(clustered.back());
  passed &= compare(clustered, cluster_queries, {0.0, 0.05, 1.0, 10000.0});
  // Force the lowest original index into the geometrically later subtree, at an exact tie.
  std::vector<Vec3f> ties(257U, Vec3f{-1, 0, 0});
  ties[0] = {1, 0, 0};
  passed &= compare(ties, {{0, 0, 0}, {1, 0, 0}, {-1, 0, 0}},
                    {0.0, std::nextafter(1.0, 0.0), 1.0, std::nextafter(1.0, 2.0)});
  passed &= compare(std::vector<Vec3f>(2049U, Vec3f{0, 0, 0}), {{0, 0, 0}, {0, 0, 1}}, {0.0, 1.0});
  const float large = 1.0e20F;
  passed &= compare({{large, large, 0},
                     {std::nextafter(large, std::numeric_limits<float>::infinity()), large, 0},
                     {-large, 0, 0}},
                    {{large, large, 0}, {0, 0, 0}, {large, -large, 0}}, {0.0, 2.0e20, 1.0e100});
  const float tiny = std::numeric_limits<float>::denorm_min();
  passed &= compare({{tiny, 0, 0}, {-tiny, 0, 0}, {0, tiny, 0}}, {{0, 0, 0}, {tiny, 0, 0}},
                    {0.0, static_cast<double>(tiny)});

  CorrespondenceSearch search(ties, 1U);
  const std::vector<Vec3f> one{{0, 0, 0}};
  const std::vector<Vec3f> too_many{{0, 0, 0}, {1, 0, 0}};
  const std::vector<Vec3f> invalid{{std::numeric_limits<float>::infinity(), 0, 0}};
  passed &=
      expect(rejects([&] { (void)search.query(too_many, 1.0); }), "capacity overflow rejected");
  passed &= expect(rejects([&] { (void)search.query(invalid, 1.0); }), "nonfinite query rejected");
  for (double radius :
       {-1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    passed &= expect(rejects([&] { (void)search.query(one, radius); }), "invalid radius rejected");
  }
  passed &= expect(rejects([&] { CorrespondenceSearch bad(invalid, 1U); }),
                   "nonfinite reference rejected");
  passed &= expect(rejects([&] { CorrespondenceSearch bad({}, 1U); }), "empty reference rejected");
  passed &=
      expect(rejects([&] { CorrespondenceSearch bad(one, 1U, static_cast<SearchStrategy>(255)); }),
             "invalid strategy rejected");
  return passed ? 0 : 1;
}
