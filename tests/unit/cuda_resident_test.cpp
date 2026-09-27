#include "cpu_kernels.hpp"
#include "cuda_correspondence.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <vector>

int main() {
  using namespace pointcloud_ad;
  using namespace pointcloud_ad::backends::cuda_backend;
  if (!available())
    return 77;
  bool ok = true;
  for (std::size_t count : {1U, 127U, 128U, 129U, 257U}) {
    std::vector<Vec3f> reference;
    std::vector<Vec3d> points, normals;
    for (std::size_t i = 0; i < count; ++i) {
      const auto x = static_cast<double>(i);
      reference.push_back({static_cast<float>(x), 0, 0});
      points.push_back({x, 0, 0.02});
      normals.push_back({0, 0, 1});
    }
    CorrespondenceSearch search(reference, count);
    search.prepare_resident(normals);
    search.upload_scan(points, normals);
    std::array<double, 16> pose{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    const auto initial_transfers = search.transfer_statistics();
    for (bool plane : {false, true})
      for (bool final : {false, true}) {
        auto result = search.evaluate(pose, 0.1, 0.01, plane, final);
        auto again = search.evaluate(pose, 0.1, 0.01, plane, final);
        Equation expected;
        for (std::size_t i = 0; i < count; ++i) {
          registration::ObjectiveInput input;
          input.point = points[i];
          input.reference = {reference[i].x, 0, 0};
          input.normal = normals[i];
          input.plane = plane;
          input.generalized = false;
          input.huber_delta = 0.01;
          registration::ObjectiveContribution part;
          registration::objective_kernel(CpuKernel::scalar)(input, part);
          if (!final) {
            for (std::size_t j = 0; j < 36; ++j)
              expected.values[j] += part.matrix[j];
            for (std::size_t j = 0; j < 6; ++j)
              expected.values[36 + j] += part.gradient[j];
          }
          expected.values[42] +=
              final ? part.absolute_residual * part.absolute_residual : part.absolute_residual;
          expected.values[43] += 1;
        }
        for (std::size_t j = 0; j < 45; ++j) {
          ok &= result.values[j] == again.values[j];
          ok &= std::abs(result.values[j] - expected.values[j]) <=
                1e-10 * std::max(1.0, std::abs(expected.values[j]));
        }
      }
    const auto iteration_transfers = search.transfer_statistics();
    ok &= iteration_transfers.host_to_device_bytes == initial_transfers.host_to_device_bytes;
    ok &= iteration_transfers.device_to_host_bytes - initial_transfers.device_to_host_bytes ==
          8 * sizeof(Equation);
    ok &= iteration_transfers.equation_evaluations - initial_transfers.equation_evaluations == 8;
    auto moved = pose;
    moved[3] = 10000;
    ok &= search.evaluate(moved, 0.1, 0.01, true, false).values[43] == 0;
    moved = pose;
    moved[3] = std::numeric_limits<double>::max();
    bool rejected = false;
    try {
      (void)search.evaluate(moved, 0.1, 0.01, true, false);
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    ok &= rejected;
    ok &= search.evaluate(pose, 0.1, 0.01, true, false).values[43] == static_cast<double>(count);
    for (auto& n : normals)
      n.z = -1;
    search.upload_scan(points, normals);
    ok &= search.evaluate(pose, 0.1, 0.01, true, false).values[43] == 0;
    search.upload_scan(points, {});
    ok &= search.evaluate(pose, 0.1, 0.01, true, false).values[43] == static_cast<double>(count);
  }

  std::vector<Vec3f> cloud;
  for (int x = -4; x <= 4; ++x)
    for (int y = -4; y <= 4; ++y)
      cloud.push_back({static_cast<float>(x) * 0.3F, static_cast<float>(y) * 0.4F,
                       static_cast<float>(x * x + 2 * y * y) * 0.01F});
  cloud.push_back(cloud[0]);
  CorrespondenceSearch covariance_search(cloud, cloud.size());
  covariance_search.prepare_resident({});
  for (std::uint32_t k : {1U, 3U, 12U, 32U, 82U}) {
    const auto actual = covariance_search.knn(k);
    for (std::size_t i = 0; i < cloud.size(); ++i) {
      std::vector<std::int32_t> ids(cloud.size());
      std::iota(ids.begin(), ids.end(), 0);
      const auto distance = [&](std::int32_t j) {
        const auto p = cloud[static_cast<std::size_t>(j)], q = cloud[i];
        const double x = static_cast<double>(p.x) - q.x, y = static_cast<double>(p.y) - q.y,
                     z = static_cast<double>(p.z) - q.z;
        return x * x + y * y + z * z;
      };
      std::sort(ids.begin(), ids.end(), [&](auto a, auto b) {
        return distance(a) < distance(b) || (distance(a) == distance(b) && a < b);
      });
      for (std::uint32_t j = 0; j < k; ++j)
        ok &= actual[i * k + j] == ids[j];
    }
  }
  const auto neighbors = covariance_search.knn(12);
  covariance_search.prepare_gicp(12, 0.001);
  const auto covariance = covariance_search.download_covariances();
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    std::vector<std::int32_t> ids(neighbors.begin() + static_cast<std::ptrdiff_t>(i * 12),
                                  neighbors.begin() + static_cast<std::ptrdiff_t>((i + 1) * 12));
    std::sort(ids.begin(), ids.end());
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (auto id : ids) {
      const auto p = cloud[static_cast<std::size_t>(id)];
      mean += Eigen::Vector3d(p.x, p.y, p.z);
    }
    mean /= 12;
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();
    for (auto id : ids) {
      const auto p = cloud[static_cast<std::size_t>(id)];
      const Eigen::Vector3d d = Eigen::Vector3d(p.x, p.y, p.z) - mean;
      matrix += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(matrix);
    const Eigen::Matrix3d expected = eigen.eigenvectors() *
                                     Eigen::Vector3d(0.001, 1, 1).asDiagonal() *
                                     eigen.eigenvectors().transpose();
    for (std::size_t r = 0; r < 3; ++r)
      for (std::size_t c = 0; c < 3; ++c)
        ok &=
            std::abs(covariance[i].values[r * 3 + c] -
                     expected(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c))) < 1e-10;
  }
  std::vector<Vec3d> scan;
  for (auto p : cloud)
    scan.push_back({p.x, p.y, p.z});
  covariance_search.upload_scan(scan, {});
  const std::array<double, 16> identity{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  const auto result = covariance_search.evaluate(identity, 0.1, 0.01, false, false);
  const auto repeat = covariance_search.evaluate(identity, 0.1, 0.01, false, false);
  ok &= result.values[43] == static_cast<double>(cloud.size()) && result.values[42] == 0;
  for (std::size_t i = 0; i < 45; ++i)
    ok &= result.values[i] == repeat.values[i];

  auto pose = identity;
  pose[0] = pose[5] = std::cos(0.01);
  pose[1] = -std::sin(0.01);
  pose[4] = std::sin(0.01);
  pose[3] = 0.002;
  const auto gpu = covariance_search.evaluate(pose, 0.1, 0.01, false, false);
  Equation oracle;
  Eigen::Matrix3d rotation;
  rotation << pose[0], pose[1], pose[2], pose[4], pose[5], pose[6], pose[8], pose[9], pose[10];
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    registration::ObjectiveInput input{};
    const auto q = scan[i];
    input.point = {pose[0] * q.x + pose[1] * q.y + pose[3], pose[4] * q.x + pose[5] * q.y, q.z};
    input.reference = q;
    input.huber_delta = 0.01;
    input.generalized = true;
    input.plane = false;
    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> covariance_matrix(
        covariance[i].values);
    const Eigen::Matrix3d information =
        (covariance_matrix + rotation * covariance_matrix * rotation.transpose()).inverse();
    for (std::size_t r = 0; r < 3; ++r)
      for (std::size_t c = 0; c < 3; ++c)
        input.information[r * 3 + c] =
            information(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c));
    registration::ObjectiveContribution part;
    registration::objective_kernel(CpuKernel::scalar)(input, part);
    for (std::size_t j = 0; j < 36; ++j)
      oracle.values[j] += part.matrix[j];
    for (std::size_t j = 0; j < 6; ++j)
      oracle.values[36 + j] += part.gradient[j];
    oracle.values[42] += part.absolute_residual;
    oracle.values[43] += 1;
  }
  for (std::size_t j = 0; j < 45; ++j)
    ok &= std::abs(gpu.values[j] - oracle.values[j]) <
          1e-9 * std::max(1.0, std::abs(oracle.values[j]));
  bool rejected = false;
  try {
    covariance_search.upload_scan(std::vector<Vec3d>(cloud.size()), {});
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  ok &= rejected;
  covariance_search.upload_scan(scan, {});
  ok &= covariance_search.evaluate(identity, 0.1, 0.01, false, false).values[43] ==
        static_cast<double>(cloud.size());
  if (!ok)
    std::cerr << "resident equation oracle mismatch\n";
  return ok ? 0 : 1;
}
