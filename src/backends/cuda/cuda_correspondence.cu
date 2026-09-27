#include "cuda_correspondence.hpp"
#include "spatial_index.hpp"

#include <cmath>
#include <cuda_runtime.h>
#include <limits>
#include <stdexcept>
#include <string>

namespace pointcloud_ad::backends::cuda_backend {
namespace {

void checked(cudaError_t code, const char* operation) {
  if (code != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(code));
  }
}

constexpr unsigned int kBlockSize = 128U;

} // namespace

// O(query_count * reference_count) exact search with O(N+M) storage. Cooperative reference
// tiles avoid a dense distance matrix. Stable lower-index tie breaking, no float atomics.
__global__ void nearest_kernel(const Vec3f* reference, std::size_t reference_count,
                               const Vec3f* queries, std::size_t query_count,
                               double max_squared_distance, std::int32_t* indices) {
  __shared__ float tile_x[kBlockSize];
  __shared__ float tile_y[kBlockSize];
  __shared__ float tile_z[kBlockSize];
  const std::size_t query_index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  Vec3f query{};
  if (query_index < query_count) {
    query = queries[query_index];
  }
  double best = max_squared_distance;
  std::int32_t best_index = -1;
  for (std::size_t offset = 0; offset < reference_count; offset += kBlockSize) {
    const std::size_t reference_index = offset + threadIdx.x;
    if (reference_index < reference_count) {
      tile_x[threadIdx.x] = reference[reference_index].x;
      tile_y[threadIdx.x] = reference[reference_index].y;
      tile_z[threadIdx.x] = reference[reference_index].z;
    }
    __syncthreads();
    const auto count = static_cast<unsigned int>(
        reference_count - offset < kBlockSize ? reference_count - offset : kBlockSize);
    if (query_index < query_count) {
      for (unsigned int index = 0; index < count; ++index) {
        const double dx = static_cast<double>(query.x) - tile_x[index];
        const double dy = static_cast<double>(query.y) - tile_y[index];
        const double dz = static_cast<double>(query.z) - tile_z[index];
        const double distance = dx * dx + dy * dy + dz * dz;
        if (distance < best || (distance == best && best_index < 0)) {
          best = distance;
          best_index = static_cast<std::int32_t>(offset + index);
        }
      }
    }
    __syncthreads();
  }
  if (query_index < query_count) {
    indices[query_index] = best_index;
  }
}

// Exact branch-and-bound traversal. Escape links eliminate recursion and per-thread stacks;
// the inclusive bound and original-index tie rule match the tiled exhaustive oracle.
__global__ void indexed_nearest_kernel(const Vec3f* reference, const std::int32_t* original_indices,
                                       const SpatialNode* nodes, std::uint32_t node_count,
                                       const Vec3f* queries, std::size_t query_count,
                                       double max_squared_distance, std::int32_t* indices) {
  const std::size_t query_index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (query_index >= query_count) {
    return;
  }
  const Vec3f query = queries[query_index];
  double best = max_squared_distance;
  std::int32_t best_index = -1;
  std::uint32_t cursor = 0U;
  while (cursor < node_count) {
    const SpatialNode node = nodes[cursor];
    const double dx =
        query.x < node.lower.x
            ? static_cast<double>(node.lower.x) - query.x
            : (query.x > node.upper.x ? static_cast<double>(query.x) - node.upper.x : 0.0);
    const double dy =
        query.y < node.lower.y
            ? static_cast<double>(node.lower.y) - query.y
            : (query.y > node.upper.y ? static_cast<double>(query.y) - node.upper.y : 0.0);
    const double dz =
        query.z < node.lower.z
            ? static_cast<double>(node.lower.z) - query.z
            : (query.z > node.upper.z ? static_cast<double>(query.z) - node.upper.z : 0.0);
    const double lower_distance = dx * dx + dy * dy + dz * dz;
    if (lower_distance > best ||
        (lower_distance == best && best_index >= 0 && node.minimum_index >= best_index)) {
      cursor = node.escape;
      continue;
    }
    if (node.count == 0U) {
      ++cursor;
      continue;
    }
    for (auto index = node.begin; index < node.begin + node.count; ++index) {
      const auto point = reference[index];
      const double px = static_cast<double>(query.x) - point.x;
      const double py = static_cast<double>(query.y) - point.y;
      const double pz = static_cast<double>(query.z) - point.z;
      const double distance = px * px + py * py + pz * pz;
      const auto original = original_indices[index];
      if (distance < best || (distance == best && (best_index < 0 || original < best_index))) {
        best = distance;
        best_index = original;
      }
    }
    cursor = node.escape;
  }
  indices[query_index] = best_index;
}

namespace resident_detail {
struct Pose {
  double v[16];
};
__device__ Vec3d direction(Pose pose, Vec3d p) {
  return {pose.v[0] * p.x + pose.v[1] * p.y + pose.v[2] * p.z,
          pose.v[4] * p.x + pose.v[5] * p.y + pose.v[6] * p.z,
          pose.v[8] * p.x + pose.v[9] * p.y + pose.v[10] * p.z};
}
__global__ void transform_resident(const Vec3d* scan, Vec3d* transformed, Vec3f* queries,
                                   std::size_t count, Pose pose) {
  const auto i = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i >= count)
    return;
  auto p = direction(pose, scan[i]);
  p.x += pose.v[3];
  p.y += pose.v[7];
  p.z += pose.v[11];
  transformed[i] = p;
  queries[i] = {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)};
}
// Each lane owns one point. The binary tree and final block merge are fixed, with no atomics.
__global__ void resident_equations(const Vec3f* reference, const Vec3d* ref_normals,
                                   const Vec3d* transformed, const Vec3f* queries,
                                   const Vec3d* scan_normals, const std::int32_t* indices,
                                   std::size_t count, Pose pose, double huber, bool plane,
                                   bool final_metrics, const Covariance* reference_covariances,
                                   const Covariance* scan_covariances, Equation* blocks) {
  __shared__ double values[kBlockSize][45];
  const unsigned int lane = threadIdx.x;
  for (unsigned int j = 0; j < 45; ++j)
    values[lane][j] = 0;
  const auto i = static_cast<std::size_t>(blockIdx.x) * blockDim.x + lane;
  if (i < count) {
    const auto packed = queries[i];
    if (!isfinite(packed.x) || !isfinite(packed.y) || !isfinite(packed.z))
      values[lane][44] = 1;
    else if (indices[i] >= 0) {
      const auto target = indices[i];
      const auto n = ref_normals ? ref_normals[target] : Vec3d{};
      const auto sn = scan_normals ? direction(pose, scan_normals[i]) : Vec3d{};
      if (!scan_normals || !ref_normals || sn.x * n.x + sn.y * n.y + sn.z * n.z >= 0) {
        const auto p = transformed[i];
        const auto r = reference[target];
        const Vec3d e{p.x - r.x, p.y - r.y, p.z - r.z};
        const double residual =
            plane ? e.x * n.x + e.y * n.y + e.z * n.z : sqrt(e.x * e.x + e.y * e.y + e.z * e.z);
        values[lane][42] = final_metrics ? residual * residual : fabs(residual);
        values[lane][43] = 1;
        if (!final_metrics) {
          const double weight = fabs(residual) <= huber ? 1.0 : huber / fabs(residual);
          if (plane) {
            const double jac[6]{
                p.y * n.z - p.z * n.y, p.z * n.x - p.x * n.z, p.x * n.y - p.y * n.x, n.x, n.y, n.z};
            for (unsigned int row = 0; row < 6; ++row) {
              for (unsigned int col = 0; col < 6; ++col)
                values[lane][row * 6 + col] = (weight * jac[row]) * jac[col];
              values[lane][36 + row] = (weight * (-residual)) * jac[row];
            }
          } else {
            const double jac[18]{0, p.z, -p.y, 1,   0,    0, -p.z, 0, p.x,
                                 0, 1,   0,    p.y, -p.x, 0, 0,    0, 1};
            double information[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
            if (reference_covariances) {
              double rotated[9]{}, combined[9]{};
              const auto source = scan_covariances[i];
              for (unsigned int row = 0; row < 3; ++row)
                for (unsigned int col = 0; col < 3; ++col)
                  for (unsigned int j = 0; j < 3; ++j)
                    rotated[row * 3 + col] += pose.v[row * 4 + j] * source.values[j * 3 + col];
              for (unsigned int row = 0; row < 3; ++row)
                for (unsigned int col = 0; col < 3; ++col) {
                  combined[row * 3 + col] = reference_covariances[target].values[row * 3 + col];
                  for (unsigned int j = 0; j < 3; ++j)
                    combined[row * 3 + col] += rotated[row * 3 + j] * pose.v[col * 4 + j];
                }
              const double a = combined[0], b = combined[1], c = combined[2], d = combined[3],
                           e = combined[4], f = combined[5], g = combined[6], h = combined[7],
                           j = combined[8];
              const double determinant =
                  a * (e * j - f * h) - b * (d * j - f * g) + c * (d * h - e * g);
              const double cofactors[9]{e * j - f * h, c * h - b * j, b * f - c * e,
                                        f * g - d * j, a * j - c * g, c * d - a * f,
                                        d * h - e * g, b * g - a * h, a * e - b * d};
              for (unsigned int col = 0; col < 9; ++col)
                information[col] = cofactors[col] / determinant;
            }
            for (unsigned int row = 0; row < 6; ++row) {
              double weighted[3];
              for (unsigned int col = 0; col < 3; ++col)
                weighted[col] = reference_covariances
                                    ? ((weight * jac[row]) * information[col] +
                                       (weight * jac[6 + row]) * information[3 + col]) +
                                          (weight * jac[12 + row]) * information[6 + col]
                                    : weight * jac[6 * col + row];
              const double a = weighted[0], b = weighted[1], c = weighted[2];
              for (unsigned int col = 0; col < 6; ++col)
                values[lane][row * 6 + col] = (a * jac[col] + b * jac[6 + col]) + c * jac[12 + col];
              values[lane][36 + row] = -((a * e.x + b * e.y) + c * e.z);
            }
          }
        }
      }
    }
  }
  __syncthreads();
  for (unsigned int stride = kBlockSize / 2; stride > 0; stride /= 2) {
    if (lane < stride)
      for (unsigned int j = 0; j < 45; ++j)
        values[lane][j] += values[lane + stride][j];
    __syncthreads();
  }
  if (lane == 0)
    for (unsigned int j = 0; j < 45; ++j)
      blocks[blockIdx.x].values[j] = values[0][j];
}
__global__ void merge_equations(const Equation* blocks, std::size_t count, Equation* result) {
  const auto entry = threadIdx.x;
  if (entry >= 45)
    return;
  double sum = 0;
  for (std::size_t i = 0; i < count; ++i)
    sum += blocks[i].values[entry];
  result->values[entry] = sum;
}
__global__ void restore_order(const Vec3f* packed, const std::int32_t* order, Vec3f* original,
                              std::size_t count) {
  const auto i = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i < count)
    original[order[i]] = packed[i];
}

__device__ double squared_distance(Vec3f a, Vec3f b) {
  const double x = static_cast<double>(a.x) - b.x, y = static_cast<double>(a.y) - b.y,
               z = static_cast<double>(a.z) - b.z;
  return x * x + y * y + z * z;
}
__global__ void exact_knn(const Vec3f* packed, const std::int32_t* order, const SpatialNode* nodes,
                          std::uint32_t node_count, const Vec3f* original, std::size_t offset,
                          std::size_t count, std::uint32_t k, std::int32_t* neighbors,
                          double* distances) {
  const auto i = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i >= count)
    return;
  auto* ids = neighbors + i * k;
  auto* ds = distances + i * k;
  const auto q = original[offset + i];
  std::uint32_t found = 0;
  // Median construction bounds depth below 32 for INT32_MAX points. Visit nearer boxes first;
  // the stack holds at most one deferred sibling per level, with spare capacity.
  std::uint32_t pending[64];
  unsigned int pending_count = 1;
  pending[0] = 0;
  while (pending_count > 0) {
    const auto cursor = pending[--pending_count];
    if (cursor >= node_count)
      continue;
    const auto node = nodes[cursor];
    const double x = q.x < node.lower.x
                         ? static_cast<double>(node.lower.x) - q.x
                         : (q.x > node.upper.x ? static_cast<double>(q.x) - node.upper.x : 0);
    const double y = q.y < node.lower.y
                         ? static_cast<double>(node.lower.y) - q.y
                         : (q.y > node.upper.y ? static_cast<double>(q.y) - node.upper.y : 0);
    const double z = q.z < node.lower.z
                         ? static_cast<double>(node.lower.z) - q.z
                         : (q.z > node.upper.z ? static_cast<double>(q.z) - node.upper.z : 0);
    const double lower = x * x + y * y + z * z;
    if (found == k &&
        (lower > ds[k - 1] || (lower == ds[k - 1] && node.minimum_index >= ids[k - 1]))) {
      continue;
    }
    if (node.count == 0) {
      const auto first = cursor + 1;
      const auto second = nodes[first].escape;
      const auto lower_bound = [&](SpatialNode child) {
        const double dx =
            q.x < child.lower.x
                ? static_cast<double>(child.lower.x) - q.x
                : (q.x > child.upper.x ? static_cast<double>(q.x) - child.upper.x : 0);
        const double dy =
            q.y < child.lower.y
                ? static_cast<double>(child.lower.y) - q.y
                : (q.y > child.upper.y ? static_cast<double>(q.y) - child.upper.y : 0);
        const double dz =
            q.z < child.lower.z
                ? static_cast<double>(child.lower.z) - q.z
                : (q.z > child.upper.z ? static_cast<double>(q.z) - child.upper.z : 0);
        return dx * dx + dy * dy + dz * dz;
      };
      const bool first_nearer = lower_bound(nodes[first]) <= lower_bound(nodes[second]);
      pending[pending_count++] = first_nearer ? second : first;
      pending[pending_count++] = first_nearer ? first : second;
      continue;
    }
    for (auto n = node.begin; n < node.begin + node.count; ++n) {
      const auto d = squared_distance(q, packed[n]);
      const auto id = order[n];
      std::uint32_t position = found;
      if (found == k) {
        if (d > ds[k - 1] || (d == ds[k - 1] && id >= ids[k - 1]))
          continue;
        position = k - 1;
      }
      while (position > 0 &&
             (d < ds[position - 1] || (d == ds[position - 1] && id < ids[position - 1]))) {
        ids[position] = ids[position - 1];
        ds[position] = ds[position - 1];
        --position;
      }
      ids[position] = id;
      ds[position] = d;
      if (found < k)
        ++found;
    }
  }
}
__global__ void covariance_kernel(const Vec3f* original, std::int32_t* neighbors,
                                  std::size_t offset, std::size_t count, std::uint32_t k,
                                  double epsilon, Covariance* output, int* invalid) {
  const auto i = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i >= count)
    return;
  auto* ids = neighbors + i * k;
  // Accumulate in original-index order, independently of traversal or equal-distance ordering.
  for (std::uint32_t j = 1; j < k; ++j) {
    auto value = ids[j];
    auto at = j;
    while (at > 0 && ids[at - 1] > value) {
      ids[at] = ids[at - 1];
      --at;
    }
    ids[at] = value;
  }
  double mean[3]{};
  for (std::uint32_t j = 0; j < k; ++j) {
    const auto p = original[ids[j]];
    mean[0] += p.x;
    mean[1] += p.y;
    mean[2] += p.z;
  }
  for (auto& v : mean)
    v /= k;
  double matrix[9]{}, vectors[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
  for (std::uint32_t j = 0; j < k; ++j) {
    const auto p = original[ids[j]];
    const double delta[3]{p.x - mean[0], p.y - mean[1], p.z - mean[2]};
    for (unsigned int r = 0; r < 3; ++r)
      for (unsigned int c = 0; c < 3; ++c)
        matrix[r * 3 + c] += delta[r] * delta[c];
  }
  // Symmetric 3x3 Jacobi with deterministic pivot tie-breaking; no vendor solver state.
  for (unsigned int iteration = 0; iteration < 48; ++iteration) {
    unsigned int p = 0, q = 1;
    if (fabs(matrix[2]) > fabs(matrix[p * 3 + q])) {
      p = 0;
      q = 2;
    }
    if (fabs(matrix[5]) > fabs(matrix[p * 3 + q])) {
      p = 1;
      q = 2;
    }
    const double off = matrix[p * 3 + q];
    if (fabs(off) <= 1e-15 * (fabs(matrix[0]) + fabs(matrix[4]) + fabs(matrix[8])))
      break;
    if (off == 0)
      break;
    const double tau = (matrix[q * 3 + q] - matrix[p * 3 + p]) / (2 * off);
    const double t = copysign(1.0, tau) / (fabs(tau) + sqrt(1 + tau * tau));
    const double c = 1 / sqrt(1 + t * t), sn = t * c;
    const double pp = matrix[p * 3 + p], qq = matrix[q * 3 + q];
    matrix[p * 3 + p] = pp - t * off;
    matrix[q * 3 + q] = qq + t * off;
    matrix[p * 3 + q] = matrix[q * 3 + p] = 0;
    for (unsigned int r = 0; r < 3; ++r) {
      if (r != p && r != q) {
        const double rp = matrix[r * 3 + p], rq = matrix[r * 3 + q];
        matrix[r * 3 + p] = matrix[p * 3 + r] = c * rp - sn * rq;
        matrix[r * 3 + q] = matrix[q * 3 + r] = sn * rp + c * rq;
      }
      const double vp = vectors[r * 3 + p], vq = vectors[r * 3 + q];
      vectors[r * 3 + p] = c * vp - sn * vq;
      vectors[r * 3 + q] = sn * vp + c * vq;
    }
  }
  unsigned int order[3]{0, 1, 2};
  for (unsigned int j = 1; j < 3; ++j)
    for (unsigned int x = j;
         x > 0 && matrix[order[x] * 3 + order[x]] < matrix[order[x - 1] * 3 + order[x - 1]]; --x) {
      auto tmp = order[x];
      order[x] = order[x - 1];
      order[x - 1] = tmp;
    }
  if (!isfinite(matrix[0]) || !isfinite(matrix[4]) || !isfinite(matrix[8]) ||
      matrix[order[1] * 3 + order[1]] <= 1e-12) {
    atomicExch(invalid, 1);
    return;
  }
  Covariance result;
  for (unsigned int r = 0; r < 3; ++r)
    for (unsigned int c = 0; c < 3; ++c)
      result.values[r * 3 + c] = (r == c ? 1.0 : 0.0) + (epsilon - 1) * vectors[r * 3 + order[0]] *
                                                            vectors[c * 3 + order[0]];
  output[offset + i] = result;
}
} // namespace resident_detail
using resident_detail::covariance_kernel;
using resident_detail::exact_knn;
using resident_detail::merge_equations;
using resident_detail::Pose;
using resident_detail::resident_equations;
using resident_detail::restore_order;
using resident_detail::transform_resident;

bool available() noexcept {
  int count = 0;
  return cudaGetDeviceCount(&count) == cudaSuccess && count > 0;
}

struct CorrespondenceSearch::Impl {
  TransferStatistics transfers;
  Vec3f* reference{};
  Vec3f* queries{};
  Vec3f* original_reference{};
  Vec3d* reference_normals{};
  Vec3d* scan{};
  Vec3d* scan_normals{};
  Vec3d* transformed{};
  Equation* equations{};
  Equation* equation{};
  std::size_t scan_count{};
  bool has_scan_normals{};
  bool resident{};
  Covariance* covariances{};
  std::int32_t* knn_indices{};
  double* knn_distances{};
  int* invalid_covariance{};
  std::uint32_t knn_capacity{};
  std::uint32_t covariance_neighbors{};
  double covariance_epsilon{};
  std::unique_ptr<CorrespondenceSearch> scan_search;
  SpatialNode* nodes{};
  std::int32_t* original_indices{};
  std::uint32_t node_count{};
  SearchStrategy strategy{SearchStrategy::indexed};
  std::int32_t* indices{};
  cudaStream_t stream{};
  std::size_t reference_count{};
  std::size_t capacity{};
  int device{};

  ~Impl() {
    int previous_device = device;
    const bool restore_device =
        cudaGetDevice(&previous_device) == cudaSuccess && previous_device != device;
    if (restore_device) {
      cudaSetDevice(device);
    }
    // All submissions are synchronized before returning. cudaFree also handles partially
    // constructed contexts; never throw while unwinding an allocation/copy failure.
    if (stream) {
      cudaStreamSynchronize(stream);
    }
    scan_search.reset();
    cudaFree(covariances);
    cudaFree(knn_indices);
    cudaFree(knn_distances);
    cudaFree(invalid_covariance);
    cudaFree(equation);
    cudaFree(equations);
    cudaFree(transformed);
    cudaFree(scan_normals);
    cudaFree(scan);
    cudaFree(reference_normals);
    cudaFree(original_reference);
    if (indices) {
      cudaFree(indices);
    }
    if (queries) {
      cudaFree(queries);
    }
    if (nodes) {
      cudaFree(nodes);
    }
    if (original_indices) {
      cudaFree(original_indices);
    }
    if (reference) {
      cudaFree(reference);
    }
    if (stream) {
      cudaStreamDestroy(stream);
    }
    if (restore_device) {
      cudaSetDevice(previous_device);
    }
  }
};

CorrespondenceSearch::CorrespondenceSearch(std::span<const Vec3f> reference,
                                           std::size_t query_capacity, SearchStrategy strategy)
    : impl_(std::make_unique<Impl>()) {
  if (reference.empty() || query_capacity == 0U ||
      reference.size() > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max()) ||
      query_capacity > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max())) {
    throw std::runtime_error("CUDA correspondence size is unsupported");
  }
  if (strategy != SearchStrategy::indexed && strategy != SearchStrategy::tiled) {
    throw std::invalid_argument("unsupported CUDA search strategy");
  }
  for (const auto point : reference) {
    if (!finite_point(point)) {
      throw std::invalid_argument("CUDA reference coordinates must be finite");
    }
  }
  SpatialIndex index;
  if (strategy == SearchStrategy::indexed) {
    index = build_spatial_index(reference);
  }
  const std::span<const Vec3f> packed =
      strategy == SearchStrategy::indexed ? std::span<const Vec3f>(index.points) : reference;
  impl_->strategy = strategy;
  impl_->reference_count = reference.size();
  impl_->capacity = query_capacity;
  try {
    checked(cudaGetDevice(&impl_->device), "cudaGetDevice");
    checked(cudaStreamCreateWithFlags(&impl_->stream, cudaStreamNonBlocking), "cudaStreamCreate");
    checked(cudaMalloc(reinterpret_cast<void**>(&impl_->reference), reference.size_bytes()),
            "reference allocation");
    checked(cudaMalloc(reinterpret_cast<void**>(&impl_->queries), query_capacity * sizeof(Vec3f)),
            "query allocation");
    checked(cudaMalloc(reinterpret_cast<void**>(&impl_->indices),
                       query_capacity * sizeof(std::int32_t)),
            "index allocation");
    checked(cudaMemcpyAsync(impl_->reference, packed.data(), packed.size_bytes(),
                            cudaMemcpyHostToDevice, impl_->stream),
            "reference upload");
    impl_->transfers.host_to_device_bytes += packed.size_bytes();
    if (strategy == SearchStrategy::indexed) {
      impl_->node_count = static_cast<std::uint32_t>(index.nodes.size());
      checked(cudaMalloc(reinterpret_cast<void**>(&impl_->nodes),
                         index.nodes.size() * sizeof(SpatialNode)),
              "node allocation");
      checked(cudaMalloc(reinterpret_cast<void**>(&impl_->original_indices),
                         index.original_indices.size() * sizeof(std::int32_t)),
              "original index allocation");
      checked(cudaMemcpyAsync(impl_->nodes, index.nodes.data(),
                              index.nodes.size() * sizeof(SpatialNode), cudaMemcpyHostToDevice,
                              impl_->stream),
              "node upload");
      impl_->transfers.host_to_device_bytes += index.nodes.size() * sizeof(SpatialNode);
      checked(cudaMemcpyAsync(impl_->original_indices, index.original_indices.data(),
                              index.original_indices.size() * sizeof(std::int32_t),
                              cudaMemcpyHostToDevice, impl_->stream),
              "original index upload");
      impl_->transfers.host_to_device_bytes += index.original_indices.size() * sizeof(std::int32_t);
    }
    checked(cudaStreamSynchronize(impl_->stream), "reference synchronization");
  } catch (...) {
    // Keep packed host storage alive until every submitted upload has completed, including
    // when a later allocation or copy fails during construction.
    if (impl_->stream) {
      cudaStreamSynchronize(impl_->stream);
    }
    throw;
  }
}

CorrespondenceSearch::~CorrespondenceSearch() = default;

TransferStatistics CorrespondenceSearch::transfer_statistics() const noexcept {
  return impl_->transfers;
}

std::size_t CorrespondenceSearch::device_storage_bytes() const noexcept {
  return (impl_->scan_search ? impl_->scan_search->device_storage_bytes() : 0) +
         (impl_->covariances ? impl_->reference_count * sizeof(Covariance) : 0) +
         (impl_->invalid_covariance ? sizeof(int) : 0) +
         std::min(impl_->reference_count, std::size_t{16384}) * impl_->knn_capacity *
             (sizeof(std::int32_t) + sizeof(double)) +
         impl_->reference_count * ((impl_->resident ? 2 : 1) * sizeof(Vec3f) +
                                   (impl_->reference_normals ? sizeof(Vec3d) : 0)) +
         (impl_->resident
              ? impl_->capacity * 3 * sizeof(Vec3d) +
                    ((impl_->capacity + kBlockSize - 1) / kBlockSize + 1) * sizeof(Equation)
              : 0) +
         impl_->capacity * (sizeof(Vec3f) + sizeof(std::int32_t)) +
         static_cast<std::size_t>(impl_->node_count) * sizeof(SpatialNode) +
         (impl_->strategy == SearchStrategy::indexed ? impl_->reference_count * sizeof(std::int32_t)
                                                     : 0U);
}

std::vector<std::int32_t> CorrespondenceSearch::query(std::span<const Vec3f> points,
                                                      double max_distance_mm) {
  std::vector<std::int32_t> result;
  query_into(points, max_distance_mm, result);
  return result;
}

void CorrespondenceSearch::query_into(std::span<const Vec3f> points, double max_distance_mm,
                                      std::vector<std::int32_t>& result) {
  if (points.size() > impl_->capacity) {
    throw std::runtime_error("CUDA query capacity exceeded");
  }
  if (!std::isfinite(max_distance_mm) || max_distance_mm < 0.0) {
    throw std::invalid_argument("CUDA query radius must be finite and nonnegative");
  }
  for (const auto point : points) {
    if (!finite_point(point)) {
      throw std::invalid_argument("CUDA query coordinates must be finite");
    }
  }
  int device = 0;
  checked(cudaGetDevice(&device), "cudaGetDevice");
  if (device != impl_->device) {
    throw std::runtime_error("CUDA device changed during registration");
  }
  result.resize(points.size());
  if (points.empty()) {
    return;
  }
  try {
    checked(cudaMemcpyAsync(impl_->queries, points.data(), points.size_bytes(),
                            cudaMemcpyHostToDevice, impl_->stream),
            "query upload");
    impl_->transfers.host_to_device_bytes += points.size_bytes();
    const auto blocks = static_cast<unsigned int>((points.size() + kBlockSize - 1U) / kBlockSize);
    if (impl_->strategy == SearchStrategy::indexed) {
      indexed_nearest_kernel<<<blocks, kBlockSize, 0, impl_->stream>>>(
          impl_->reference, impl_->original_indices, impl_->nodes, impl_->node_count,
          impl_->queries, points.size(), max_distance_mm * max_distance_mm, impl_->indices);
    } else {
      nearest_kernel<<<blocks, kBlockSize, 0, impl_->stream>>>(
          impl_->reference, impl_->reference_count, impl_->queries, points.size(),
          max_distance_mm * max_distance_mm, impl_->indices);
    }
    checked(cudaGetLastError(), "nearest kernel launch");
    checked(cudaMemcpyAsync(result.data(), impl_->indices, result.size() * sizeof(std::int32_t),
                            cudaMemcpyDeviceToHost, impl_->stream),
            "index download");
    impl_->transfers.device_to_host_bytes += result.size() * sizeof(std::int32_t);
    checked(cudaStreamSynchronize(impl_->stream), "nearest kernel completion");
  } catch (...) {
    // Drain work before the local result vector can be released on an exceptional return.
    cudaStreamSynchronize(impl_->stream);
    throw;
  }
}

void CorrespondenceSearch::prepare_resident(std::span<const Vec3d> normals) {
  if (impl_->resident || impl_->scan || impl_->reference_normals || impl_->original_reference)
    throw std::invalid_argument("resident workspace already prepared");
  if (!normals.empty() && normals.size() != impl_->reference_count)
    throw std::invalid_argument("reference normal size mismatch");
  try {
    checked(cudaMalloc(reinterpret_cast<void**>(&impl_->original_reference),
                       impl_->reference_count * sizeof(Vec3f)),
            "original reference allocation");
    if (impl_->strategy == SearchStrategy::indexed) {
      const auto blocks =
          static_cast<unsigned int>((impl_->reference_count + kBlockSize - 1) / kBlockSize);
      restore_order<<<blocks, kBlockSize, 0, impl_->stream>>>(
          impl_->reference, impl_->original_indices, impl_->original_reference,
          impl_->reference_count);
      checked(cudaGetLastError(), "original order launch");
    } else
      checked(cudaMemcpyAsync(impl_->original_reference, impl_->reference,
                              impl_->reference_count * sizeof(Vec3f), cudaMemcpyDeviceToDevice,
                              impl_->stream),
              "original reference copy");

    if (!normals.empty()) {
      checked(cudaMalloc(reinterpret_cast<void**>(&impl_->reference_normals), normals.size_bytes()),
              "reference normals allocation");
      checked(cudaMemcpyAsync(impl_->reference_normals, normals.data(), normals.size_bytes(),
                              cudaMemcpyHostToDevice, impl_->stream),
              "reference normals upload");
      impl_->transfers.host_to_device_bytes += normals.size_bytes();
    }
    checked(cudaMalloc(reinterpret_cast<void**>(&impl_->scan), impl_->capacity * sizeof(Vec3d)),
            "scan allocation");
    checked(
        cudaMalloc(reinterpret_cast<void**>(&impl_->scan_normals), impl_->capacity * sizeof(Vec3d)),
        "scan normal allocation");
    checked(
        cudaMalloc(reinterpret_cast<void**>(&impl_->transformed), impl_->capacity * sizeof(Vec3d)),
        "transform allocation");
    checked(cudaMalloc(reinterpret_cast<void**>(&impl_->equations),
                       ((impl_->capacity + kBlockSize - 1) / kBlockSize) * sizeof(Equation)),
            "equation blocks allocation");
    checked(cudaMalloc(reinterpret_cast<void**>(&impl_->equation), sizeof(Equation)),
            "equation allocation");
    checked(cudaStreamSynchronize(impl_->stream), "resident preparation");
    impl_->resident = true;
  } catch (...) {
    cudaStreamSynchronize(impl_->stream);
    throw;
  }
}
void CorrespondenceSearch::upload_scan(std::span<const Vec3d> points,
                                       std::span<const Vec3d> normals) {
  if (!impl_->resident || points.empty() || points.size() > impl_->capacity ||
      (!normals.empty() && normals.size() != points.size()))
    throw std::invalid_argument("invalid resident scan");
  for (auto p : points)
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      throw std::invalid_argument("nonfinite resident scan");
  int device{};
  checked(cudaGetDevice(&device), "cudaGetDevice");
  if (device != impl_->device)
    throw std::runtime_error("CUDA device changed during registration");
  impl_->scan_count = 0;
  try {
    checked(cudaMemcpyAsync(impl_->scan, points.data(), points.size_bytes(), cudaMemcpyHostToDevice,
                            impl_->stream),
            "resident scan upload");
    impl_->transfers.host_to_device_bytes += points.size_bytes();
    if (!normals.empty())
      checked(cudaMemcpyAsync(impl_->scan_normals, normals.data(), normals.size_bytes(),
                              cudaMemcpyHostToDevice, impl_->stream),
              "scan normals upload");
    if (!normals.empty())
      impl_->transfers.host_to_device_bytes += normals.size_bytes();
    checked(cudaStreamSynchronize(impl_->stream), "scan upload completion");
    if (impl_->covariance_neighbors) {
      if (points.size() < impl_->covariance_neighbors)
        throw std::invalid_argument("GICP scan has insufficient support");
      std::vector<Vec3f> packed;
      packed.reserve(points.size());
      for (auto p : points)
        packed.push_back(
            {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)});
      auto scan_search = std::make_unique<CorrespondenceSearch>(packed, 1);
      scan_search->prepare_resident({});
      scan_search->prepare_gicp(impl_->covariance_neighbors, impl_->covariance_epsilon);
      const auto transfers = scan_search->transfer_statistics();
      impl_->transfers.host_to_device_bytes += transfers.host_to_device_bytes;
      impl_->transfers.device_to_host_bytes += transfers.device_to_host_bytes;
      impl_->scan_search = std::move(scan_search);
    }
    impl_->has_scan_normals = !normals.empty();
    impl_->scan_count = points.size();
  } catch (...) {
    cudaStreamSynchronize(impl_->stream);
    throw;
  }
}
Equation CorrespondenceSearch::evaluate(const std::array<double, 16>& matrix, double radius,
                                        double huber, bool plane, bool final_metrics) {
  if (!impl_->resident || impl_->scan_count == 0 || (plane && !impl_->reference_normals) ||
      !std::isfinite(radius) || radius <= 0 || !std::isfinite(huber) || huber <= 0)
    throw std::invalid_argument("invalid resident evaluation");
  Pose pose{};
  for (std::size_t i = 0; i < 16; ++i) {
    if (!std::isfinite(matrix[i]))
      throw std::invalid_argument("nonfinite pose");
    pose.v[i] = matrix[i];
  }
  int device{};
  checked(cudaGetDevice(&device), "cudaGetDevice");
  if (device != impl_->device)
    throw std::runtime_error("CUDA device changed during registration");
  Equation result;
  try {
    const auto blocks =
        static_cast<unsigned int>((impl_->scan_count + kBlockSize - 1) / kBlockSize);
    transform_resident<<<blocks, kBlockSize, 0, impl_->stream>>>(
        impl_->scan, impl_->transformed, impl_->queries, impl_->scan_count, pose);
    checked(cudaGetLastError(), "resident transform launch");
    if (impl_->strategy == SearchStrategy::indexed)
      indexed_nearest_kernel<<<blocks, kBlockSize, 0, impl_->stream>>>(
          impl_->reference, impl_->original_indices, impl_->nodes, impl_->node_count,
          impl_->queries, impl_->scan_count, radius * radius, impl_->indices);
    else
      nearest_kernel<<<blocks, kBlockSize, 0, impl_->stream>>>(
          impl_->reference, impl_->reference_count, impl_->queries, impl_->scan_count,
          radius * radius, impl_->indices);
    checked(cudaGetLastError(), "resident nearest launch");
    resident_equations<<<blocks, kBlockSize, 0, impl_->stream>>>(
        impl_->original_reference, impl_->reference_normals, impl_->transformed, impl_->queries,
        impl_->has_scan_normals ? impl_->scan_normals : nullptr, impl_->indices, impl_->scan_count,
        pose, huber, plane, final_metrics,
        impl_->covariance_neighbors ? impl_->covariances : nullptr,
        impl_->scan_search ? impl_->scan_search->impl_->covariances : nullptr, impl_->equations);
    checked(cudaGetLastError(), "resident objective launch");
    merge_equations<<<1, 64, 0, impl_->stream>>>(impl_->equations, blocks, impl_->equation);
    checked(cudaGetLastError(), "resident merge launch");
    checked(cudaMemcpyAsync(&result, impl_->equation, sizeof(result), cudaMemcpyDeviceToHost,
                            impl_->stream),
            "compact equation download");
    impl_->transfers.device_to_host_bytes += sizeof(result);
    ++impl_->transfers.equation_evaluations;
    checked(cudaStreamSynchronize(impl_->stream), "resident objective completion");
    if (result.values[44] != 0)
      throw std::invalid_argument("transformed coordinate exceeds search precision range");
    return result;
  } catch (...) {
    cudaStreamSynchronize(impl_->stream);
    throw;
  }
}

void CorrespondenceSearch::prepare_knn(std::uint32_t k) {
  if (!impl_->resident || impl_->strategy != SearchStrategy::indexed || k == 0 || k > 1024 ||
      k > impl_->reference_count)
    throw std::invalid_argument("invalid KNN neighborhood");
  if (k <= impl_->knn_capacity)
    return;
  checked(cudaFree(impl_->knn_indices), "KNN resize");
  impl_->knn_indices = nullptr;
  checked(cudaFree(impl_->knn_distances), "KNN resize");
  impl_->knn_distances = nullptr;
  impl_->knn_capacity = 0;
  const auto size = std::min(impl_->reference_count, std::size_t{16384}) * k;
  checked(cudaMalloc(reinterpret_cast<void**>(&impl_->knn_indices), size * sizeof(std::int32_t)),
          "KNN indices allocation");
  checked(cudaMalloc(reinterpret_cast<void**>(&impl_->knn_distances), size * sizeof(double)),
          "KNN distance allocation");
  impl_->knn_capacity = k;
}
std::vector<std::int32_t> CorrespondenceSearch::knn(std::uint32_t k) {
  prepare_knn(k);
  std::vector<std::int32_t> result(impl_->reference_count * k);
  try {
    for (std::size_t offset = 0; offset < impl_->reference_count; offset += 16384) {
      const auto count = std::min(std::size_t{16384}, impl_->reference_count - offset);
      const auto blocks = static_cast<unsigned int>((count + kBlockSize - 1) / kBlockSize);
      exact_knn<<<blocks, kBlockSize, 0, impl_->stream>>>(
          impl_->reference, impl_->original_indices, impl_->nodes, impl_->node_count,
          impl_->original_reference, offset, count, k, impl_->knn_indices, impl_->knn_distances);
      checked(cudaGetLastError(), "exact KNN launch");
      checked(cudaMemcpyAsync(result.data() + offset * k, impl_->knn_indices,
                              count * k * sizeof(std::int32_t), cudaMemcpyDeviceToHost,
                              impl_->stream),
              "KNN oracle download");
      impl_->transfers.device_to_host_bytes += count * k * sizeof(std::int32_t);
      checked(cudaStreamSynchronize(impl_->stream), "KNN completion");
    }
    return result;
  } catch (...) {
    cudaStreamSynchronize(impl_->stream);
    throw;
  }
}
void CorrespondenceSearch::prepare_gicp(std::uint32_t k, double epsilon) {
  if (k < 3 || !std::isfinite(epsilon) || epsilon <= 0 || epsilon > 1 ||
      impl_->covariance_neighbors)
    throw std::invalid_argument("invalid covariance settings");
  prepare_knn(k);
  if (!impl_->covariances)
    checked(cudaMalloc(reinterpret_cast<void**>(&impl_->covariances),
                       impl_->reference_count * sizeof(Covariance)),
            "covariance allocation");
  if (!impl_->invalid_covariance)
    checked(cudaMalloc(reinterpret_cast<void**>(&impl_->invalid_covariance), sizeof(int)),
            "covariance flag allocation");
  int invalid = 0;
  try {
    checked(cudaMemsetAsync(impl_->invalid_covariance, 0, sizeof(int), impl_->stream),
            "covariance flag reset");
    for (std::size_t offset = 0; offset < impl_->reference_count; offset += 16384) {
      const auto count = std::min(std::size_t{16384}, impl_->reference_count - offset);
      const auto blocks = static_cast<unsigned int>((count + kBlockSize - 1) / kBlockSize);
      exact_knn<<<blocks, kBlockSize, 0, impl_->stream>>>(
          impl_->reference, impl_->original_indices, impl_->nodes, impl_->node_count,
          impl_->original_reference, offset, count, k, impl_->knn_indices, impl_->knn_distances);
      checked(cudaGetLastError(), "covariance KNN launch");
      covariance_kernel<<<blocks, kBlockSize, 0, impl_->stream>>>(
          impl_->original_reference, impl_->knn_indices, offset, count, k, epsilon,
          impl_->covariances, impl_->invalid_covariance);
      checked(cudaGetLastError(), "covariance launch");
    }
    checked(cudaMemcpyAsync(&invalid, impl_->invalid_covariance, sizeof(int),
                            cudaMemcpyDeviceToHost, impl_->stream),
            "covariance validity download");
    impl_->transfers.device_to_host_bytes += sizeof(int);
    checked(cudaStreamSynchronize(impl_->stream), "covariance completion");
    if (invalid)
      throw std::invalid_argument("GICP covariance neighborhood is collinear or coincident");
    impl_->covariance_neighbors = k;
    impl_->covariance_epsilon = epsilon;
  } catch (...) {
    cudaStreamSynchronize(impl_->stream);
    throw;
  }
}
std::vector<Covariance> CorrespondenceSearch::download_covariances() {
  if (!impl_->covariance_neighbors)
    throw std::invalid_argument("covariances not prepared");
  std::vector<Covariance> result(impl_->reference_count);
  checked(cudaMemcpy(result.data(), impl_->covariances, result.size() * sizeof(Covariance),
                     cudaMemcpyDeviceToHost),
          "covariance oracle download");
  impl_->transfers.device_to_host_bytes += result.size() * sizeof(Covariance);
  return result;
}

} // namespace pointcloud_ad::backends::cuda_backend
