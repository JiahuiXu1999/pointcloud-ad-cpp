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

bool available() noexcept {
  int count = 0;
  return cudaGetDeviceCount(&count) == cudaSuccess && count > 0;
}

struct CorrespondenceSearch::Impl {
  Vec3f* reference{};
  Vec3f* queries{};
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
      checked(cudaMemcpyAsync(impl_->original_indices, index.original_indices.data(),
                              index.original_indices.size() * sizeof(std::int32_t),
                              cudaMemcpyHostToDevice, impl_->stream),
              "original index upload");
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

std::size_t CorrespondenceSearch::device_storage_bytes() const noexcept {
  return impl_->reference_count * sizeof(Vec3f) +
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
    checked(cudaStreamSynchronize(impl_->stream), "nearest kernel completion");
  } catch (...) {
    // Drain work before the local result vector can be released on an exceptional return.
    cudaStreamSynchronize(impl_->stream);
    throw;
  }
}

} // namespace pointcloud_ad::backends::cuda_backend
