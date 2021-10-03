#pragma once

#include <vector>
#include <Eigen/Core>

#include <gtsam_ext/types/frame.hpp>

// forward declaration
namespace thrust {

template <typename T>
class device_allocator;

template <typename T, typename Alloc>
class device_vector;

}  // namespace thrust

namespace gtsam_ext {

struct FrameGPU : public Frame {
public:
  using Ptr = std::shared_ptr<FrameGPU>;
  using ConstPtr = std::shared_ptr<const FrameGPU>;

  template <typename T, int D>
  FrameGPU(const std::vector<Eigen::Matrix<T, D, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, D, 1>>>& points, bool allocate_cpu = true);
  ~FrameGPU();

  // add_*_gpu() only adds attributes to GPU storage
  template <typename T>
  void add_times(const std::vector<T>& times);
  template <typename T>
  void add_times_gpu(const std::vector<T>& times);

  template <typename T, int D>
  void add_normals(const std::vector<Eigen::Matrix<T, D, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, D, 1>>>& normals);
  template <typename T, int D>
  void add_normals_gpu(const std::vector<Eigen::Matrix<T, D, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, D, 1>>>& normals);

  template <typename T, int D>
  void add_covs(const std::vector<Eigen::Matrix<T, D, D>, Eigen::aligned_allocator<Eigen::Matrix<T, D, D>>>& covs);
  template <typename T, int D>
  void add_covs_gpu(const std::vector<Eigen::Matrix<T, D, D>, Eigen::aligned_allocator<Eigen::Matrix<T, D, D>>>& covs);

public:
  using FloatsGPU = thrust::device_vector<float, thrust::device_allocator<float>>;
  using PointsGPU = thrust::device_vector<Eigen::Vector3f, thrust::device_allocator<Eigen::Vector3f>>;
  using MatricesGPU = thrust::device_vector<Eigen::Matrix3f, thrust::device_allocator<Eigen::Matrix3f>>;

  std::vector<double> times_storage;
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points_storage;
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> normals_storage;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> covs_storage;

  std::unique_ptr<FloatsGPU> times_gpu_storage;
  std::unique_ptr<PointsGPU> points_gpu_storage;
  std::unique_ptr<PointsGPU> normals_gpu_storage;
  std::unique_ptr<MatricesGPU> covs_gpu_storage;
};

}  // namespace gtsam_ext