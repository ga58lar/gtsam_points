// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
#include <gtsam_ext/cuda/cuda_memory.hpp>

#include <cuda_runtime.h>
#include <gtsam_ext/cuda/check_error.cuh>

namespace gtsam_ext {

void* cuda_malloc(size_t size, CUstream_st* stream) {
  void* ptr = nullptr;
  check_error << cudaMallocAsync(&ptr, size, stream);
  return ptr;
}

void* cuda_malloc_and_upload(const void* data, size_t size, CUstream_st* stream) {
  void* ptr = nullptr;
  check_error << cudaMallocAsync(&ptr, size, stream);
  check_error << cudaMemcpyAsync(ptr, data, size, cudaMemcpyHostToDevice, stream);
  return ptr;
}

void cuda_free(void* ptr, CUstream_st* stream) {
  check_error << cudaFreeAsync(ptr, stream);
}

void cuda_host_to_device(void* dst, const void* src, size_t size, CUstream_st* stream) {
  check_error << cudaMemcpyAsync(dst, src, size, cudaMemcpyHostToDevice, stream);
}

void cuda_device_to_host(void* dst, const void* src, size_t size, CUstream_st* stream) {
  check_error << cudaMemcpyAsync(dst, src, size, cudaMemcpyDeviceToHost, stream);
}

}  // namespace gtsam_ext