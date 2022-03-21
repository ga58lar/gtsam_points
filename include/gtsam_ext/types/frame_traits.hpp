// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)

#pragma once

#include <iostream>
#include <Eigen/Core>

namespace gtsam_ext {

namespace frame {

template <typename T>
struct traits {};

// int size(const T& t)
template <typename T, typename = void>
struct size_defined : std::false_type {};

template <typename T>
struct size_defined<T, std::enable_if_t<std::is_invocable_v<decltype(&traits<T>::size), const T&>>> : std::true_type {};

template <typename T>
std::enable_if_t<size_defined<T>::value, int> size(const T& t) {
  return traits<T>::size(t);
}

template <typename T>
std::enable_if_t<!size_defined<T>::value, int> size(const T& t) {
  std::cerr << "warning: calling frame::size() for unsupported class" << std::endl;
  return 0;
}

// bool has_times(const T& t)
template <typename T, typename = void>
struct has_times_defined : std::false_type {};

template <typename T>
struct has_times_defined<T, std::enable_if_t<std::is_invocable_v<decltype(&traits<T>::has_times), const T&>>> : std::true_type {};

template <typename T>
std::enable_if_t<has_times_defined<T>::value, bool> has_times(const T& t) {
  return traits<T>::has_times(t);
}

template <typename T>
std::enable_if_t<!has_times_defined<T>::value, bool> has_times(const T& t) {
  return false;
}

// bool has_points(const T& t)
template <typename T, typename = void>
struct has_points_defined : std::false_type {};

template <typename T>
struct has_points_defined<T, std::enable_if_t<std::is_invocable_v<decltype(&traits<T>::has_points), const T&>>> : std::true_type {};

template <typename T>
std::enable_if_t<has_points_defined<T>::value, bool> has_points(const T& t) {
  return traits<T>::has_points(t);
}

template <typename T>
std::enable_if_t<!has_points_defined<T>::value, bool> has_points(const T& t) {
  return false;
}

// bool has_normals(const T& t)
template <typename T, typename = void>
struct has_normals_defined : std::false_type {};

template <typename T>
struct has_normals_defined<T, std::enable_if_t<std::is_invocable_v<decltype(&traits<T>::has_normals), const T&>>> : std::true_type {};

template <typename T>
std::enable_if_t<has_normals_defined<T>::value, bool> has_normals(const T& t) {
  return traits<T>::has_normals(t);
}

template <typename T>
std::enable_if_t<!has_normals_defined<T>::value, bool> has_normals(const T& t) {
  return false;
}

// bool has_covs(const T& t)
template <typename T, typename = void>
struct has_covs_defined : std::false_type {};

template <typename T>
struct has_covs_defined<T, std::enable_if_t<std::is_invocable_v<decltype(&traits<T>::has_covs), const T&>>> : std::true_type {};

template <typename T>
std::enable_if_t<has_covs_defined<T>::value, bool> has_covs(const T& t) {
  return traits<T>::has_covs(t);
}

template <typename T>
std::enable_if_t<!has_covs_defined<T>::value, bool> has_covs(const T& t) {
  return false;
}

// bool has_intensities(const T& t)
template <typename T, typename = void>
struct has_intensities_defined : std::false_type {};

template <typename T>
struct has_intensities_defined<T, std::enable_if_t<std::is_invocable_v<decltype(&traits<T>::has_intensities), const T&>>> : std::true_type {};

template <typename T>
std::enable_if_t<has_intensities_defined<T>::value, bool> has_intensities(const T& t) {
  return traits<T>::has_intensities(t);
}

template <typename T>
std::enable_if_t<!has_intensities_defined<T>::value, bool> has_intensities(const T& t) {
  return false;
}

// Point accessors
template <typename T>
double time(const T& t, size_t i) {
  return traits<T>::time(t, i);
}

template <typename T>
auto point(const T& t, size_t i) {
  return traits<T>::point(t, i);
}

template <typename T>
auto normal(const T& t, size_t i) {
  return traits<T>::normal(t, i);
}

template <typename T>
auto cov(const T& t, size_t i) {
  return traits<T>::cov(t, i);
}

template <typename T>
auto intensity(const T& t, size_t i) {
  return traits<T>::intensity(t, i);
}

template <typename T>
auto time_gpu(const T& t, size_t i) {
  return traits<T>::time_gpu(t, i);
}

template <typename T>
auto point_gpu(const T& t, size_t i) {
  return traits<T>::point_gpu(t, i);
}

template <typename T>
auto normal_gpu(const T& t, size_t i) {
  return traits<T>::normal_gpu(t, i);
}

template <typename T>
auto cov_gpu(const T& t, size_t i) {
  return traits<T>::cov_gpu(t, i);
}

template <typename T>
auto intensity_gpu(const T& t, size_t i) {
  return traits<T>::intensity_gpu(t, i);
}

// low-level interface
template <typename T, typename = void>
struct points_ptr_defined : std::false_type {};

template <typename T>
struct points_ptr_defined<T, std::enable_if_t<std::is_invocable_v<decltype(&traits<T>::points_ptr), const T&>>> : std::true_type {};

template <typename T>
std::enable_if_t<points_ptr_defined<T>::value, const Eigen::Vector4d*> points_ptr(const T& t) {
  return traits<T>::points_ptr(t);
}

template <typename T>
std::enable_if_t<!points_ptr_defined<T>::value, const Eigen::Vector4d*> points_ptr(const T& t) {
  std::cerr << "warning: calling frame::points_ptr() for unsupported class" << std::endl;
  return nullptr;
}

}  // namespace frame

}  // namespace gtsam_ext