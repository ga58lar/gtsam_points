#include <gtsam_ext/factors/integrated_loam_factor.hpp>

#include <gtsam/geometry/SO3.h>
#include <gtsam_ext/types/kdtree.hpp>

namespace gtsam_ext {

IntegratedPointToPlaneFactor::IntegratedPointToPlaneFactor(gtsam::Key target_key, gtsam::Key source_key, const Frame::ConstPtr& target, const Frame::ConstPtr& source)
: gtsam_ext::IntegratedMatchingCostFactor(target_key, source_key),
  num_threads(1),
  max_correspondence_distance_sq(1.0),
  target(target),
  source(source) {
  //
  if (!target->points || !source->points) {
    std::cerr << "error: target or source points has not been allocated!!" << std::endl;
    abort();
  }

  target_tree.reset(new KdTree(target->num_points, target->points));
}

IntegratedPointToPlaneFactor::~IntegratedPointToPlaneFactor() {}

void IntegratedPointToPlaneFactor::update_correspondences(const Eigen::Isometry3d& delta) const {
  correspondences.resize(source->size());

#pragma omp parallel for num_threads(num_threads) schedule(guided, 8)
  for (int i = 0; i < source->size(); i++) {
    Eigen::Vector4d pt = delta * source->points[i];

    std::array<size_t, 3> k_indices;
    std::array<double, 3> k_sq_dists;
    target_tree->index.knnSearch(pt.data(), 3, k_indices.data(), k_sq_dists.data());

    if (k_sq_dists.back() > max_correspondence_distance_sq) {
      correspondences[i] = std::make_tuple(-1, -1, -1);
    } else {
      correspondences[i] = std::make_tuple(k_indices[0], k_indices[1], k_indices[2]);
    }
  }
}

double IntegratedPointToPlaneFactor::evaluate(
  const Eigen::Isometry3d& delta,
  Eigen::Matrix<double, 6, 6>* H_target,
  Eigen::Matrix<double, 6, 6>* H_source,
  Eigen::Matrix<double, 6, 6>* H_target_source,
  Eigen::Matrix<double, 6, 1>* b_target,
  Eigen::Matrix<double, 6, 1>* b_source) const {
  //
  if (correspondences.size() != source->size()) {
    update_correspondences(delta);
  }

  //
  double sum_errors = 0.0;

  std::vector<Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> Hs_target;
  std::vector<Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> Hs_source;
  std::vector<Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> Hs_target_source;
  std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>> bs_target;
  std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>> bs_source;

  if (H_target && H_source && H_target_source && b_target && b_source) {
    Hs_target.resize(num_threads, Eigen::Matrix<double, 6, 6>::Zero());
    Hs_source.resize(num_threads, Eigen::Matrix<double, 6, 6>::Zero());
    Hs_target_source.resize(num_threads, Eigen::Matrix<double, 6, 6>::Zero());
    bs_target.resize(num_threads, Eigen::Matrix<double, 6, 1>::Zero());
    bs_source.resize(num_threads, Eigen::Matrix<double, 6, 1>::Zero());
  }

#pragma omp parallel for num_threads(num_threads) reduction(+ : sum_errors) schedule(guided, 8)
  for (int i = 0; i < source->size(); i++) {
    auto target_indices = correspondences[i];
    if (std::get<0>(target_indices) < 0) {
      continue;
    }

    const auto& x_i = source->points[i];
    const auto& x_j = target->points[std::get<0>(target_indices)];
    const auto& x_l = target->points[std::get<1>(target_indices)];
    const auto& x_m = target->points[std::get<2>(target_indices)];

    Eigen::Vector4d transed_x_i = delta * x_i;
    Eigen::Vector4d normal = Eigen::Vector4d::Zero();
    normal.head<3>() = (x_j - x_l).head<3>().cross((x_j - x_m).head<3>());
    normal = normal / normal.norm();

    Eigen::Vector4d error = x_j - transed_x_i;
    Eigen::Vector4d plane_error = error.array() * normal.array();
    sum_errors += 0.5 * plane_error.transpose() * plane_error;

    if (Hs_target.empty()) {
      continue;
    }

    Eigen::Matrix<double, 4, 6> J_target = Eigen::Matrix<double, 4, 6>::Zero();
    J_target.block<3, 3>(0, 0) = -gtsam::SO3::Hat(transed_x_i.head<3>());
    J_target.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 4, 6> J_source = Eigen::Matrix<double, 4, 6>::Zero();
    J_source.block<3, 3>(0, 0) = delta.linear() * gtsam::SO3::Hat(x_i.head<3>());
    J_source.block<3, 3>(0, 3) = -delta.linear();

    J_target = normal.asDiagonal() * J_target;
    J_source = normal.asDiagonal() * J_source;

    int thread_num = 0;
#ifdef _OPENMP
    thread_num = omp_get_thread_num();
#endif

    Hs_target[thread_num] += J_target.transpose() * J_target;
    Hs_source[thread_num] += J_source.transpose() * J_source;
    Hs_target_source[thread_num] += J_target.transpose() * J_source;
    bs_target[thread_num] += J_target.transpose() * plane_error;
    bs_source[thread_num] += J_source.transpose() * plane_error;
  }

  if (H_target && H_source && H_target_source && b_target && b_source) {
    H_target->setZero();
    H_source->setZero();
    H_target_source->setZero();
    b_target->setZero();
    b_source->setZero();

    for (int i = 0; i < num_threads; i++) {
      (*H_target) += Hs_target[i];
      (*H_source) += Hs_source[i];
      (*H_target_source) += Hs_target_source[i];
      (*b_target) += bs_target[i];
      (*b_source) += bs_source[i];
    }
  }

  return sum_errors;
}

IntegratedPointToEdgeFactor::IntegratedPointToEdgeFactor(gtsam::Key target_key, gtsam::Key source_key, const Frame::ConstPtr& target, const Frame::ConstPtr& source)
: gtsam_ext::IntegratedMatchingCostFactor(target_key, source_key),
  num_threads(1),
  max_correspondence_distance_sq(1.0),
  target(target),
  source(source) {
  //
  if (!target->points || !source->points) {
    std::cerr << "error: target or source points has not been allocated!!" << std::endl;
    abort();
  }

  target_tree.reset(new KdTree(target->num_points, target->points));
}

IntegratedPointToEdgeFactor::~IntegratedPointToEdgeFactor() {}

void IntegratedPointToEdgeFactor::update_correspondences(const Eigen::Isometry3d& delta) const {
  correspondences.resize(source->size());

#pragma omp parallel for num_threads(num_threads) schedule(guided, 8)
  for (int i = 0; i < source->size(); i++) {
    Eigen::Vector4d pt = delta * source->points[i];

    std::array<size_t, 2> k_indices;
    std::array<double, 2> k_sq_dists;
    target_tree->index.knnSearch(pt.data(), 2, k_indices.data(), k_sq_dists.data());

    if (k_sq_dists.back() > max_correspondence_distance_sq) {
      correspondences[i] = std::make_tuple(-1, -1);
    } else {
      correspondences[i] = std::make_tuple(k_indices[0], k_indices[1]);
    }
  }
}

double IntegratedPointToEdgeFactor::evaluate(
  const Eigen::Isometry3d& delta,
  Eigen::Matrix<double, 6, 6>* H_target,
  Eigen::Matrix<double, 6, 6>* H_source,
  Eigen::Matrix<double, 6, 6>* H_target_source,
  Eigen::Matrix<double, 6, 1>* b_target,
  Eigen::Matrix<double, 6, 1>* b_source) const {
  //
  if (correspondences.size() != source->size()) {
    update_correspondences(delta);
  }

  //
  double sum_errors = 0.0;

  std::vector<Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> Hs_target;
  std::vector<Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> Hs_source;
  std::vector<Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> Hs_target_source;
  std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>> bs_target;
  std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>> bs_source;

  if (H_target && H_source && H_target_source && b_target && b_source) {
    Hs_target.resize(num_threads, Eigen::Matrix<double, 6, 6>::Zero());
    Hs_source.resize(num_threads, Eigen::Matrix<double, 6, 6>::Zero());
    Hs_target_source.resize(num_threads, Eigen::Matrix<double, 6, 6>::Zero());
    bs_target.resize(num_threads, Eigen::Matrix<double, 6, 1>::Zero());
    bs_source.resize(num_threads, Eigen::Matrix<double, 6, 1>::Zero());
  }

#pragma omp parallel for num_threads(num_threads) reduction(+ : sum_errors) schedule(guided, 8)
  for (int i = 0; i < source->size(); i++) {
    auto target_indices = correspondences[i];
    if (std::get<0>(target_indices) < 0) {
      continue;
    }

    const auto& x_i = source->points[i];
    const auto& x_j = target->points[std::get<0>(target_indices)];
    const auto& x_l = target->points[std::get<1>(target_indices)];

    Eigen::Vector4d transed_x_i = delta * x_i;

    double c_inv = 1.0 / (x_j - x_l).norm();
    Eigen::Vector4d x_ij = transed_x_i - x_j;
    Eigen::Vector4d x_il = transed_x_i - x_l;

    Eigen::Vector4d error = Eigen::Vector4d::Zero();
    error.head<3>() = x_ij.head<3>().cross(x_il.head<3>()) * c_inv;
    sum_errors += 0.5 * error.transpose() * error;

    if (Hs_target.empty()) {
      continue;
    }

    Eigen::Matrix<double, 4, 6> J_target = Eigen::Matrix<double, 4, 6>::Zero();
    J_target.block<3, 3>(0, 0) = -gtsam::SO3::Hat(transed_x_i.head<3>());
    J_target.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 4, 6> J_source = Eigen::Matrix<double, 4, 6>::Zero();
    J_source.block<3, 3>(0, 0) = delta.linear() * gtsam::SO3::Hat(x_i.head<3>());
    J_source.block<3, 3>(0, 3) = -delta.linear();

    Eigen::Matrix<double, 4, 4> J_e = Eigen::Matrix4d::Zero();
    J_e.block<1, 3>(0, 0) << 0.0, -x_j[2] + x_l[2], x_j[1] - x_l[1];
    J_e.block<1, 3>(1, 0) << x_j[2] - x_l[2], 0.0, -x_j[0] + x_l[0];
    J_e.block<1, 3>(2, 0) << -x_j[1] + x_l[1], x_j[0] - x_l[0], 0.0;

    J_target = c_inv * J_e * J_target;
    J_source = c_inv * J_e * J_source;

    int thread_num = 0;
#ifdef _OPENMP
    thread_num = omp_get_thread_num();
#endif

    Hs_target[thread_num] += J_target.transpose() * J_target;
    Hs_source[thread_num] += J_source.transpose() * J_source;
    Hs_target_source[thread_num] += J_target.transpose() * J_source;
    bs_target[thread_num] += J_target.transpose() * error;
    bs_source[thread_num] += J_source.transpose() * error;
  }

  if (H_target && H_source && H_target_source && b_target && b_source) {
    H_target->setZero();
    H_source->setZero();
    H_target_source->setZero();
    b_target->setZero();
    b_source->setZero();

    for (int i = 0; i < num_threads; i++) {
      (*H_target) += Hs_target[i];
      (*H_source) += Hs_source[i];
      (*H_target_source) += Hs_target_source[i];
      (*b_target) += bs_target[i];
      (*b_source) += bs_source[i];
    }
  }

  return sum_errors;
}

IntegratedLOAMFactor::IntegratedLOAMFactor(
  gtsam::Key target_key,
  gtsam::Key source_key,
  const Frame::ConstPtr& target_edges,
  const Frame::ConstPtr& target_planes,
  const Frame::ConstPtr& source_edges,
  const Frame::ConstPtr& source_planes)
: gtsam_ext::IntegratedMatchingCostFactor(target_key, source_key),
  enable_correspondence_validation(false) {
  //
  edge_factor.reset(new IntegratedPointToEdgeFactor(target_key, source_key, target_edges, source_edges));
  plane_factor.reset(new IntegratedPointToPlaneFactor(target_key, source_key, target_planes, source_planes));
}

IntegratedLOAMFactor::~IntegratedLOAMFactor() {}

void IntegratedLOAMFactor::set_num_threads(int n) {
  edge_factor->set_num_threads(n);
  plane_factor->set_num_threads(n);
}

void IntegratedLOAMFactor::set_max_corresponding_distance(double dist_edge, double dist_plane) {
  edge_factor->set_max_corresponding_distance(dist_edge);
  plane_factor->set_max_corresponding_distance(dist_plane);
}

void IntegratedLOAMFactor::set_enable_correspondence_validation(bool enable) {
  enable_correspondence_validation = enable;
}

void IntegratedLOAMFactor::update_correspondences(const Eigen::Isometry3d& delta) const {
  edge_factor->update_correspondences(delta);
  plane_factor->update_correspondences(delta);

  validate_correspondences();
}

double IntegratedLOAMFactor::evaluate(
  const Eigen::Isometry3d& delta,
  Eigen::Matrix<double, 6, 6>* H_target,
  Eigen::Matrix<double, 6, 6>* H_source,
  Eigen::Matrix<double, 6, 6>* H_target_source,
  Eigen::Matrix<double, 6, 1>* b_target,
  Eigen::Matrix<double, 6, 1>* b_source) const {
  //

  double error = edge_factor->evaluate(delta, H_target, H_source, H_target_source, b_target, b_source);

  if(H_target && H_source && H_target_source && b_target && b_source) {
    Eigen::Matrix<double, 6, 6> H_t, H_s, H_ts;
    Eigen::Matrix<double, 6, 1> b_t, b_s;
    error += plane_factor->evaluate(delta, &H_t, &H_s, &H_ts, &b_t, &b_s);

    (*H_target) += H_t;
    (*H_source) += H_s;
    (*H_target_source) += H_ts;
    (*b_target) += b_t;
    (*b_source) += b_s;
  } else {
    error += plane_factor->evaluate(delta);
  }

  return error;
}

void IntegratedLOAMFactor::validate_correspondences() const {
  if(!enable_correspondence_validation) {
    return;
  }

  for (auto& corr : edge_factor->correspondences) {
    if(std::get<0>(corr) < 0) {
      continue;
    }

    const auto& pt1 = edge_factor->target->points[std::get<0>(corr)];
    const auto& pt2 = edge_factor->target->points[std::get<1>(corr)];

    const double v_theta1 = std::atan2(pt1.z(), pt1.head<2>().norm());
    const double v_theta2 = std::atan2(pt2.z(), pt2.head<2>().norm());

    // Reject pairs in a same scan line
    if(std::abs(v_theta1 - v_theta2) < 0.1 * M_PI / 180.0) {
      corr = std::make_pair(-1, -1);
    }
  }
}
}