#include <gtsam_ext/factors/integrated_matching_cost_factor.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/HessianFactor.h>

namespace gtsam_ext {

IntegratedMatchingCostFactor::IntegratedMatchingCostFactor(gtsam::Key target_key, gtsam::Key source_key)
: gtsam::NonlinearFactor(gtsam::cref_list_of<2>(target_key)(source_key)),
  is_binary(true),
  fixed_target_pose(Eigen::Isometry3d::Identity()) {}

IntegratedMatchingCostFactor::~IntegratedMatchingCostFactor() {}

double IntegratedMatchingCostFactor::error(const gtsam::Values& values) const {
  Eigen::Isometry3d delta = calc_delta(values);
  return evaluate(delta);
}

boost::shared_ptr<gtsam::GaussianFactor> IntegratedMatchingCostFactor::linearize(const gtsam::Values& values) const {
  Eigen::Isometry3d delta = calc_delta(values);

  update_correspondences(delta);

  Eigen::Matrix<double, 6, 6> H_target, H_source, H_target_source;
  Eigen::Matrix<double, 6, 1> b_target, b_source;
  double error = evaluate(delta, &H_target, &H_source, &H_target_source, &b_target, &b_source);

  gtsam::HessianFactor::shared_ptr factor;

  if (is_binary) {
    factor.reset(new gtsam::HessianFactor(keys()[0], keys()[1], H_target, H_target_source, -b_target, H_source, -b_source, 0.0));
  } else {
    factor.reset(new gtsam::HessianFactor(keys()[0], H_source, -b_source, 0.0));
  }

  return factor;
}

Eigen::Isometry3d IntegratedMatchingCostFactor::calc_delta(const gtsam::Values& values) const {
  if (is_binary) {
    gtsam::Pose3 target_pose = values.at<gtsam::Pose3>(keys()[0]);
    gtsam::Pose3 source_pose = values.at<gtsam::Pose3>(keys()[1]);
    Eigen::Isometry3d delta((target_pose.inverse() * source_pose).matrix());
    return delta;
  } else {
    gtsam::Pose3 target_pose(fixed_target_pose.matrix());
    gtsam::Pose3 source_pose = values.at<gtsam::Pose3>(keys()[1]);
    Eigen::Isometry3d delta((target_pose.inverse() * source_pose).matrix());
    return delta;
  }
}

}  // namespace gtsam_ext