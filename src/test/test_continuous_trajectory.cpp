#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam_ext/util/expressions.hpp>
#include <gtsam_ext/util/bspline.hpp>
#include <gtsam_ext/util/continuous_trajectory.hpp>

using gtsam::symbol_shorthand::X;

struct ContinuousTrajectoryTestBase : public testing::Test {
public:
  virtual void SetUp() {
    std::mt19937 mt(8192 - 3);

    for (double t = 0.0; t < 10.0; t += 0.5) {
      gtsam::Vector6 tan;
      for (int i = 0; i < 6; i++) {
        tan[i] = std::uniform_real_distribution<>(-0.5, 0.5)(mt);
      }
      tan[3] += t;

      stamps.push_back(t);
      poses.push_back(gtsam::Pose3::Expmap(tan));
    }

    values.reset(new gtsam::Values);
    ct.reset(new gtsam_ext::ContinuousTrajectory('x', stamps.front(), stamps.back(), 0.2));
    *values = ct->fit_knots(stamps, poses);
  }

  void check_pose_error(const gtsam::Pose3& x0, const gtsam::Pose3& x1, const std::string& label, const double thresh = 1e-1) {
    const gtsam::Pose3 error = x0.inverse() * x1;
    const double error_t = error.translation().norm();
    const double error_r = Eigen::AngleAxisd(error.rotation().matrix()).angle();

    EXPECT_LT(error_t, thresh) << "[" << label << "] Too large translation error";
    EXPECT_LT(error_r, thresh) << "[" << label << "] Too large rotation error";
  }

  void check_error(const Eigen::VectorXd& x0, const Eigen::VectorXd& x1, const std::string& label, const double thresh = 1e-1) {  //
    EXPECT_LT((x0 - x1).array().abs().maxCoeff(), thresh) << "[" << label << "] Too large error " << x0.transpose() << " vs " << x1.transpose();
  }

public:
  std::vector<double> stamps;
  std::vector<gtsam::Pose3> poses;

  std::unique_ptr<gtsam::Values> values;
  std::unique_ptr<gtsam_ext::ContinuousTrajectory> ct;
};

TEST_F(ContinuousTrajectoryTestBase, FittingTest) {
  for (int i = 0; i < stamps.size(); i++) {
    const auto pose = ct->pose(*values, stamps[i]);
    check_pose_error(pose, poses[i], "Fitting");
  }
}

TEST_F(ContinuousTrajectoryTestBase, InterpolationTest) {
  values->insert(0, 0.0);
  for (double t = stamps.front(); t < stamps.back(); t += 0.05) {
    const auto pose = ct->pose(*values, t);

    values->update(0, t);
    const int knot_i = ct->knot_id(t);
    const double knot_t = ct->knot_stamp(knot_i);

    const gtsam::Double_ p = (1.0 / ct->knot_interval) * (gtsam::Double_(gtsam::Key(0)) - gtsam::Double_(knot_t));
    const auto pose0_ = gtsam_ext::bspline(X(knot_i), p);
    const auto pose0 = pose0_.value(*values);
    check_pose_error(pose, pose0, "Interpolation");

    const auto rot0_ = gtsam_ext::bspline_so3(  //
      gtsam::rotation(X(knot_i - 1)),
      gtsam::rotation(X(knot_i)),
      gtsam::rotation(X(knot_i + 1)),
      gtsam::rotation(X(knot_i + 2)),
      p);
    std::vector<gtsam::Matrix> Hs_rot0(rot0_.keys().size());
    const auto rot0 = rot0_.value(*values, Hs_rot0);

    const auto trans0_ = gtsam_ext::bspline_trans(
      gtsam_ext::translation(X(knot_i - 1)),
      gtsam_ext::translation(X(knot_i)),
      gtsam_ext::translation(X(knot_i + 1)),
      gtsam_ext::translation(X(knot_i + 2)),
      p);
    std::vector<gtsam::Matrix> Hs_trans0(trans0_.keys().size());
    const auto trans0 = trans0_.value(*values, Hs_trans0);
    check_pose_error(pose, gtsam::Pose3(rot0, trans0), "Independent");

    // Derivatives
    const auto dr_dt_ = gtsam_ext::bspline_angular_vel(  //
      gtsam::rotation(X(knot_i - 1)),
      gtsam::rotation(X(knot_i)),
      gtsam::rotation(X(knot_i + 1)),
      gtsam::rotation(X(knot_i + 2)),
      p);
    const auto dr_dt = dr_dt_.value(*values);
    check_error(dr_dt, Hs_rot0.front() * ct->knot_interval, "Angular vel", 5e-2);

    const auto dt_dt_ = gtsam_ext::bspline_linear_vel(
      gtsam_ext::translation(X(knot_i - 1)),
      gtsam_ext::translation(X(knot_i)),
      gtsam_ext::translation(X(knot_i + 1)),
      gtsam_ext::translation(X(knot_i + 2)),
      p);

    std::vector<gtsam::Matrix> Hs_tvel(dt_dt_.keys().size());
    const auto dt_dt = dt_dt_.value(*values, Hs_tvel);
    check_error(dt_dt / ct->knot_interval, Hs_trans0.front(), "Linear vel", 5e-2);

    const auto dt_dt2_ = gtsam_ext::bspline_linear_acc(
      gtsam_ext::translation(X(knot_i - 1)),
      gtsam_ext::translation(X(knot_i)),
      gtsam_ext::translation(X(knot_i + 1)),
      gtsam_ext::translation(X(knot_i + 2)),
      p);
    const auto dt_dt2 = dt_dt2_.value(*values);
    check_error(dt_dt2 / ct->knot_interval, Hs_tvel.front(), "Linear acc", 5e-2);

    const Eigen::Vector3d g(0.0, 0.0, 9.80665);
    const auto imu_ = gtsam_ext::bspline_imu(  //
      gtsam::Pose3_(X(knot_i - 1)),
      gtsam::Pose3_(X(knot_i)),
      gtsam::Pose3_(X(knot_i + 1)),
      gtsam::Pose3_(X(knot_i + 2)),
      p,
      g);
    const auto imu = imu_.value(*values);
    gtsam::Vector6 imu2 = (gtsam::Vector6() << rot0.unrotate(dt_dt2 + g), rot0.unrotate(dr_dt)).finished();
    check_error(imu, imu2, "IMU");
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}