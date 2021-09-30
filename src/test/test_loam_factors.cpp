#include <random>
#include <string>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/format.hpp>

#include <gtest/gtest.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam_ext/types/frame_cpu.hpp>
#include <gtsam_ext/factors/integrated_loam_factor.hpp>
#include <gtsam_ext/optimizers/levenberg_marquardt_ext.hpp>

struct LOAMTestBase : public testing::Test {
  virtual void SetUp() {
    const std::string data_path = "./data/newer_01";

    std::ifstream ifs(data_path + "/graph.txt");
    EXPECT_EQ(ifs.is_open(), true) << "Failed to open " << data_path << "/graph.txt";

    for (int i = 0; i < 5; i++) {
      // Read poses
      std::string token;
      Eigen::Vector3d trans;
      Eigen::Quaterniond quat;
      ifs >> token >> trans.x() >> trans.y() >> trans.z() >> quat.x() >> quat.y() >> quat.z() >> quat.w();

      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.translation() = trans;
      pose.linear() = quat.toRotationMatrix();
      poses.insert(i, gtsam::Pose3::identity());
      poses_gt.insert(i, gtsam::Pose3(pose.matrix()));

      // Load points
      const std::string edge_path = (boost::format("%s/edges_%06d.bin") % data_path % (i * 10)).str();
      const std::string plane_path = (boost::format("%s/planes_%06d.bin") % data_path % (i * 10)).str();

      edge_frames.push_back(read_points(edge_path));
      plane_frames.push_back(read_points(plane_path));
    }
  }

  gtsam_ext::Frame::Ptr read_points(const std::string& path) {
    std::ifstream ifs(path, std::ios::binary | std::ios::ate);
    EXPECT_EQ(ifs.is_open(), true) << "Failed to open " << path;

    std::streamsize points_bytes = ifs.tellg();
    size_t num_points = points_bytes / (sizeof(Eigen::Vector3f));

    ifs.seekg(0, std::ios::beg);
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points;
    points.resize(num_points);
    ifs.read(reinterpret_cast<char*>(points.data()), sizeof(Eigen::Vector3f) * num_points);

    return gtsam_ext::Frame::Ptr(new gtsam_ext::FrameCPU(points));
  }

  std::vector<gtsam_ext::Frame::Ptr> edge_frames;
  std::vector<gtsam_ext::Frame::Ptr> plane_frames;
  gtsam::Values poses;
  gtsam::Values poses_gt;
};

TEST_F(LOAMTestBase, LoadCheck) {
  EXPECT_EQ(edge_frames.size(), 5) << "Failed to load edge points";
  EXPECT_EQ(plane_frames.size(), 5) << "Failed to load plane points";
  EXPECT_EQ(poses.size(), 5) << "Failed to load GT poses";
  EXPECT_EQ(poses_gt.size(), 5) << "Failed to load GT poses";
}

class FactorTest : public LOAMTestBase, public testing::WithParamInterface<std::string> {
public:
  gtsam::NonlinearFactor::shared_ptr create_factor(
    gtsam::Key target_key,
    gtsam::Key source_key,
    const gtsam_ext::Frame::ConstPtr& target_edges,
    const gtsam_ext::Frame::ConstPtr& target_planes,
    const gtsam_ext::Frame::ConstPtr& source_edges,
    const gtsam_ext::Frame::ConstPtr& source_planes
  ) {
    std::string method = GetParam();

    gtsam::NonlinearFactor::shared_ptr factor;
    if (method == "LOAM") {
      factor.reset(new gtsam_ext::IntegratedLOAMFactor(target_key, source_key, target_edges, target_planes, source_edges, source_planes));
    } else if (method == "EDGE") {
      factor.reset(new gtsam_ext::IntegratedPointToEdgeFactor(target_key, source_key, target_edges, source_edges));
    } else if (method == "PLANE") {
      factor.reset(new gtsam_ext::IntegratedPointToPlaneFactor(target_key, source_key, target_planes, source_planes));
    }

    return factor;
  }

  void test_graph(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values, const std::string& note = "") {
    gtsam_ext::LevenbergMarquardtExtParams lm_params;
    gtsam_ext::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);
    gtsam::Values result = optimizer.optimize();

    bool is_first = true;
    gtsam::Pose3 delta;

    for (const auto& value : result) {
      const gtsam::Pose3 pose_gt = poses_gt.at<gtsam::Pose3>(value.key);
      const gtsam::Pose3 pose = value.value.cast<gtsam::Pose3>();

      if (is_first) {
        is_first = false;
        delta = pose_gt * pose.inverse();
        continue;
      }

      const gtsam::Pose3 pose_error = pose_gt.inverse() * (delta * pose);
      const gtsam::Vector6 error = gtsam::Pose3::Logmap(pose_error);
      double error_r = error.head<3>().norm();
      double error_t = error.tail<3>().norm();

      EXPECT_LT(error_r, 0.015) << "Too large rotation error " << note;
      EXPECT_LT(error_t, 0.15) << "Too large translation error " << note;
    }
  }
};

INSTANTIATE_TEST_SUITE_P(gtsam_ext, FactorTest, testing::Values("EDGE", "PLANE", "LOAM"), [](const auto& info) { return info.param; });

TEST_P(FactorTest, test) {
  auto f = create_factor(0, 1, edge_frames[0], plane_frames[0], edge_frames[1], plane_frames[1]);
  if (f == nullptr) {
    std::cerr << "[          ] SKIP:" << GetParam() << std::endl;
    return;
  }

  for (int i = 0; i < 4; i++) {
    gtsam::Values values;
    gtsam::NonlinearFactorGraph graph;

    values.insert(i, poses.at(i));
    values.insert(i + 1, poses.at(i + 1));
    graph.add(create_factor(i, i + 1, edge_frames[i], plane_frames[i], edge_frames[i + 1], plane_frames[i + 1]));
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(i, poses.at<gtsam::Pose3>(i), gtsam::noiseModel::Isotropic::Precision(6, 1e6)));

    test_graph(graph, values, "FORWARD_TEST_" + std::to_string(i));

    graph.erase(graph.begin() + static_cast<int>(graph.size()) - 1);
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(i + 1, poses.at<gtsam::Pose3>(i + 1), gtsam::noiseModel::Isotropic::Precision(6, 1e6)));

    test_graph(graph, values, "BACKWARD_TEST_" + std::to_string(i));
  }

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph;
  for (int i = 0; i < 5; i++) {
    values.insert(i, poses.at(i));
    for (int j = i + 1; j < 5; j++) {
      graph.add(create_factor(i, j, edge_frames[i], plane_frames[i], edge_frames[j], plane_frames[j]));
    }
  }
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, poses.at<gtsam::Pose3>(0), gtsam::noiseModel::Isotropic::Precision(6, 1e6)));
  test_graph(graph, values, "MULTI_FRAME");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}