#include <iostream>
#include <fstream>
#include <sstream>

#include <gtsam/inference/Symbol.h>
#include <gtsam_points/util/continuous_trajectory.hpp>

#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

std::vector<double> stamps{};
std::vector<gtsam::Pose3> poses{};

void readTrajectoryFromFile(const std::string& filename) {
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        double stamp;
        Eigen::Matrix4d pose_matrix;
        iss >> stamp;
        for (int i = 0; i < 12; ++i) {
            iss >> pose_matrix(i / 4, i % 4);
        }
        pose_matrix(3, 0) = 0.0;
        pose_matrix(3, 1) = 0.0;
        pose_matrix(3, 2) = 0.0;
        pose_matrix(3, 3) = 1.0;

        gtsam::Pose3 pose(pose_matrix);
        // std::cout << "stamp: " << stamp << std::endl;
        stamps.push_back(stamp);
        poses.push_back(pose);
    }

    infile.close();
}

void writePosesToFile(const std::string& filename, const std::vector<gtsam::Pose3>& poses) {
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    for (size_t i = 0; i < poses.size(); ++i) {
        Eigen::Matrix4d pose_matrix = poses[i].matrix();
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 4; ++col) {
                outfile << pose_matrix(row, col);
                if (row != 2 || col != 3) {
                    outfile << " ";
                }
            }
        }
        outfile << std::endl;
    }

    outfile.close();
}


int main(int argc, char** argv) {
  readTrajectoryFromFile(argv[1]);

  auto viewer = guik::LightViewer::instance();
  viewer->set_draw_xy_grid(false);

  for (auto const& pose : poses) {
    // std::cout << pose << std::endl;
    Eigen::Affine3f model_matrix(pose.matrix().cast<float>());
    viewer->update_drawable("target_" + std::to_string(pose.x()), glk::Primitives::coordinate_system(), guik::VertexColor(model_matrix.scale(0.5f)));
  }

  // Create continuous trajectory and optimize spline knots to fit the trajectory with the target poses
  gtsam_points::ContinuousTrajectory ct('x', stamps.front(), stamps.back(), 0.5);
  gtsam::Values values = ct.fit_knots(stamps, poses);

  std::vector<gtsam::Pose3> optimized_poses{};
  for (const auto& stamp : stamps) {
      gtsam::Pose3 pose = ct.pose(values, stamp);
      optimized_poses.push_back(pose);
  }
  for (int i = 0; i < ct.knot_max_id(); i++) {
    Eigen::Affine3f model_matrix(values.at<gtsam::Pose3>(gtsam::Symbol('x', i)).matrix().cast<float>());
    viewer->update_drawable("knot_" + std::to_string(i), glk::Primitives::coordinate_system(), guik::FlatColor(0.5f, 0.5f, 0.5f, 1.0f, model_matrix.scale(0.5f)));
  }
  writePosesToFile("/dev_ws/data/continuous/output.txt", optimized_poses);


  // Drawable filter
  auto T_world_lidar = poses.front();
  viewer->lookat(T_world_lidar.translation().cast<float>());
  bool show_knots = true;
  bool show_targets = false;
  viewer->register_drawable_filter("filter", [&](const std::string& name) {
    if (!show_knots && name.find("knot_") != std::string::npos) {
      return false;
    }

    if (!show_targets && name.find("target_") != std::string::npos) {
      return false;
    }

    return true;
  });

  // Visualization
  float time = stamps.front();
  viewer->register_ui_callback("ui", [&] {
    ImGui::Checkbox("knots", &show_knots);
    ImGui::SameLine();
    ImGui::Checkbox("targets", &show_targets);
    ImGui::Separator();

    ImGui::DragFloat("time", &time, 0.01f, stamps.front(), stamps.back());

    // Calculate interpolated pose and linear acc and angular vel
    const gtsam::Pose3 pose = ct.pose(values, time);
    // const gtsam::Vector6 imu = ct.imu(values, time);

    // There are also expression interfaces to easily create constraints
    const gtsam::Pose3_ pose_ = ct.pose(time, gtsam::Double_(time));
    // const gtsam::Vector6_ imu_ = ct.imu(time, gtsam::Double_(time));

    Eigen::Affine3f model_matrix(pose.matrix().cast<float>());
    viewer->update_drawable("pose", glk::Primitives::coordinate_system(), guik::VertexColor(model_matrix.scale(0.5f)));

    // ImGui::Text("Linear acc : %.3f %.3f %.3f", imu[0], imu[1], imu[2]);
    // ImGui::Text("Angular vel: %.3f %.3f %.3f", imu[3], imu[4], imu[5]);
  });

  viewer->spin();
}