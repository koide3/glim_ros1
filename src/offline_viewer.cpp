#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <glim/util/config.hpp>
#include <glim/viewer/offline_viewer.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/serialization.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_points/factors/linear_damping_factor.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "offline_viewer");

  ros::NodeHandle nh("~");
  std::string config_path = nh.param<std::string>("config_path", "config");
  if (config_path[0] != '/') {
    // config_path is relative to the glim directory
    config_path = ros::package::getPath("glim") + "/" + config_path;
  }

  std::cout << "config_path: " << config_path << std::endl;
  glim::GlobalConfig::instance(config_path);

  glim::OfflineViewer viewer;
  viewer.wait();

  return 0;
}