#include <iostream>
#include <ros/package.h>
#include <glim/util/config.hpp>
#include <glim/viewer/offline_viewer.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/serialization.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_ext/factors/linear_damping_factor.hpp>

int main(int argc, char** argv) {
  const std::string config_path = ros::package::getPath("glim") + "/config";
  glim::GlobalConfig::instance(config_path);

  glim::OfflineViewer viewer;
  viewer.wait();

  return 0;
}