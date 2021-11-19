#include <iostream>
#include <ros/package.h>
#include <glim/util/config.hpp>
#include <glim/viewer/offline_viewer.hpp>

int main(int argc, char** argv) {
  const std::string config_path = ros::package::getPath("glim") + "/config";
  glim::GlobalConfig::instance(config_path);

  glim::OfflineViewer viewer;
  viewer.wait();
}