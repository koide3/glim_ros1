#include <memory>
#include <iostream>
#include <boost/format.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/package.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nmea_msgs/Sentence.h>

#include <glim/util/config.hpp>
#include <glim/util/ros_cloud_converter.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/frontend/odometry_estimation_ct.hpp>
#include <glim/backend/async_sub_mapping.hpp>
#include <glim/backend/sub_mapping_ct.hpp>
#include <glim/backend/async_global_mapping.hpp>
#include <glim/backend/global_mapping_ct.hpp>
#include <glim/viewer/standard_viewer.hpp>

#include <glim/util/easy_profiler.hpp>

class GlimROS {
public:
  GlimROS() {
    const std::string config_path = ros::package::getPath("glim") + "/config";
    glim::GlobalConfig::instance()->override_param("global", "config_path", config_path);

    standard_viewer.reset(new glim::StandardViewer);

    preprocessor.reset(new glim::CloudPreprocessor);
    odometry_estimation.reset(new glim::OdometryEstimationCT);

    std::shared_ptr<glim::SubMappingBase> sub(new glim::SubMappingCT);
    sub_mapping.reset(new glim::AsyncSubMapping(sub));

    std::shared_ptr<glim::GlobalMappingBase> global(new glim::GlobalMappingCT);
    global_mapping.reset(new glim::AsyncGlobalMapping(global));
  }

  void insert_frame(const glim::RawPoints::ConstPtr& raw_points) {
    auto preprocessed = preprocessor->preprocess(raw_points->stamp, raw_points->times, raw_points->points);
    std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
    odometry_estimation->insert_frame(preprocessed, marginalized_frames);

    for(const auto& marginalized_frame : marginalized_frames) {
      sub_mapping->insert_frame(marginalized_frame);
    }

    const auto submaps = sub_mapping->get_results();
    for(const auto& submap: submaps) {
      global_mapping->insert_submap(submap);
    }
  }

private:
  std::unique_ptr<glim::CloudPreprocessor> preprocessor;
  std::unique_ptr<glim::OdometryEstimationBase> odometry_estimation;
  std::unique_ptr<glim::AsyncSubMapping> sub_mapping;
  std::unique_ptr<glim::AsyncGlobalMapping> global_mapping;

  std::unique_ptr<glim::StandardViewer> standard_viewer;
};

int main(int argc, char** argv) {
  GlimROS glim_ros;

  const std::string bag_filename = "/home/koide/datasets/map_iv/data1/data1_points.bag";
  std::vector<std::string> topics = {"/points_raw_ex"};

  rosbag::Bag bag(bag_filename, rosbag::bagmode::Read);
  if(!bag.isOpen()) {
    std::cerr << "error: failed to open " << bag_filename << std::endl;
    return 1;
  }

  for(rosbag::MessageInstance const m : rosbag::View(bag, rosbag::TopicQuery(topics))) {
    const auto points_msg = m.instantiate<sensor_msgs::PointCloud2>();
    if(points_msg == nullptr) {
      continue;
    }

    auto raw_points = glim::RawPoints::extract(points_msg);
    glim_ros.insert_frame(raw_points);
  }

  return 0;
}