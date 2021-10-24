#include <memory>
#include <iostream>
#include <boost/format.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/package.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nmea_msgs/Sentence.h>
#include <rosgraph_msgs/Clock.h>

#include <glim/util/config.hpp>
#include <glim/util/console_colors.hpp>
#include <glim/util/ros_cloud_converter.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/frontend/async_odometry_estimation.hpp>
#include <glim/frontend/odometry_estimation.hpp>
#include <glim/frontend/odometry_estimation_ct.hpp>
#include <glim/backend/async_sub_mapping.hpp>
#include <glim/backend/sub_mapping.hpp>
#include <glim/backend/async_global_mapping.hpp>
#include <glim/backend/global_mapping_ct.hpp>
#include <glim/backend/global_mapping.hpp>
#include <glim/viewer/standard_viewer.hpp>

#include <glim/util/easy_profiler.hpp>
#include <glim_ros/rviz_viewer.hpp>

class GlimROS {
public:
  GlimROS() {
    const std::string config_path = ros::package::getPath("glim") + "/config";
    glim::GlobalConfig::instance(config_path);

    rviz_viewer.reset(new glim::RvizViewer);
    standard_viewer.reset(new glim::StandardViewer);

    preprocessor.reset(new glim::CloudPreprocessor);
    // std::shared_ptr<glim::OdometryEstimationBase> odom(new glim::OdometryEstimationCT);
    std::shared_ptr<glim::OdometryEstimationBase> odom(new glim::OdometryEstimation);
    odometry_estimation.reset(new glim::AsyncOdometryEstimation(odom));

    std::shared_ptr<glim::SubMappingBase> sub(new glim::SubMapping);
    sub_mapping.reset(new glim::AsyncSubMapping(sub));

    std::shared_ptr<glim::GlobalMappingBase> global(new glim::GlobalMapping);
    global_mapping.reset(new glim::AsyncGlobalMapping(global));
  }

  void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
    odometry_estimation->insert_imu(stamp, linear_acc, angular_vel);
    sub_mapping->insert_imu(stamp, linear_acc, angular_vel);
    global_mapping->insert_imu(stamp, linear_acc, angular_vel);
  }

  void insert_frame(const glim::RawPoints::ConstPtr& raw_points) {
    while (odometry_estimation->input_queue_size() > 10) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    auto preprocessed = preprocessor->preprocess(raw_points->stamp, raw_points->times, raw_points->points);
    odometry_estimation->insert_frame(preprocessed);
  }

  bool spin_once() {
    std::vector<glim::EstimationFrame::ConstPtr> estimation_results;
    std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
    odometry_estimation->get_results(estimation_results, marginalized_frames);

    for (const auto& marginalized_frame : marginalized_frames) {
      sub_mapping->insert_frame(marginalized_frame);
    }

    const auto submaps = sub_mapping->get_results();
    for (const auto& submap : submaps) {
      global_mapping->insert_submap(submap);
    }

    rviz_viewer->spin_once();
    return standard_viewer->ok();
  }

  void wait() { standard_viewer->wait(); }

  void save(const std::string& path) { global_mapping->save(path); }

private:
  std::unique_ptr<glim::CloudPreprocessor> preprocessor;
  std::unique_ptr<glim::AsyncOdometryEstimation> odometry_estimation;
  std::unique_ptr<glim::AsyncSubMapping> sub_mapping;
  std::unique_ptr<glim::AsyncGlobalMapping> global_mapping;

  std::unique_ptr<glim::RvizViewer> rviz_viewer;
  std::unique_ptr<glim::StandardViewer> standard_viewer;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "glim_ros");
  ros::NodeHandle nh;
  ros::Publisher clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

  GlimROS glim_ros;

  const std::string bag_filename = "/home/koide/datasets/lio_sam/rooftop_ouster_dataset.bag";
  std::vector<std::string> topics = {"/points_raw", "/imu_raw"};

  rosbag::Bag bag(bag_filename, rosbag::bagmode::Read);
  if (!bag.isOpen()) {
    std::cerr << glim::console::bold_red << "error: failed to open " << bag_filename << glim::console::reset << std::endl;
    return 1;
  }

  for (rosbag::MessageInstance const m : rosbag::View(bag, rosbag::TopicQuery(topics))) {
    if (!glim_ros.spin_once()) {
      break;
    }

    const auto imu_msg = m.instantiate<sensor_msgs::Imu>();
    if (imu_msg) {
      const double stamp = imu_msg->header.stamp.toSec();
      const auto& linear_acc = imu_msg->linear_acceleration;
      const auto& angular_vel = imu_msg->angular_velocity;
      glim_ros.insert_imu(stamp, Eigen::Vector3d(linear_acc.x, linear_acc.y, linear_acc.z), Eigen::Vector3d(angular_vel.x, angular_vel.y, angular_vel.z));
    }

    const auto points_msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (points_msg) {
      auto raw_points = glim::RawPoints::extract(points_msg);
      glim_ros.insert_frame(raw_points);
    }

    // Ros-related
    rosgraph_msgs::Clock::Ptr clock_msg(new rosgraph_msgs::Clock);
    clock_msg->clock.sec = m.getTime().sec;
    clock_msg->clock.nsec = m.getTime().nsec;
    clock_pub.publish(clock_msg);
    ros::spinOnce();
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));
  glim_ros.save("/tmp/dump");
  glim_ros.wait();

  return 0;
}