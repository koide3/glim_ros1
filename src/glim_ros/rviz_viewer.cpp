#include <glim_ros/rviz_viewer.hpp>

#include <mutex>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/util/trajectory_manager.hpp>
#include <glim/util/ros_cloud_converter.hpp>

namespace glim {

RvizViewer::RvizViewer() : nh(), private_nh("~") {
  points_pub = private_nh.advertise<sensor_msgs::PointCloud2>("/glim_ros/points", 1);
  map_pub = private_nh.advertise<sensor_msgs::PointCloud2>("/glim_ros/map", 1, true);

  odom_pub = private_nh.advertise<nav_msgs::Odometry>("/glim_ros/odom", 1);
  pose_pub = private_nh.advertise<geometry_msgs::PoseStamped>("/glim_ros/pose", 1);
  transform_pub = private_nh.advertise<geometry_msgs::TransformStamped>("/glim_ros/transform", 1);

  imu_frame_id = "imu";
  lidar_frame_id = "lidar";
  odom_frame_id = "odom";
  world_frame_id = "world";

  trajectory.reset(new TrajectoryManager);

  set_callbacks();

  kill_switch = false;
  thread = std::thread([this] {
    while (!kill_switch) {
      const auto expected = std::chrono::milliseconds(10);
      const auto t1 = std::chrono::high_resolution_clock::now();
      spin_once();
      const auto t2 = std::chrono::high_resolution_clock::now();

      if (t2 - t1 < expected) {
        std::this_thread::sleep_for(expected - (t2 - t1));
      }
    }
  });
}

RvizViewer::~RvizViewer() {
  kill_switch = true;
  thread.join();
}

void RvizViewer::set_callbacks() {
  using std::placeholders::_1;
  OdometryEstimationCallbacks::on_new_frame.add(std::bind(&RvizViewer::odometry_new_frame, this, _1));
  GlobalMappingCallbacks::on_update_submaps.add(std::bind(&RvizViewer::globalmap_on_update_submaps, this, _1));
}

void RvizViewer::odometry_new_frame(const EstimationFrame::ConstPtr& new_frame) {
  if (points_pub.getNumSubscribers()) {
    std::string frame_id;
    switch (new_frame->frame_id) {
      case FrameID::LIDAR:
        frame_id = lidar_frame_id;
        break;
      case FrameID::IMU:
        frame_id = imu_frame_id;
        break;
      case FrameID::WORLD:
        frame_id = world_frame_id;
        break;
    }

    auto points = frame_to_pointcloud2(frame_id, new_frame->stamp, *new_frame->frame);
    points_pub.publish(points);
  }

  const Eigen::Isometry3d T_odom_imu = new_frame->T_world_imu;
  const Eigen::Quaterniond quat_odom_imu(T_odom_imu.linear());

  const Eigen::Isometry3d T_lidar_imu = new_frame->T_lidar_imu;
  const Eigen::Quaterniond quat_lidar_imu(T_lidar_imu.linear());

  Eigen::Isometry3d T_world_odom;
  Eigen::Quaterniond quat_world_odom;

  Eigen::Isometry3d T_world_imu;
  Eigen::Quaterniond quat_world_imu;

  {
    std::lock_guard<std::mutex> lock(trajectory_mutex);
    trajectory->add_odom(new_frame->stamp, new_frame->T_world_imu);
    T_world_odom = trajectory->get_T_world_odom();
    quat_world_odom = Eigen::Quaterniond(T_world_odom.linear());

    T_world_imu = trajectory->odom2world(T_odom_imu);
    quat_world_imu = Eigen::Quaterniond(T_world_imu.linear());
  }

  // Odom -> IMU
  geometry_msgs::TransformStamped trans;
  trans.header.stamp = ros::Time(new_frame->stamp);
  trans.header.frame_id = odom_frame_id;
  trans.child_frame_id = imu_frame_id;
  trans.transform.translation.x = T_odom_imu.translation().x();
  trans.transform.translation.y = T_odom_imu.translation().y();
  trans.transform.translation.z = T_odom_imu.translation().z();
  trans.transform.rotation.x = quat_odom_imu.x();
  trans.transform.rotation.y = quat_odom_imu.y();
  trans.transform.rotation.z = quat_odom_imu.z();
  trans.transform.rotation.w = quat_odom_imu.w();
  tf_broadcaster.sendTransform(trans);

  // IMU -> LiDAR
  trans.header.frame_id = imu_frame_id;
  trans.child_frame_id = lidar_frame_id;
  trans.transform.translation.x = T_lidar_imu.translation().x();
  trans.transform.translation.y = T_lidar_imu.translation().y();
  trans.transform.translation.z = T_lidar_imu.translation().z();
  trans.transform.rotation.x = quat_lidar_imu.x();
  trans.transform.rotation.y = quat_lidar_imu.y();
  trans.transform.rotation.z = quat_lidar_imu.z();
  trans.transform.rotation.w = quat_lidar_imu.w();
  tf_broadcaster.sendTransform(trans);

  // World -> Odom
  trans.header.frame_id = world_frame_id;
  trans.child_frame_id = odom_frame_id;
  trans.transform.translation.x = T_world_odom.translation().x();
  trans.transform.translation.y = T_world_odom.translation().y();
  trans.transform.translation.z = T_world_odom.translation().z();
  trans.transform.rotation.x = quat_world_odom.x();
  trans.transform.rotation.y = quat_world_odom.y();
  trans.transform.rotation.z = quat_world_odom.z();
  trans.transform.rotation.w = quat_world_odom.w();
  tf_broadcaster.sendTransform(trans);

  if (odom_pub.getNumSubscribers()) {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time(new_frame->stamp);
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = imu_frame_id;
    odom.pose.pose.position.x = T_odom_imu.translation().x();
    odom.pose.pose.position.y = T_odom_imu.translation().y();
    odom.pose.pose.position.z = T_odom_imu.translation().z();
    odom.pose.pose.orientation.x = quat_odom_imu.x();
    odom.pose.pose.orientation.y = quat_odom_imu.y();
    odom.pose.pose.orientation.z = quat_odom_imu.z();
    odom.pose.pose.orientation.w = quat_odom_imu.w();
    odom_pub.publish(odom);
  }

  if (pose_pub.getNumSubscribers()) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time(new_frame->stamp);
    pose.header.frame_id = world_frame_id;
    pose.pose.position.x = T_world_imu.translation().x();
    pose.pose.position.y = T_world_imu.translation().y();
    pose.pose.position.z = T_world_imu.translation().z();
    pose.pose.orientation.x = quat_world_imu.x();
    pose.pose.orientation.y = quat_world_imu.y();
    pose.pose.orientation.z = quat_world_imu.z();
    pose.pose.orientation.w = quat_world_imu.w();
    pose_pub.publish(pose);
  }

  if (transform_pub.getNumSubscribers()) {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time(new_frame->stamp);
    transform.header.frame_id = imu_frame_id;
    transform.child_frame_id = odom_frame_id;
    transform.transform.translation.x = T_odom_imu.translation().x();
    transform.transform.translation.y = T_odom_imu.translation().y();
    transform.transform.translation.z = T_odom_imu.translation().z();
    transform.transform.rotation.x = quat_odom_imu.x();
    transform.transform.rotation.y = quat_odom_imu.y();
    transform.transform.rotation.z = quat_odom_imu.z();
    transform.transform.rotation.w = quat_odom_imu.w();
    transform_pub.publish(transform);
  }
}

void RvizViewer::globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& submaps) {
  const SubMap::ConstPtr latest_submap = submaps.back();

  const double stamp_endpoint_R = latest_submap->odom_frames.back()->stamp;
  const Eigen::Isometry3d T_world_endpoint_R = latest_submap->T_world_origin * latest_submap->T_origin_endpoint_R;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex);
    trajectory->update_anchor(stamp_endpoint_R, T_world_endpoint_R);
  }

  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> submap_poses(submaps.size());
  for (int i = 0; i < submaps.size(); i++) {
    submap_poses[i] = submaps[i]->T_world_origin;
  }

  invoke([this, latest_submap, submap_poses] {
    this->submaps.push_back(latest_submap->frame);

    if (!map_pub.getNumSubscribers()) {
      return;
    }

    int total_num_points = 0;
    for (const auto& submap : this->submaps) {
      total_num_points += submap->size();
    }

    gtsam_points::PointCloudCPU::Ptr merged(new gtsam_points::PointCloudCPU);
    merged->num_points = total_num_points;
    merged->points_storage.resize(total_num_points);
    merged->points = merged->points_storage.data();

    int begin = 0;
    for (int i = 0; i < this->submaps.size(); i++) {
      const auto& submap = this->submaps[i];
      std::transform(submap->points, submap->points + submap->size(), merged->points + begin, [&](const Eigen::Vector4d& p) { return submap_poses[i] * p; });
      begin += submap->size();
    }

    auto points_msg = frame_to_pointcloud2(world_frame_id, ros::Time::now().toSec(), *merged);
    map_pub.publish(points_msg);
  });
}

void RvizViewer::invoke(const std::function<void()>& task) {
  std::lock_guard<std::mutex> lock(invoke_queue_mutex);
  invoke_queue.push_back(task);
}

void RvizViewer::spin_once() {
  std::lock_guard<std::mutex> lock(invoke_queue_mutex);
  for (const auto& task : invoke_queue) {
    task();
  }
  invoke_queue.clear();
}

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::RvizViewer();
}