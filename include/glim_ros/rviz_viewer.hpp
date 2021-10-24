#pragma once

#include <mutex>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include <gtsam_ext/types/frame_cpu.hpp>
#include <glim/frontend/callbacks.hpp>
#include <glim/backend/callbacks.hpp>
#include <glim/util/trajectory_manager.hpp>
#include <glim/util/ros_cloud_converter.hpp>

namespace glim {

class RvizViewer {
public:
  RvizViewer() : nh(), private_nh("~") {
    points_pub = private_nh.advertise<sensor_msgs::PointCloud2>("points", 1);
    map_pub = private_nh.advertise<sensor_msgs::PointCloud2>("map", 1, true);

    odom_pub = private_nh.advertise<nav_msgs::Odometry>("odom", 1);
    pose_pub = private_nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

    imu_frame_id = "imu";
    lidar_frame_id = "lidar";
    odom_frame_id = "odom";
    world_frame_id = "world";

    trajectory.reset(new TrajectoryManager);

    set_callbacks();
  }

  ~RvizViewer() {}

  void set_callbacks() {
    using std::placeholders::_1;
    OdometryEstimationCallbacks::on_new_frame.add(std::bind(&RvizViewer::frontend_new_frame, this, _1));
    GlobalMappingCallbacks::on_update_submaps.add(std::bind(&RvizViewer::globalmap_on_update_submaps, this, _1));
  }

  void frontend_new_frame(const EstimationFrame::ConstPtr& new_frame) {
    if (points_pub.getNumSubscribers()) {
      auto points = frame_to_pointcloud2(lidar_frame_id, new_frame->stamp, *new_frame->frame);
      points_pub.publish(points);
    }

    const std::string frame_id = new_frame->frame_id == FrameID::LIDAR ? lidar_frame_id : imu_frame_id;
    const Eigen::Isometry3d T_odom_sensor = new_frame->T_world_sensor();
    const Eigen::Quaterniond quat_odom_sensor(T_odom_sensor.linear());

    Eigen::Isometry3d T_world_odom;
    Eigen::Quaterniond quat_world_odom;

    Eigen::Isometry3d T_world_sensor;
    Eigen::Quaterniond quat_world_sensor;

    {
      std::lock_guard<std::mutex> lock(trajectory_mutex);
      trajectory->add_odom(new_frame->stamp, new_frame->T_world_sensor());
      T_world_odom = trajectory->get_T_world_odom();
      quat_world_odom = Eigen::Quaterniond(T_world_odom.linear());

      T_world_sensor = trajectory->odom2world(T_odom_sensor);
      quat_world_sensor = Eigen::Quaterniond(T_world_sensor.linear());
    }

    geometry_msgs::TransformStamped trans;
    trans.header.stamp = ros::Time(new_frame->stamp);
    trans.header.frame_id = odom_frame_id;
    trans.child_frame_id = frame_id;
    trans.transform.translation.x = T_odom_sensor.translation().x();
    trans.transform.translation.y = T_odom_sensor.translation().y();
    trans.transform.translation.z = T_odom_sensor.translation().z();
    trans.transform.rotation.x = quat_odom_sensor.x();
    trans.transform.rotation.y = quat_odom_sensor.y();
    trans.transform.rotation.z = quat_odom_sensor.z();
    trans.transform.rotation.w = quat_odom_sensor.w();
    tf_broadcaster.sendTransform(trans);

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
      odom.child_frame_id = frame_id;
      odom.pose.pose.position.x = T_odom_sensor.translation().x();
      odom.pose.pose.position.y = T_odom_sensor.translation().y();
      odom.pose.pose.position.z = T_odom_sensor.translation().z();
      odom.pose.pose.orientation.x = quat_odom_sensor.x();
      odom.pose.pose.orientation.y = quat_odom_sensor.y();
      odom.pose.pose.orientation.z = quat_odom_sensor.z();
      odom.pose.pose.orientation.w = quat_odom_sensor.w();
      odom_pub.publish(odom);
    }

    if (pose_pub.getNumSubscribers()) {
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time(new_frame->stamp);
      pose.header.frame_id = world_frame_id;
      pose.pose.position.x = T_world_sensor.translation().x();
      pose.pose.position.y = T_world_sensor.translation().y();
      pose.pose.position.z = T_world_sensor.translation().z();
      pose.pose.orientation.x = quat_world_sensor.x();
      pose.pose.orientation.y = quat_world_sensor.y();
      pose.pose.orientation.z = quat_world_sensor.z();
      pose.pose.orientation.w = quat_world_sensor.w();
      pose_pub.publish(pose);
    }
  }

  void globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& submaps) {
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

      gtsam_ext::FrameCPU::Ptr merged(new gtsam_ext::FrameCPU);
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

  void invoke(const std::function<void()>& task) {
    std::lock_guard<std::mutex> lock(invoke_queue_mutex);
    invoke_queue.push_back(task);
  }

  void spin_once() {
    std::lock_guard<std::mutex> lock(invoke_queue_mutex);
    for (const auto& task : invoke_queue) {
      task();
    }
    invoke_queue.clear();
  }

private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  tf2_ros::TransformBroadcaster tf_broadcaster;

  std::string imu_frame_id;
  std::string lidar_frame_id;
  std::string odom_frame_id;
  std::string world_frame_id;

  ros::Publisher points_pub;
  ros::Publisher map_pub;

  ros::Publisher odom_pub;
  ros::Publisher pose_pub;

  std::mutex trajectory_mutex;
  std::unique_ptr<TrajectoryManager> trajectory;

  std::vector<gtsam_ext::Frame::ConstPtr> submaps;

  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;
};

}  // namespace glim