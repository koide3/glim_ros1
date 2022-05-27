#pragma once

#include <atomic>
#include <thread>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <glim/frontend/estimation_frame.hpp>
#include <glim/backend/sub_map.hpp>
#include <glim/util/extension_module.hpp>

namespace glim {

class TrajectoryManager;

class RvizViewer : public ExtensionModule {
public:
  RvizViewer();
  ~RvizViewer();

private:
  void set_callbacks();
  void frontend_new_frame(const EstimationFrame::ConstPtr& new_frame);
  void globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& submaps);
  void invoke(const std::function<void()>& task);

  void spin_once();

private:
  std::atomic_bool kill_switch;
  std::thread thread;

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
  ros::Publisher transform_pub;

  std::mutex trajectory_mutex;
  std::unique_ptr<TrajectoryManager> trajectory;

  std::vector<gtsam_ext::Frame::ConstPtr> submaps;

  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;
};

}  // namespace glim