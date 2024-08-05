#include <atomic>
#include <thread>
#include <memory>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <glim/util/ros_cloud_converter.hpp>

namespace glim {

class TimeKeeper;
class CloudPreprocessor;
class OdometryEstimationBase;
class AsyncOdometryEstimation;
class AsyncSubMapping;
class AsyncGlobalMapping;

class ExtensionModule;
class GenericTopicSubscription;

/**
 * @brief glim instance for ROS environments
 */
class GlimROS {
public:
  GlimROS(ros::NodeHandle& nh);
  ~GlimROS();

  const std::vector<std::shared_ptr<ExtensionModule>>& extensions();
  const std::vector<std::shared_ptr<GenericTopicSubscription>>& extension_subscriptions();

  void insert_image(const double stamp, const cv::Mat& image);
  void insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);
  void insert_frame(const glim::RawPoints::Ptr& raw_points);

  void wait(bool auto_quit);

  void save(const std::string& path);

private:
  void loop();

private:
  std::atomic_bool kill_switch;
  std::thread thread;

  bool keep_raw_points;
  double imu_time_offset;
  double acc_scale;
  std::unique_ptr<glim::TimeKeeper> time_keeper;
  std::unique_ptr<glim::CloudPreprocessor> preprocessor;

  std::unique_ptr<glim::AsyncOdometryEstimation> odometry_estimation;
  std::unique_ptr<glim::AsyncSubMapping> sub_mapping;
  std::unique_ptr<glim::AsyncGlobalMapping> global_mapping;

  std::vector<std::shared_ptr<ExtensionModule>> extension_modules;
  std::vector<std::shared_ptr<GenericTopicSubscription>> extension_subs;
};

}  // namespace glim