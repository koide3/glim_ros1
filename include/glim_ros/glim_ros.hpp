#include <atomic>
#include <thread>
#include <memory>
#include <opencv2/core.hpp>
#include <glim/util/ros_cloud_converter.hpp>

namespace glim {

class CloudPreprocessor;
class OdometryEstimationBase;
class AsyncOdometryEstimation;
class AsyncSubMapping;
class AsyncGlobalMapping;

class DBoWLoopDetector;
class ScanContextLoopDetector;
class OrbSLAMFrontend;

class RvizViewer;
class StandardViewer;

/**
 * @brief glim instance for ROS environments
 */
class GlimROS {
public:
  GlimROS();
  ~GlimROS();

  void insert_image(const double stamp, const cv::Mat& image);
  void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);
  void insert_frame(const glim::RawPoints::ConstPtr& raw_points);

  void insert_vi_image(const double stamp, const cv::Mat& image);
  void insert_vi_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);

  bool ok();
  void wait();

  void save(const std::string& path);

private:
  void loop();

private:
  std::atomic_bool kill_switch;
  std::thread thread;

  std::unique_ptr<glim::CloudPreprocessor> preprocessor;

  std::unique_ptr<glim::AsyncOdometryEstimation> odometry_estimation;
  std::unique_ptr<glim::AsyncSubMapping> sub_mapping;
  std::unique_ptr<glim::AsyncGlobalMapping> global_mapping;

#ifdef BUILD_WITH_GLIM_EXT
  std::unique_ptr<glim::DBoWLoopDetector> dbow_loop_detector;
  std::unique_ptr<glim::ScanContextLoopDetector> sc_loop_detector;
  std::unique_ptr<glim::OrbSLAMFrontend> orb_slam_frontend;
#endif

  std::unique_ptr<glim::RvizViewer> rviz_viewer;
#ifdef BUILD_WITH_VIEWER
  std::unique_ptr<glim::StandardViewer> standard_viewer;
#endif
};

}  // namespace glim