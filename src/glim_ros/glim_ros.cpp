#include <glim_ros/glim_ros.hpp>

#include <ros/package.h>

#include <glim/util/config.hpp>
#include <glim/util/console_colors.hpp>
#include <glim/util/time_keeper.hpp>
#include <glim/util/ros_cloud_converter.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/frontend/async_odometry_estimation.hpp>
#include <glim/frontend/odometry_estimation_ct.hpp>
#include <glim/frontend/odometry_estimation_gpu.hpp>
#include <glim/backend/async_sub_mapping.hpp>
#include <glim/backend/sub_mapping.hpp>
#include <glim/backend/async_global_mapping.hpp>
#include <glim/backend/global_mapping.hpp>
#include <glim/viewer/standard_viewer.hpp>

#ifdef BUILD_WITH_GLIM_EXT
#include <glim_ext/util/config_ext.hpp>
#include <glim_ext/dbow_loop_detector.hpp>
#include <glim_ext/scan_context_loop_detector.hpp>
#include <glim_ext/orb_slam_frontend.hpp>
#endif

#include <glim_ros/rviz_viewer.hpp>

namespace glim {

GlimROS::GlimROS() {
  const std::string config_ros_path = ros::package::getPath("glim_ros") + "/config/glim_ros.json";
  glim::Config config_ros(config_ros_path);

  const std::string config_path = ros::package::getPath("glim") + config_ros.param<std::string>("glim_ros", "config_path", "/config");
  std::cout << "config_path: " << config_path << std::endl;
  glim::GlobalConfig::instance(config_path);

  // Viewer
#ifdef BUILD_WITH_VIEWER
  if(config_ros.param<bool>("glim_ros", "enable_viewer", true)) {
    standard_viewer.reset(new glim::StandardViewer);
  }
#endif

  if(config_ros.param<bool>("glim_ros", "enable_rviz", true)) {
    rviz_viewer.reset(new glim::RvizViewer);
  }

  // Extention modules
#ifdef BUILD_WITH_GLIM_EXT
  std::cout << console::bold_red << "Extension modules are enabled!!" << console::reset << std::endl;
  std::cout << console::bold_red << "You must carefully check and follow the licenses of ext modules" << console::reset << std::endl;

  const std::string config_ext_path = ros::package::getPath("glim_ext") + "/config";
  glim::GlobalConfigExt::instance(config_ext_path);

  if(config_ros.param<bool>("glim_ros", "enable_ext_dbow", false)) {
    dbow_loop_detector.reset(new glim::DBoWLoopDetector);
  }

  if(config_ros.param<bool>("glim_ros", "enable_ext_scancontext", false)) {
    sc_loop_detector.reset(new glim::ScanContextLoopDetector);
  }

  if(config_ros.param<bool>("glim_ros", "enable_ext_orbslam", false)) {
    orb_slam_frontend.reset(new glim::OrbSLAMFrontend(true, false));
  }
#endif

  // Preprocessing
  imu_time_offset = config_ros.param<double>("glim_ros", "imu_time_offset", 0.0);
  acc_scale = config_ros.param<double>("glim_ros", "acc_scale", 1.0);

  time_keeper.reset(new glim::TimeKeeper);
  preprocessor.reset(new glim::CloudPreprocessor);

  // Odometry estimation
  glim::Config config_frontend(glim::GlobalConfig::get_config_path("config_frontend"));
  const std::string frontend_mode = config_frontend.param<std::string>("odometry_estimation", "frontend_mode", "CPU");

  bool enable_imu = true;
  std::shared_ptr<glim::OdometryEstimationBase> odom;
  if(frontend_mode == "CPU") {
  } else if(frontend_mode == "GPU") {
#ifdef BUILD_GTSAM_EXT_GPU
    odom.reset(new glim::OdometryEstimationGPU);
#else
    std::cerr << console::bold_red << "error: GPU frontend is selected although glim is built without GPU support!!" << console::reset << std::endl;
#endif
  } else if(frontend_mode == "CT") {
    enable_imu = false;
    odom.reset(new glim::OdometryEstimationCT);
  }

  odometry_estimation.reset(new glim::AsyncOdometryEstimation(odom, enable_imu));

  // Sub mapping
  std::shared_ptr<glim::SubMappingBase> sub(new glim::SubMapping);
  sub_mapping.reset(new glim::AsyncSubMapping(sub));

  // Global mapping
  std::shared_ptr<glim::GlobalMappingBase> global(new glim::GlobalMapping);
  global_mapping.reset(new glim::AsyncGlobalMapping(global));

  // Start process loop
  kill_switch = false;
  thread = std::thread([this] { loop(); });
}

GlimROS::~GlimROS() {
  kill_switch = true;
  thread.join();
}

void GlimROS::insert_image(const double stamp, const cv::Mat& image) {
  odometry_estimation->insert_image(stamp, image);
  sub_mapping->insert_image(stamp, image);
  global_mapping->insert_image(stamp, image);
}

void GlimROS::insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  stamp += imu_time_offset;
  time_keeper->validate_imu_stamp(stamp);

  odometry_estimation->insert_imu(stamp, acc_scale * linear_acc, angular_vel);
  sub_mapping->insert_imu(stamp, acc_scale * linear_acc, angular_vel);
  global_mapping->insert_imu(stamp, acc_scale * linear_acc, angular_vel);
}

void GlimROS::insert_frame(const glim::RawPoints::Ptr& raw_points) {
  time_keeper->process(raw_points);
  auto preprocessed = preprocessor->preprocess(raw_points->stamp, raw_points->times, raw_points->points);

  while (odometry_estimation->input_queue_size() > 10) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  odometry_estimation->insert_frame(preprocessed);
}

void GlimROS::insert_vi_image(const double stamp, const cv::Mat& image) {
#ifdef BUILD_WITH_GLIM_EXT
  if(orb_slam_frontend) {
    orb_slam_frontend->insert_image(stamp, image);
  }
#endif
}

void GlimROS::insert_vi_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
#ifdef BUILD_WITH_GLIM_EXT
  if(orb_slam_frontend) {
    orb_slam_frontend->insert_imu(stamp, linear_acc, angular_vel);
  }
#endif
}

void GlimROS::loop() {
  while(!kill_switch) {
    std::vector<glim::EstimationFrame::ConstPtr> estimation_results;
    std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
    odometry_estimation->get_results(estimation_results, marginalized_frames);

    if(estimation_results.empty() && marginalized_frames.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    for(const auto& marginalized_frame : marginalized_frames) {
      sub_mapping->insert_frame(marginalized_frame);
    }

    const auto submaps = sub_mapping->get_results();
    for(const auto& submap : submaps) {
      global_mapping->insert_submap(submap);
    }
  }
}

void GlimROS::save(const std::string& path) {
  global_mapping->save(path);
}

#ifdef BUILD_WITH_VIEWER
bool GlimROS::ok() {
  if(!standard_viewer) {
    return true;
  }
  return standard_viewer->ok();
}

void GlimROS::wait() {
  if(standard_viewer) {
    standard_viewer->wait();
  }
}

#else
bool GlimROS::ok() {
  return true;
}
void GlimROS::wait() {}
#endif

}  // namespace glim