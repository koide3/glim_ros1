#include <glim_ros/glim_ros.hpp>

#include <ros/package.h>

#include <glim/util/config.hpp>
#include <glim/util/console_colors.hpp>
#include <glim/util/time_keeper.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros.hpp>
#include <glim/util/ros_cloud_converter.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/frontend/async_odometry_estimation.hpp>
#include <glim/frontend/odometry_estimation_ct.hpp>
#include <glim/frontend/odometry_estimation_cpu.hpp>
#include <glim/frontend/odometry_estimation_gpu.hpp>
#include <glim/backend/async_sub_mapping.hpp>
#include <glim/backend/sub_mapping.hpp>
#include <glim/backend/async_global_mapping.hpp>
#include <glim/backend/global_mapping.hpp>
#include <glim/viewer/standard_viewer.hpp>

#include <glim_ros/rviz_viewer.hpp>

namespace glim {

GlimROS::GlimROS(ros::NodeHandle& nh) {
  std::string config_ros_path = ros::package::getPath("glim_ros") + "/config/glim_ros.json";
  config_ros_path = nh.param<std::string>("config_ros_path", config_ros_path);
  std::cout << "config_ros_path: " << config_ros_path << std::endl;
  glim::Config config_ros(config_ros_path);

  std::string config_path = ros::package::getPath("glim") + config_ros.param<std::string>("glim_ros", "config_path", "/config");
  config_path = nh.param<std::string>("config_path", config_path);
  std::cout << "config_path: " << config_path << std::endl;
  glim::GlobalConfig::instance(config_path);

  // Viewer
#ifdef BUILD_WITH_VIEWER
  if (config_ros.param<bool>("glim_ros", "enable_viewer", true)) {
    standard_viewer.reset(new glim::StandardViewer);
  }
#endif

  if (config_ros.param<bool>("glim_ros", "enable_rviz", true)) {
    extension_modules.push_back(std::shared_ptr<glim::RvizViewer>(new glim::RvizViewer));
  }

  // Extention modules
  const auto extensions = config_ros.param<std::vector<std::string>>("glim_ros", "extension_modules");
  if (extensions && !extensions->empty()) {
    std::cout << console::bold_red << "Extension modules are enabled!!" << console::reset << std::endl;
    std::cout << console::bold_red << "You must carefully check and follow the licenses of ext modules" << console::reset << std::endl;

    const std::string config_ext_path = ros::package::getPath("glim_ext") + "/config";
    std::cout << "config_ext_path: " << config_ext_path << std::endl;
    glim::GlobalConfig::instance()->override_param<std::string>("global", "config_ext", config_ext_path);

    for (const auto& extension : *extensions) {
      auto ext_module = ExtensionModule::load(extension);
      if (ext_module == nullptr) {
        std::cerr << console::bold_red << "error: failed to load " << extension << console::reset << std::endl;
        continue;
      } else {
        extension_modules.push_back(ext_module);

        auto ext_module_ros = std::dynamic_pointer_cast<ExtensionModuleROS>(ext_module);
        if (ext_module_ros) {
          const auto subs = ext_module_ros->create_subscriptions();
          extension_subs.insert(extension_subs.end(), subs.begin(), subs.end());
        }
      }
    }
  }

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
  if (frontend_mode == "CPU") {
    odom.reset(new glim::OdometryEstimationCPU);
  } else if (frontend_mode == "GPU") {
#ifdef BUILD_GTSAM_EXT_GPU
    odom.reset(new glim::OdometryEstimationGPU);
#else
    std::cerr << console::bold_red << "error: GPU frontend is selected although glim is built without GPU support!!" << console::reset << std::endl;
#endif
  } else if (frontend_mode == "CT") {
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

const std::vector<std::shared_ptr<GenericTopicSubscription>>& GlimROS::extension_subscriptions() {
  return extension_subs;
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
  // auto preprocessed = preprocessor->preprocess(raw_points->stamp, raw_points->times, raw_points->points);
  auto preprocessed = preprocessor->preprocess(raw_points);

  // note: Raw points are used only in extension modules for visualization purposes.
  //       If you need to reduce the memory footprint, you can safely comment out the following line.
  preprocessed->raw_points = raw_points;

  while (odometry_estimation->input_queue_size() > 10) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  odometry_estimation->insert_frame(preprocessed);
}

void GlimROS::loop() {
  while (!kill_switch) {
    std::vector<glim::EstimationFrame::ConstPtr> estimation_results;
    std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
    odometry_estimation->get_results(estimation_results, marginalized_frames);

    if (estimation_results.empty() && marginalized_frames.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    for (const auto& marginalized_frame : marginalized_frames) {
      sub_mapping->insert_frame(marginalized_frame);
    }

    const auto submaps = sub_mapping->get_results();
    for (const auto& submap : submaps) {
      global_mapping->insert_submap(submap);
    }
  }
}

void GlimROS::save(const std::string& path) {
  global_mapping->save(path);
}

bool GlimROS::ok() {
#ifdef BUILD_WITH_VIEWER
  if (!standard_viewer) {
    return true;
  }
  return standard_viewer->ok();
#else
  return true;
#endif
}

void GlimROS::wait() {
  std::cout << "odometry" << std::endl;
  odometry_estimation->join();

  std::vector<glim::EstimationFrame::ConstPtr> estimation_results;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
  odometry_estimation->get_results(estimation_results, marginalized_frames);
  for (const auto& marginalized_frame : marginalized_frames) {
    sub_mapping->insert_frame(marginalized_frame);
  }

  std::cout << "submap" << std::endl;
  sub_mapping->join();

  const auto submaps = sub_mapping->get_results();
  for (const auto& submap : submaps) {
    global_mapping->insert_submap(submap);
  }
  global_mapping->join();

#ifdef BUILD_WITH_VIEWER
  if (standard_viewer) {
    standard_viewer->wait();
  }
#endif
}

void GlimROS::stop() {
  std::cout << "odometry" << std::endl;
  odometry_estimation->join();

  std::vector<glim::EstimationFrame::ConstPtr> estimation_results;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
  odometry_estimation->get_results(estimation_results, marginalized_frames);
  for (const auto& marginalized_frame : marginalized_frames) {
    sub_mapping->insert_frame(marginalized_frame);
  }

  std::cout << "submap" << std::endl;
  sub_mapping->join();

  const auto submaps = sub_mapping->get_results();
  for (const auto& submap : submaps) {
    global_mapping->insert_submap(submap);
  }
  global_mapping->join();

#ifdef BUILD_WITH_VIEWER
  if (standard_viewer) {
    standard_viewer->stop();
  }
#endif
}

}  // namespace glim