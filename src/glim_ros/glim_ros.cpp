#include <glim_ros/glim_ros.hpp>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <ros/package.h>

#include <gtsam_points/optimizers/linearization_hook.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu_create.hpp>

#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/time_keeper.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros.hpp>
#include <glim/util/ros_cloud_converter.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/odometry/async_odometry_estimation.hpp>
#include <glim/odometry/odometry_estimation_ct.hpp>
#include <glim/odometry/odometry_estimation_cpu.hpp>
#include <glim/odometry/odometry_estimation_gpu.hpp>
#include <glim/mapping/async_sub_mapping.hpp>
#include <glim/mapping/sub_mapping.hpp>
#include <glim/mapping/async_global_mapping.hpp>
#include <glim/mapping/global_mapping.hpp>

namespace glim {

GlimROS::GlimROS(ros::NodeHandle& nh) {
  // Setup logger
  auto logger = spdlog::default_logger();
  auto ringbuffer_sink = get_ringbuffer_sink();
  logger->sinks().push_back(ringbuffer_sink);
  glim::set_default_logger(logger);

  if (nh.param<bool>("debug", false)) {
    spdlog::info("enable debug printing");
    logger->set_level(spdlog::level::trace);

    if (!logger->sinks().empty()) {
      auto console_sink = logger->sinks()[0];
      console_sink->set_level(spdlog::level::debug);
    }

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("/tmp/glim_log.log", true);
    file_sink->set_level(spdlog::level::trace);
    logger->sinks().push_back(file_sink);
  }

  spdlog::info("register linearization hooks");
#ifdef BUILD_GTSAM_POINTS_GPU
  gtsam_points::LinearizationHook::register_hook([]() { return gtsam_points::create_nonlinear_factor_set_gpu(); });
#endif

  std::string config_path = nh.param<std::string>("config_path", "config");
  if (config_path[0] != '/') {
    // config_path is relative to the glim directory
    config_path = ros::package::getPath("glim") + "/" + config_path;
  }

  spdlog::info("config_path: {}", config_path);
  glim::GlobalConfig::instance(config_path);
  glim::Config config_ros(glim::GlobalConfig::get_config_path("config_ros"));

  keep_raw_points = config_ros.param<bool>("glim_ros", "keep_raw_points", false);

  // Preprocessing
  imu_time_offset = config_ros.param<double>("glim_ros", "imu_time_offset", 0.0);
  acc_scale = config_ros.param<double>("glim_ros", "acc_scale", 1.0);

  time_keeper.reset(new glim::TimeKeeper);
  preprocessor.reset(new glim::CloudPreprocessor);

  // Odometry estimation
  glim::Config config_odometry(glim::GlobalConfig::get_config_path("config_odometry"));
  const std::string odometry_estimation_so_name = config_odometry.param<std::string>("odometry_estimation", "so_name", "libodometry_estimation_cpu.so");
  spdlog::info("load {}", odometry_estimation_so_name);

  std::shared_ptr<glim::OdometryEstimationBase> odom = OdometryEstimationBase::load_module(odometry_estimation_so_name);
  if (!odom) {
    spdlog::critical("failed to load odometry estimation module");
    abort();
  }
  odometry_estimation.reset(new glim::AsyncOdometryEstimation(odom, odom->requires_imu()));

  // Sub mapping
  const std::string sub_mapping_so_name = glim::Config(glim::GlobalConfig::get_config_path("config_sub_mapping")).param<std::string>("sub_mapping", "so_name", "libsub_mapping.so");
  if (!sub_mapping_so_name.empty()) {
    spdlog::info("load {}", sub_mapping_so_name);
    auto sub = SubMappingBase::load_module(sub_mapping_so_name);
    if (sub) {
      sub_mapping.reset(new AsyncSubMapping(sub));
    }
  }

  // Global mapping
  const std::string global_mapping_so_name =
    glim::Config(glim::GlobalConfig::get_config_path("config_global_mapping")).param<std::string>("global_mapping", "so_name", "libglobal_mapping.so");
  if (!global_mapping_so_name.empty()) {
    spdlog::info("load {}", global_mapping_so_name);
    auto global = GlobalMappingBase::load_module(global_mapping_so_name);
    if (global) {
      global_mapping.reset(new AsyncGlobalMapping(global));
    }
  }

  // Extention modules
  const auto extensions = config_ros.param<std::vector<std::string>>("glim_ros", "extension_modules");
  if (extensions && !extensions->empty()) {
    for (const auto& extension : *extensions) {
      if (extension.find("viewer") == std::string::npos) {
        spdlog::warn("Extension modules are enabled!!");
        spdlog::warn("You must carefully check and follow the licenses of ext modules");

        const std::string config_ext_path = ros::package::getPath("glim_ext") + "/config";
          spdlog::info("config_ext_path: {}", config_ext_path);
        glim::GlobalConfig::instance()->override_param<std::string>("global", "config_ext", config_ext_path);

        break;
      }
    }

    for (const auto& extension : *extensions) {
      spdlog::info("load {}", extension);
      auto ext_module = ExtensionModule::load_module(extension);
      if (ext_module == nullptr) {
        spdlog::error("failed to load {}", extension);
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

  // Start process loop
  kill_switch = false;
  thread = std::thread([this] { loop(); });

  spdlog::debug("initialized");
}

GlimROS::~GlimROS() {
  kill_switch = true;
  thread.join();
}

const std::vector<std::shared_ptr<ExtensionModule>>& GlimROS::extensions() {
  return extension_modules;
}

const std::vector<std::shared_ptr<GenericTopicSubscription>>& GlimROS::extension_subscriptions() {
  return extension_subs;
}

void GlimROS::insert_image(const double stamp, const cv::Mat& image) {
  spdlog::trace("image: {:.6f}", stamp);

  odometry_estimation->insert_image(stamp, image);

  if (sub_mapping) {
    sub_mapping->insert_image(stamp, image);
  }
  if (global_mapping) {
    global_mapping->insert_image(stamp, image);
  }
}

void GlimROS::insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  spdlog::trace("IMU: {:.6f}", stamp);

  stamp += imu_time_offset;
  time_keeper->validate_imu_stamp(stamp);

  odometry_estimation->insert_imu(stamp, acc_scale * linear_acc, angular_vel);

  if (sub_mapping) {
    sub_mapping->insert_imu(stamp, acc_scale * linear_acc, angular_vel);
  }
  if (global_mapping) {
    global_mapping->insert_imu(stamp, acc_scale * linear_acc, angular_vel);
  }
}

void GlimROS::insert_frame(const glim::RawPoints::Ptr& raw_points) {
  spdlog::trace("points: {:.6f}", raw_points->stamp);

  time_keeper->process(raw_points);
  // auto preprocessed = preprocessor->preprocess(raw_points->stamp, raw_points->times, raw_points->points);
  auto preprocessed = preprocessor->preprocess(raw_points);

  if (!preprocessed || preprocessed->size() < 100) {
    spdlog::warn("skipping frame with too few points");
    return;
  }

  if (keep_raw_points) {
    // note: Raw points are used only in extension modules for visualization purposes.
    //       If you need to reduce the memory footprint, you can safely comment out the following line.
    preprocessed->raw_points = raw_points;
  }

  while (odometry_estimation->workload() > 10) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  odometry_estimation->insert_frame(preprocessed);
}

void GlimROS::loop() {
  while (!kill_switch) {
    for (const auto& ext_module : extension_modules) {
      if (!ext_module->ok()) {
        ros::shutdown();
      }
    }

    std::vector<glim::EstimationFrame::ConstPtr> estimation_results;
    std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
    odometry_estimation->get_results(estimation_results, marginalized_frames);

    if (estimation_results.empty() && marginalized_frames.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (sub_mapping) {
      for (const auto& marginalized_frame : marginalized_frames) {
        sub_mapping->insert_frame(marginalized_frame);
      }
      const auto submaps = sub_mapping->get_results();

      if (global_mapping) {
        for (const auto& submap : submaps) {
          global_mapping->insert_submap(submap);
        }
      }
    }
  }
}

void GlimROS::save(const std::string& path) {
  if (global_mapping) {
    global_mapping->save(path);
  }
}

void GlimROS::wait(bool auto_quit) {
  spdlog::info("waiting for odometry estimation");
  odometry_estimation->join();

  if (sub_mapping) {
    std::vector<glim::EstimationFrame::ConstPtr> estimation_results;
    std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
    odometry_estimation->get_results(estimation_results, marginalized_frames);
    for (const auto& marginalized_frame : marginalized_frames) {
      sub_mapping->insert_frame(marginalized_frame);
    }

    spdlog::info("waiting for local mapping");
    sub_mapping->join();
    const auto submaps = sub_mapping->get_results();

    if (global_mapping) {
      for (const auto& submap : submaps) {
        global_mapping->insert_submap(submap);
      }

      spdlog::info("waiting for global mapping");
      global_mapping->join();
    }
  }

  if (!auto_quit) {
    bool terminate = false;
    while (ros::ok() && !terminate) {
      for (const auto& ext_module : extension_modules) {
        terminate |= (!ext_module->ok());
      }
    }
  }
}

}  // namespace glim