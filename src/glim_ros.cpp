#include <glob.h>
#include <memory>
#include <iostream>
#include <boost/format.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/package.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
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
#include <glim/backend/global_mapping.hpp>
#include <glim/viewer/standard_viewer.hpp>

#ifdef BUILD_WITH_GLIM_EXT
#include <glim_ext/util/config_ext.hpp>
#include <glim_ext/dbow_loop_detector.hpp>
#include <glim_ext/scan_context_loop_detector.hpp>
#include <glim_ext/orb_slam_frontend.hpp>
#endif

#include <glim/util/easy_profiler.hpp>
#include <glim_ros/rviz_viewer.hpp>

class GlimROS {
public:
  GlimROS() {
    const std::string config_path = ros::package::getPath("glim") + "/config";
    glim::GlobalConfig::instance(config_path);

    const std::string config_ros_path = ros::package::getPath("glim_ros") + "/config/glim_rosbag.json";
    config.reset(new glim::Config(config_ros_path));

    rviz_viewer.reset(new glim::RvizViewer);
    standard_viewer.reset(new glim::StandardViewer);

#ifdef BUILD_WITH_GLIM_EXT
    const std::string config_ext_path = ros::package::getPath("glim_ext") + "/config";
    glim::GlobalConfigExt::instance(config_ext_path);

    // dbow_loop_detector.reset(new glim::DBoWLoopDetector);
    // sc_loop_detector.reset(new glim::ScanContextLoopDetector);
    orb_slam_frontend.reset(new glim::OrbSLAMFrontend(true, false));
#endif

    preprocessor.reset(new glim::CloudPreprocessor);
    std::shared_ptr<glim::OdometryEstimationBase> odom(new glim::OdometryEstimation);
    odometry_estimation.reset(new glim::AsyncOdometryEstimation(odom));

    std::shared_ptr<glim::SubMappingBase> sub(new glim::SubMapping);
    sub_mapping.reset(new glim::AsyncSubMapping(sub));

    std::shared_ptr<glim::GlobalMappingBase> global(new glim::GlobalMapping);
    global_mapping.reset(new glim::AsyncGlobalMapping(global));
  }

  void insert_image(const double stamp, const cv::Mat& image) {
    odometry_estimation->insert_image(stamp, image);
    sub_mapping->insert_image(stamp, image);
    global_mapping->insert_image(stamp, image);
  }

  void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
    odometry_estimation->insert_imu(stamp, linear_acc, angular_vel);
    sub_mapping->insert_imu(stamp, linear_acc, angular_vel);
    global_mapping->insert_imu(stamp, linear_acc, angular_vel);
  }

  void insert_vi_image(const double stamp, const cv::Mat& image) {
#ifdef BUILD_WITH_GLIM_EXT
    if (orb_slam_frontend) {
      orb_slam_frontend->insert_image(stamp, image);
    }
#endif
  }

  void insert_vi_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
#ifdef BUILD_WITH_GLIM_EXT
    if (orb_slam_frontend) {
      orb_slam_frontend->insert_imu(stamp, linear_acc, angular_vel);
    }
#endif
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

  const glim::Config& get_config() const { return *config; }

private:
  std::unique_ptr<glim::Config> config;

  std::unique_ptr<glim::CloudPreprocessor> preprocessor;
  std::unique_ptr<glim::AsyncOdometryEstimation> odometry_estimation;
  std::unique_ptr<glim::AsyncSubMapping> sub_mapping;
  std::unique_ptr<glim::AsyncGlobalMapping> global_mapping;

#ifdef BUILD_WITH_GLIM_EXT
  std::unique_ptr<glim::DBoWLoopDetector> dbow_loop_detector;
  std::unique_ptr<glim::ScanContextLoopDetector> sc_loop_detector;
  // std::unique_ptr<glim::DSOFrontend> dso_frontend;
  std::unique_ptr<glim::OrbSLAMFrontend> orb_slam_frontend;
#endif

  std::unique_ptr<glim::RvizViewer> rviz_viewer;
  std::unique_ptr<glim::StandardViewer> standard_viewer;
};

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: glim_rosbag input_rosbag_path" << std::endl;
    return 0;
  }

  ros::init(argc, argv, "glim_rosbag");
  ros::NodeHandle nh;
  ros::Publisher clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

  // Initialize GLIM
  GlimROS glim_ros;

  // List input topics
  const auto topics = glim_ros.get_config().param<std::vector<std::string>>("glim_rosbag", "topics");
  if (!topics) {
    std::cerr << "error: topics must be specified" << std::endl;
    return 1;
  }

  std::cout << "topics:" << std::endl;
  for (const auto& topic : *topics) {
    std::cout << "- " << topic << std::endl;
  }

  // List input rosbag filenames
  const std::string bag_path = argv[1];
  std::vector<std::string> bag_filenames;

  for (int i = 1; i < argc; i++) {
    std::vector<std::string> filenames;
    glob_t globbuf;
    int ret = glob(bag_path.c_str(), 0, nullptr, &globbuf);
    for (int i = 0; i < globbuf.gl_pathc; i++) {
      filenames.push_back(globbuf.gl_pathv[i]);
    }
    globfree(&globbuf);

    std::sort(filenames.begin(), filenames.end());
    bag_filenames.insert(bag_filenames.end(), filenames.begin(), filenames.end());
  }

  std::cout << "bag_filenames:" << std::endl;
  for (const auto& bag_filename : bag_filenames) {
    std::cout << "- " << bag_filename << std::endl;
  }

  // Bag read function
  const auto read_bag = [&](const std::string& bag_filename, const std::vector<std::string>& topics) {
    std::cout << "opening " << bag_filename << std::endl;

    rosbag::Bag bag(bag_filename, rosbag::bagmode::Read);
    if (!bag.isOpen()) {
      std::cerr << glim::console::bold_red << "error: failed to open " << bag_filename << glim::console::reset << std::endl;
      return false;
    }

    for (rosbag::MessageInstance const m : rosbag::View(bag, rosbag::TopicQuery(topics))) {
      if (!glim_ros.spin_once()) {
        return false;
      }

      const auto imu_msg = m.instantiate<sensor_msgs::Imu>();
      if (imu_msg) {
        const double stamp = imu_msg->header.stamp.toSec();
        const auto& linear_acc = imu_msg->linear_acceleration;
        const auto& angular_vel = imu_msg->angular_velocity;

        if (m.getTopic() == glim_ros.get_config().param<std::string>("glim_rosbag", "vi_imu_topic", "")) {
          glim_ros.insert_vi_imu(stamp, Eigen::Vector3d(linear_acc.x, linear_acc.y, linear_acc.z), Eigen::Vector3d(angular_vel.x, angular_vel.y, angular_vel.z));
        } else {
          glim_ros.insert_imu(stamp, Eigen::Vector3d(linear_acc.x, linear_acc.y, linear_acc.z), Eigen::Vector3d(angular_vel.x, angular_vel.y, angular_vel.z));
        }
      }

      const auto points_msg = m.instantiate<sensor_msgs::PointCloud2>();
      if (points_msg) {
        auto raw_points = glim::RawPoints::extract(points_msg);
        glim_ros.insert_frame(raw_points);
      }

      const auto compressed_img_msg = m.instantiate<sensor_msgs::CompressedImage>();
      if (compressed_img_msg) {
        auto cv_image = cv_bridge::toCvCopy(compressed_img_msg, "bgr8");
        if (m.getTopic() == glim_ros.get_config().param<std::string>("glim_rosbag", "vi_image_topic", "")) {
          glim_ros.insert_vi_image(compressed_img_msg->header.stamp.toSec(), cv_image->image);
        } else {
          glim_ros.insert_image(compressed_img_msg->header.stamp.toSec(), cv_image->image);
        }
      }

      const auto img_msg = m.instantiate<sensor_msgs::Image>();
      if (img_msg) {
        auto cv_image = cv_bridge::toCvCopy(img_msg, "bgr8");
        if (m.getTopic() == glim_ros.get_config().param<std::string>("glim_rosbag", "vi_image_topic", "")) {
          glim_ros.insert_vi_image(img_msg->header.stamp.toSec(), cv_image->image);
        } else {
          glim_ros.insert_image(img_msg->header.stamp.toSec(), cv_image->image);
        }
      }

      // Ros-related
      rosgraph_msgs::Clock::Ptr clock_msg(new rosgraph_msgs::Clock);
      clock_msg->clock.sec = m.getTime().sec;
      clock_msg->clock.nsec = m.getTime().nsec;
      clock_pub.publish(clock_msg);
      ros::spinOnce();
    }

    return true;
  };

  // Read all rosbags
  for (const auto& bag_filename : bag_filenames) {
    if (!read_bag(bag_filename, *topics)) {
      break;
    }
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));
  glim_ros.save("/tmp/dump");
  glim_ros.wait();

  return 0;
}