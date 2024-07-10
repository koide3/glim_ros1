#include <glob.h>
#include <thread>
#include <memory>
#include <iostream>
#include <spdlog/spdlog.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/package.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <rosgraph_msgs/Clock.h>

#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros.hpp>

#include <glim_ros/glim_ros.hpp>

class SpeedCounter {
public:
  SpeedCounter(const ros::Time& begin_time, const ros::Time& end_time) : begin_time(begin_time), end_time(end_time), last_real_time(std::chrono::high_resolution_clock::now()) {}

  void update(const ros::Time& stamp) {
    const auto now = std::chrono::high_resolution_clock::now();
    if (now - last_real_time < std::chrono::seconds(15)) {
      return;
    }

    if (last_sim_time.sec && last_sim_time.nsec) {
      const auto real = now - last_real_time;
      const auto sim = stamp - last_sim_time;
      const double playback_speed = sim.toSec() / (std::chrono::duration_cast<std::chrono::nanoseconds>(real).count() / 1e9);

      const double current = (stamp - begin_time).toSec();
      const double duration = (end_time - begin_time).toSec();
      const double percentage = 100.0 * current / duration;

      spdlog::info((boost::format("playback speed:%.3fx %.2fs/%.2fs (%.2f%%)") % playback_speed % current % duration % percentage).str());
    }

    last_sim_time = stamp;
    last_real_time = now;
  }

private:
  const ros::Time begin_time;
  const ros::Time end_time;

  ros::Time last_sim_time;
  std::chrono::high_resolution_clock::time_point last_real_time;
};

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: glim_rosbag input_rosbag_path" << std::endl;
    return 0;
  }

  ros::init(argc, argv, "glim_rosbag");
  ros::NodeHandle nh("~");
  ros::Publisher clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

  ROS_INFO_STREAM("Starting GLIM");

  // Initialize GLIM
  glim::GlimROS glim_ros(nh);

  // List input topics
  glim::Config config_ros(glim::GlobalConfig::get_config_path("config_ros"));
  const std::string imu_topic = config_ros.param<std::string>("glim_ros", "imu_topic", "");
  const std::string points_topic = config_ros.param<std::string>("glim_ros", "points_topic", "");
  const std::string image_topic = config_ros.param<std::string>("glim_ros", "image_topic", "");

  std::vector<std::string> topics = {imu_topic, points_topic, image_topic};
  topics.erase(std::remove_if(topics.begin(), topics.end(), [](const std::string& topic) { return topic.empty(); }), topics.end());

  std::unordered_map<std::string, std::vector<glim::GenericTopicSubscription::Ptr>> subscription_map;
  for (const auto& sub : glim_ros.extension_subscriptions()) {
    topics.push_back(sub->topic);
    subscription_map[sub->topic].push_back(sub);
  }

  spdlog::info("topics:");
  for (const auto& topic : topics) {
    spdlog::info("- {}", topic);
  }

  // List input rosbag filenames
  std::vector<std::string> bag_filenames;

  for (int i = 1; i < argc; i++) {
    std::vector<std::string> filenames;
    glob_t globbuf;
    int ret = glob(argv[i], 0, nullptr, &globbuf);
    for (int i = 0; i < globbuf.gl_pathc; i++) {
      filenames.push_back(globbuf.gl_pathv[i]);
    }
    globfree(&globbuf);

    bag_filenames.insert(bag_filenames.end(), filenames.begin(), filenames.end());
  }
  std::sort(bag_filenames.begin(), bag_filenames.end());

  spdlog::info("bag_filenames:");
  for (const auto& bag_filename : bag_filenames) {
    spdlog::info("- {}", bag_filename);
  }

  // Bag read function
  const auto read_bag = [&](const std::string& bag_filename, const std::vector<std::string>& topics) {
    spdlog::info("opening {}", bag_filename);
    rosbag::Bag bag(bag_filename, rosbag::bagmode::Read);
    if (!bag.isOpen()) {
      spdlog::error("failed to open {}", bag_filename);
      return false;
    }

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    SpeedCounter speed_counter(view.getBeginTime(), view.getEndTime());

    // Read messages
    for (rosbag::MessageInstance const m : view) {
      if (!ros::ok()) {
        return false;
      }
      speed_counter.update(m.getTime());

      const std::string topic = m.getTopic();

      // IMU message
      if (topic == imu_topic) {
        const auto imu_msg = m.instantiate<sensor_msgs::Imu>();
        if (imu_msg) {
          const double stamp = imu_msg->header.stamp.toSec();
          const auto& linear_acc = imu_msg->linear_acceleration;
          const auto& angular_vel = imu_msg->angular_velocity;

          glim_ros.insert_imu(stamp, Eigen::Vector3d(linear_acc.x, linear_acc.y, linear_acc.z), Eigen::Vector3d(angular_vel.x, angular_vel.y, angular_vel.z));
        } else {
          spdlog::error("failed to instantiate IMU message");
        }
      }
      // PointCloud2 message
      else if (topic == points_topic) {
        const auto points_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (points_msg) {
          auto raw_points = glim::extract_raw_points(points_msg);
          glim_ros.insert_frame(raw_points);
        } else {
          spdlog::error("failed to instantiate PointCloud2 message");
        }
      }
      // Image message
      else if (topic == image_topic) {
        // sensor_msgs::CompressedImage
        const auto compressed_img_msg = m.instantiate<sensor_msgs::CompressedImage>();
        if (compressed_img_msg) {
          auto cv_image = cv_bridge::toCvCopy(compressed_img_msg, "bgr8");
          glim_ros.insert_image(compressed_img_msg->header.stamp.toSec(), cv_image->image);
        }

        const auto img_msg = m.instantiate<sensor_msgs::Image>();
        if (img_msg) {
          auto cv_image = cv_bridge::toCvCopy(img_msg, "bgr8");
          glim_ros.insert_image(img_msg->header.stamp.toSec(), cv_image->image);
        }

        if (!compressed_img_msg && !img_msg) {
          spdlog::error("failed to instantiate Image message");
        }
      }

      // Extension modules
      const auto found = subscription_map.find(m.getTopic());
      if (found != subscription_map.end()) {
        for (const auto& sub : found->second) {
          sub->insert_message_instance(m);
        }
      }

      for (int i = 0; i < glim_ros.extensions().size(); i++) {
        if (glim_ros.extensions()[i]->needs_wait()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          spdlog::warn("throttling the mapping process for extension module {}", i);
          i = -1;
        }
      }

      // Publish clock
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
    if (!read_bag(bag_filename, topics)) {
      break;
    }
  }

  glim_ros.wait(nh.param<bool>("auto_quit", false));
  glim_ros.save("/tmp/dump");

  return 0;
}