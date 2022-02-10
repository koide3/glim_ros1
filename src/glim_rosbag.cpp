#include <glob.h>
#include <thread>
#include <memory>
#include <iostream>

#include <ros/ros.h>
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
#include <glim/common/callbacks.hpp>

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

      glim::notify(glim::NotificationLevel::INFO, (boost::format("playback speed:%.3fx %.2fs/%.2fs (%.2f%%)") % playback_speed % current % duration % percentage).str());
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
  if(argc < 2) {
    std::cerr << "usage: glim_rosbag input_rosbag_path" << std::endl;
    return 0;
  }

  ros::init(argc, argv, "glim_rosbag");
  ros::NodeHandle nh;
  ros::Publisher clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

  // Initialize GLIM
  glim::GlimROS glim_ros;

  glim::Config config_rosbag(ros::package::getPath("glim_ros") + "/config/glim_ros.json");

  // List input topics
  const auto topics = config_rosbag.param<std::vector<std::string>>("glim_rosbag", "topics");
  if(!topics) {
    std::cerr << "error: topics must be specified" << std::endl;
    return 1;
  }

  std::cout << "topics:" << std::endl;
  for(const auto& topic : *topics) {
    std::cout << "- " << topic << std::endl;
  }

  // List input rosbag filenames
  std::vector<std::string> bag_filenames;

  for(int i = 1; i < argc; i++) {
    std::vector<std::string> filenames;
    glob_t globbuf;
    int ret = glob(argv[i], 0, nullptr, &globbuf);
    for(int i = 0; i < globbuf.gl_pathc; i++) {
      filenames.push_back(globbuf.gl_pathv[i]);
    }
    globfree(&globbuf);

    bag_filenames.insert(bag_filenames.end(), filenames.begin(), filenames.end());
  }
  std::sort(bag_filenames.begin(), bag_filenames.end());

  std::cout << "bag_filenames:" << std::endl;
  for(const auto& bag_filename : bag_filenames) {
    std::cout << "- " << bag_filename << std::endl;
  }

  // Bag read function
  const auto read_bag = [&](const std::string& bag_filename, const std::vector<std::string>& topics) {
    std::cout << "opening " << bag_filename << std::endl;
    glim::notify(glim::NotificationLevel::INFO, "opening " + bag_filename);
    rosbag::Bag bag(bag_filename, rosbag::bagmode::Read);
    if(!bag.isOpen()) {
      std::cerr << glim::console::bold_red << "error: failed to open " << bag_filename << glim::console::reset << std::endl;
      return false;
    }

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    SpeedCounter speed_counter(view.getBeginTime(), view.getEndTime());

    // Read messages
    for (rosbag::MessageInstance const m : view) {
      if(!glim_ros.ok()) {
        return false;
      }
      speed_counter.update(m.getTime());

      // sensor_msgs::IMU
      const auto imu_msg = m.instantiate<sensor_msgs::Imu>();
      if(imu_msg) {
        const double stamp = imu_msg->header.stamp.toSec();
        const auto& linear_acc = imu_msg->linear_acceleration;
        const auto& angular_vel = imu_msg->angular_velocity;

        if(m.getTopic() == config_rosbag.param<std::string>("glim_rosbag", "vi_imu_topic", "")) {
          glim_ros.insert_vi_imu(stamp, Eigen::Vector3d(linear_acc.x, linear_acc.y, linear_acc.z), Eigen::Vector3d(angular_vel.x, angular_vel.y, angular_vel.z));
        } else {
          glim_ros.insert_imu(stamp, Eigen::Vector3d(linear_acc.x, linear_acc.y, linear_acc.z), Eigen::Vector3d(angular_vel.x, angular_vel.y, angular_vel.z));
        }
      }

      // sensor_msgs::PointCloud2
      const auto points_msg = m.instantiate<sensor_msgs::PointCloud2>();
      if(points_msg) {
        auto raw_points = glim::extract_raw_points(points_msg);
        glim_ros.insert_frame(raw_points);
      }

      // sensor_msgs::CompressedImage
      const auto compressed_img_msg = m.instantiate<sensor_msgs::CompressedImage>();
      if(compressed_img_msg) {
        auto cv_image = cv_bridge::toCvCopy(compressed_img_msg, "bgr8");
        if(m.getTopic() == config_rosbag.param<std::string>("glim_rosbag", "vi_image_topic", "")) {
          glim_ros.insert_vi_image(compressed_img_msg->header.stamp.toSec(), cv_image->image);
        } else {
          glim_ros.insert_image(compressed_img_msg->header.stamp.toSec(), cv_image->image);
        }
      }

      // sensor_msgs::Image
      const auto img_msg = m.instantiate<sensor_msgs::Image>();
      if(img_msg) {
        auto cv_image = cv_bridge::toCvCopy(img_msg, "bgr8");
        if(m.getTopic() == config_rosbag.param<std::string>("glim_rosbag", "vi_image_topic", "")) {
          glim_ros.insert_vi_image(img_msg->header.stamp.toSec(), cv_image->image);
        } else {
          glim_ros.insert_image(img_msg->header.stamp.toSec(), cv_image->image);
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
  for(const auto& bag_filename : bag_filenames) {
    if(!read_bag(bag_filename, *topics)) {
      break;
    }
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));
  glim_ros.save("/tmp/dump");
  glim_ros.wait();

  return 0;
}