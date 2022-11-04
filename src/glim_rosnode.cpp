#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros.hpp>

#include <glim_ros/glim_ros.hpp>

class GlimNode {
public:
  GlimNode() : nh("~"), image_transport(nh) {
    ROS_INFO_STREAM("Starting GLIM");
    glim_ros.reset(new glim::GlimROS(nh));

    glim::Config config_ros(glim::GlobalConfig::get_config_path("config_ros"));
    const std::string imu_topic = config_ros.param<std::string>("glim_ros", "imu_topic", "");
    const std::string points_topic = config_ros.param<std::string>("glim_ros", "points_topic", "");
    const std::string image_topic = config_ros.param<std::string>("glim_ros", "image_topic", "");

    image_sub = image_transport.subscribe(image_topic, 100, &GlimNode::image_callback, this);
    imu_sub = nh.subscribe(imu_topic, 1000, &GlimNode::imu_callback, this);
    points_sub = nh.subscribe(points_topic, 100, &GlimNode::points_callback, this);

    ext_subs = glim_ros->extension_subscriptions();
    for (auto& sub : ext_subs) {
      sub->create_subscriber(nh);
    }
  }

  void image_callback(const sensor_msgs::ImageConstPtr& image_msg) {
    auto cv_image = cv_bridge::toCvCopy(image_msg, "bgr8");
    glim_ros->insert_image(image_msg->header.stamp.toSec(), cv_image->image);
  }

  void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg) {
    const double stamp = imu_msg->header.stamp.toSec();
    const auto& linear_acc = imu_msg->linear_acceleration;
    const auto& angular_vel = imu_msg->angular_velocity;

    glim_ros->insert_imu(stamp, Eigen::Vector3d(linear_acc.x, linear_acc.y, linear_acc.z), Eigen::Vector3d(angular_vel.x, angular_vel.y, angular_vel.z));
  }

  void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    auto raw_points = glim::extract_raw_points(points_msg);
    glim_ros->insert_frame(raw_points);
  }

  void spin() {
    while (ros::ok()) {
      ros::spinOnce();
    }

    glim_ros->wait(true);
    glim_ros->save("/tmp/dump");
  }

private:
  ros::NodeHandle nh;

  image_transport::ImageTransport image_transport;
  image_transport::Subscriber image_sub;

  ros::Subscriber imu_sub;
  ros::Subscriber points_sub;
  std::vector<std::shared_ptr<glim::GenericTopicSubscription>> ext_subs;

  std::unique_ptr<glim::GlimROS> glim_ros;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "glim_rosnode");
  GlimNode node;
  node.spin();

  return 0;
}