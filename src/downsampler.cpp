#include <atomic>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <gtsam_ext/types/frame.hpp>
#include <gtsam_ext/types/frame_cpu.hpp>
#include <glim/util/ros_cloud_converter.hpp>

class Downsampler {
public:
  Downsampler() {}
  virtual ~Downsampler() {}

  virtual gtsam_ext::Frame::Ptr downsample(const gtsam_ext::Frame::Ptr& frame) const = 0;
};

class RandomDownsampler : public Downsampler {
public:
  RandomDownsampler(const double sampling_rate) : sampling_rate(sampling_rate) {}

  virtual gtsam_ext::Frame::Ptr downsample(const gtsam_ext::Frame::Ptr& frame) const override {
    std::mt19937 mt(frame->size() + (random_seed++));
    return gtsam_ext::random_sampling(frame, sampling_rate, mt);
  }

private:
  const double sampling_rate;
  mutable std::atomic_ulong random_seed;
};

class VoxelgridDownsampler : public Downsampler {
public:
  VoxelgridDownsampler(const double resolution) : resolution(resolution) {}

  virtual gtsam_ext::Frame::Ptr downsample(const gtsam_ext::Frame::Ptr& frame) const override {
    //
    return gtsam_ext::voxelgrid_sampling(frame, resolution);
  }

private:
  const double resolution;
};

class RandomgridDownsampler : public Downsampler {
public:
  RandomgridDownsampler(const double sampling_rate, const double resolution) : sampling_rate(sampling_rate), resolution(resolution) {}

  virtual gtsam_ext::Frame::Ptr downsample(const gtsam_ext::Frame::Ptr& frame) const override {
    std::mt19937 mt(frame->size() + (random_seed++));
    return gtsam_ext::randomgrid_sampling(frame, resolution, sampling_rate, mt);
  }

private:
  const double sampling_rate;
  const double resolution;
  mutable std::atomic_ulong random_seed;
};

class Node {
public:
  Node() : nh("~") {
    const std::string input_topic = nh.param<std::string>("input_topic", "points");
    const std::string output_topic = nh.param<std::string>("output_topic", "downsampled");

    points_pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 10);
    points_sub = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 10, &Node::points_callback, this);
  }

  std::unique_ptr<Downsampler> create_downsampler(const int num_points) {
    const std::string mode = nh.param<std::string>("mode", "RANDOM");
    const double resolution = nh.param<double>("resolution", 0.25);
    const int target_num_points = nh.param<int>("target_num_points", -1);
    double sampling_rate = nh.param<double>("sampling_rate", 0.1);

    if (target_num_points > 0) {
      sampling_rate = std::min(static_cast<double>(target_num_points) / num_points, 1.0);
      std::cout << "sampling_rate:" << sampling_rate << std::endl;
    }

    std::unique_ptr<Downsampler> downsampler;
    if (mode == "RANDOM") {
      downsampler.reset(new RandomDownsampler(sampling_rate));
    } else if (mode == "VOXELGRID") {
      downsampler.reset(new VoxelgridDownsampler(resolution));
    } else if (mode == "RANDOMGRID") {
      downsampler.reset(new RandomgridDownsampler(sampling_rate, resolution));
    } else {
      std::cerr << "unknown sampling mode:" << mode << std::endl;
      abort();
    }

    return std::move(downsampler);
  }

  gtsam_ext::Frame::Ptr remove_nan_points(const gtsam_ext::Frame::Ptr& points) {
    std::vector<int> indices;
    indices.reserve(points->size());

    for (int i = 0; i < points->size(); i++) {
      if (points->points[i].array().isFinite().all()) {
        indices.push_back(i);
      }
    }

    return gtsam_ext::sample(points, indices);
  }

  void points_callback(const sensor_msgs::PointCloud2::ConstPtr& points_msg) {
    auto points = glim::extract_raw_points(points_msg);

    if (!downsampler) {
      downsampler = create_downsampler(points->size());
    }

    auto frame = std::make_shared<gtsam_ext::Frame>();
    frame->num_points = points->size();
    frame->times = points->times.data();
    frame->points = points->points.data();

    frame = remove_nan_points(frame);
    frame = downsampler->downsample(frame);

    std::vector<double> times;
    if (nh.param<bool>("replace_timestamps", false)) {
      times.resize(frame->size(), 0.0);
      frame->times = times.data();
    }

    auto msg = glim::frame_to_pointcloud2(points_msg->header.frame_id, points_msg->header.stamp.toSec(), *frame);
    points_pub.publish(msg);

    std::cout << points->size() << " => " << frame->size() << std::endl;
  }

private:
  ros::NodeHandle nh;

  ros::Subscriber points_sub;
  ros::Publisher points_pub;

  std::unique_ptr<Downsampler> downsampler;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "downsampler");

  Node node;

  ros::spin();

  return 0;
}