#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <plant_msgs/PlantPoint.h>
#include <plant_msgs/PlantTrack.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

class PlantDepthNode
{
public:
  PlantDepthNode()
    : nh_()
    , private_nh_("~")
  {
    private_nh_.param("target_v_ratio", target_v_ratio_, 0.75);
    private_nh_.param("roi_size", roi_size_, 7);
    private_nh_.param("min_depth", min_depth_, 0.05);
    private_nh_.param("max_depth", max_depth_, 1.50);

    track_sub_ = nh_.subscribe("/plant/tracks", 10, &PlantDepthNode::trackCallback, this);
    depth_sub_ = nh_.subscribe("/camera/depth/image_raw", 1, &PlantDepthNode::depthCallback, this);
    info_sub_ = nh_.subscribe("/camera/color/camera_info", 1, &PlantDepthNode::infoCallback, this);
    point_pub_ = nh_.advertise<plant_msgs::PlantPoint>("/plant/points", 10);
  }

private:
  void depthCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    depth_ = msg;
  }

  void infoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    info_ = msg;
  }

  void trackCallback(const plant_msgs::PlantTrackConstPtr& msg)
  {
    plant_msgs::PlantPoint point;
    point.header = msg->header;
    point.track_id = msg->track_id;
    point.class_name = msg->class_name;
    point.depth_valid = false;

    if (!depth_ || !info_) {
      point_pub_.publish(point);
      return;
    }

    const int u = static_cast<int>((msg->xmin + msg->xmax) * 0.5);
    const int v = static_cast<int>(msg->ymin + (msg->ymax - msg->ymin) * target_v_ratio_);
    const double depth = medianDepth(u, v);

    if (std::isfinite(depth)) {
      const double fx = info_->K[0];
      const double fy = info_->K[4];
      const double cx = info_->K[2];
      const double cy = info_->K[5];

      point.point.x = (u - cx) * depth / fx;
      point.point.y = (v - cy) * depth / fy;
      point.point.z = depth;
      point.depth_valid = true;
    }

    point_pub_.publish(point);
  }

  double medianDepth(int u, int v) const
  {
    const int half = roi_size_ / 2;
    std::vector<double> values;
    values.reserve(roi_size_ * roi_size_);

    for (int y = v - half; y <= v + half; ++y) {
      for (int x = u - half; x <= u + half; ++x) {
        const double depth = depthAt(x, y);
        if (std::isfinite(depth) && depth >= min_depth_ && depth <= max_depth_) {
          values.push_back(depth);
        }
      }
    }

    if (values.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    const std::size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    return values[mid];
  }

  double depthAt(int x, int y) const
  {
    if (x < 0 || y < 0 || x >= static_cast<int>(depth_->width) || y >= static_cast<int>(depth_->height)) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    const std::size_t offset = y * depth_->step + x * byteDepth();

    if (depth_->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
        depth_->encoding == sensor_msgs::image_encodings::MONO16) {
      const std::uint16_t value = *reinterpret_cast<const std::uint16_t*>(&depth_->data[offset]);
      if (value == 0) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      return value * 0.001;
    }

    if (depth_->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      const float value = *reinterpret_cast<const float*>(&depth_->data[offset]);
      return value;
    }

    return std::numeric_limits<double>::quiet_NaN();
  }

  std::size_t byteDepth() const
  {
    if (depth_->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      return sizeof(float);
    }
    return sizeof(std::uint16_t);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber track_sub_;
  ros::Subscriber depth_sub_;
  ros::Subscriber info_sub_;
  ros::Publisher point_pub_;
  sensor_msgs::ImageConstPtr depth_;
  sensor_msgs::CameraInfoConstPtr info_;
  double target_v_ratio_;
  int roi_size_;
  double min_depth_;
  double max_depth_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plant_depth");
  PlantDepthNode node;
  ros::spin();
  return 0;
}
