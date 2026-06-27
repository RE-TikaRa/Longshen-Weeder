#include <cstdint>
#include <map>
#include <sstream>
#include <string>

#include <geometry_msgs/Point.h>
#include <plant_msgs/PlantPoint.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class WeedMonitorNode
{
public:
  WeedMonitorNode()
    : private_nh_("~")
  {
    private_nh_.param("/arm_workspace/x_min", x_min_, 0.10);
    private_nh_.param("/arm_workspace/x_max", x_max_, 0.45);
    private_nh_.param("/arm_workspace/y_min", y_min_, -0.20);
    private_nh_.param("/arm_workspace/y_max", y_max_, 0.20);
    private_nh_.param("/arm_workspace/z_min", z_min_, 0.00);
    private_nh_.param("/arm_workspace/z_max", z_max_, 0.25);
    private_nh_.param("/arm_workspace/arm_frame", arm_frame_, std::string("arm_base"));
    private_nh_.param("point_timeout_s", point_timeout_s_, 1.0);

    point_sub_ = nh_.subscribe("/plant/points", 10, &WeedMonitorNode::pointCallback, this);
    target_sub_ = nh_.subscribe("/weed/target", 10, &WeedMonitorNode::targetCallback, this);
    weed_state_sub_ = nh_.subscribe("/weed/state", 10, &WeedMonitorNode::weedStateCallback, this);
    chassis_status_sub_ = nh_.subscribe("/chassis/status", 10, &WeedMonitorNode::chassisStatusCallback, this);
    serial_status_sub_ = nh_.subscribe("/serial/status", 10, &WeedMonitorNode::serialStatusCallback, this);
    arm_state_sub_ = nh_.subscribe("/arm/state", 10, &WeedMonitorNode::armStateCallback, this);

    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/weed/markers", 10);

    publish_timer_ = nh_.createTimer(ros::Duration(0.1), &WeedMonitorNode::publish, this);
  }

private:
  struct TrackedPoint
  {
    geometry_msgs::Point point;
    std::string class_name;
    ros::Time stamp;
  };

  void pointCallback(const plant_msgs::PlantPointConstPtr& msg)
  {
    if (!msg->depth_valid) {
      points_.erase(msg->track_id);
      return;
    }
    TrackedPoint& tracked = points_[msg->track_id];
    tracked.point = msg->point;
    tracked.class_name = msg->class_name;
    tracked.stamp = ros::Time::now();
  }

  void targetCallback(const plant_msgs::PlantPointConstPtr& msg)
  {
    active_target_ = *msg;
    has_active_target_ = true;
    active_target_stamp_ = ros::Time::now();
  }

  void weedStateCallback(const std_msgs::StringConstPtr& msg)
  {
    weed_state_ = msg->data;
  }

  void chassisStatusCallback(const std_msgs::StringConstPtr& msg)
  {
    chassis_status_ = msg->data;
  }

  void serialStatusCallback(const std_msgs::StringConstPtr& msg)
  {
    serial_status_ = msg->data;
  }

  void armStateCallback(const std_msgs::StringConstPtr& msg)
  {
    arm_state_ = msg->data;
  }

  void publish(const ros::TimerEvent&)
  {
    const ros::Time now = ros::Time::now();
    expirePoints(now);

    visualization_msgs::MarkerArray markers;
    int id = 0;

    markers.markers.push_back(makeWorkspace(id++, now));

    for (const auto& entry : points_) {
      markers.markers.push_back(makePointMarker(id++, now, entry.first, entry.second));
      markers.markers.push_back(makeLabelMarker(id++, now, entry.first, entry.second));
    }

    if (has_active_target_ && (now - active_target_stamp_).toSec() <= point_timeout_s_) {
      markers.markers.push_back(makeActiveTargetMarker(id++, now));
    }

    markers.markers.push_back(makeStatusMarker(id++, now));

    marker_pub_.publish(markers);
  }

  void expirePoints(const ros::Time& now)
  {
    for (auto it = points_.begin(); it != points_.end();) {
      if ((now - it->second.stamp).toSec() > point_timeout_s_) {
        it = points_.erase(it);
      } else {
        ++it;
      }
    }
  }

  visualization_msgs::Marker makeWorkspace(int id, const ros::Time& now) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = arm_frame_;
    marker.header.stamp = now;
    marker.ns = "arm_workspace";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = (x_min_ + x_max_) * 0.5;
    marker.pose.position.y = (y_min_ + y_max_) * 0.5;
    marker.pose.position.z = (z_min_ + z_max_) * 0.5;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = x_max_ - x_min_;
    marker.scale.y = y_max_ - y_min_;
    marker.scale.z = z_max_ - z_min_;
    marker.color.r = 0.0f;
    marker.color.g = 0.6f;
    marker.color.b = 1.0f;
    marker.color.a = 0.15f;
    return marker;
  }

  visualization_msgs::Marker makePointMarker(int id, const ros::Time& now, std::uint32_t track_id,
                                             const TrackedPoint& tracked) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = arm_frame_;
    marker.header.stamp = now;
    marker.ns = "plant_points";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = tracked.point;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    if (tracked.class_name == "weed") {
      marker.color.r = 1.0f;
      marker.color.g = 0.2f;
      marker.color.b = 0.0f;
    } else {
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.2f;
    }
    marker.color.a = 0.9f;
    (void)track_id;
    return marker;
  }

  visualization_msgs::Marker makeLabelMarker(int id, const ros::Time& now, std::uint32_t track_id,
                                             const TrackedPoint& tracked) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = arm_frame_;
    marker.header.stamp = now;
    marker.ns = "plant_labels";
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = tracked.point;
    marker.pose.position.z += 0.04;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.03;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    std::ostringstream ss;
    ss << tracked.class_name << " #" << track_id;
    marker.text = ss.str();
    return marker;
  }

  visualization_msgs::Marker makeActiveTargetMarker(int id, const ros::Time& now) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = arm_frame_;
    marker.header.stamp = now;
    marker.ns = "active_target";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = active_target_.point;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.06;
    marker.scale.y = 0.06;
    marker.scale.z = 0.01;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.8f;
    return marker;
  }

  visualization_msgs::Marker makeStatusMarker(int id, const ros::Time& now) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = arm_frame_;
    marker.header.stamp = now;
    marker.ns = "status_text";
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = (x_min_ + x_max_) * 0.5;
    marker.pose.position.y = y_max_ + 0.10;
    marker.pose.position.z = z_max_ + 0.05;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.03;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    std::ostringstream ss;
    ss << "weed: " << (weed_state_.empty() ? "?" : weed_state_) << "\n"
       << "arm: " << (arm_state_.empty() ? "?" : arm_state_) << "\n"
       << "serial: " << (serial_status_.empty() ? "?" : serial_status_) << "\n"
       << "chassis: " << (chassis_status_.empty() ? "?" : chassis_status_);
    marker.text = ss.str();
    return marker;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber point_sub_;
  ros::Subscriber target_sub_;
  ros::Subscriber weed_state_sub_;
  ros::Subscriber chassis_status_sub_;
  ros::Subscriber serial_status_sub_;
  ros::Subscriber arm_state_sub_;
  ros::Publisher marker_pub_;
  ros::Timer publish_timer_;

  std::map<std::uint32_t, TrackedPoint> points_;

  plant_msgs::PlantPoint active_target_;
  bool has_active_target_ = false;
  ros::Time active_target_stamp_;

  std::string weed_state_;
  std::string arm_state_;
  std::string serial_status_;
  std::string chassis_status_;

  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double z_min_;
  double z_max_;
  std::string arm_frame_;
  double point_timeout_s_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "weed_monitor");
  WeedMonitorNode node;
  ros::spin();
  return 0;
}
