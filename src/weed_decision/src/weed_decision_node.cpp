#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <string>

#include <geometry_msgs/PointStamped.h>
#include <plant_msgs/PlantPoint.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class WeedDecisionNode
{
public:
  WeedDecisionNode()
    : private_nh_("~")
    , tf_listener_(tf_buffer_)
    , state_(IDLE)
    , active_id_(0)
    , serial_online_(false)
    , chassis_emergency_(false)
  {
    private_nh_.param("/arm_workspace/x_min", x_min_, 0.10);
    private_nh_.param("/arm_workspace/x_max", x_max_, 0.45);
    private_nh_.param("/arm_workspace/y_min", y_min_, -0.20);
    private_nh_.param("/arm_workspace/y_max", y_max_, 0.20);
    private_nh_.param("/arm_workspace/z_min", z_min_, 0.00);
    private_nh_.param("/arm_workspace/z_max", z_max_, 0.25);
    private_nh_.param("/tracker/stable_frames", stable_frames_, 3);
    private_nh_.param("/arm_workspace/arm_frame", arm_frame_, std::string("arm_base"));
    private_nh_.param("/arm_workspace/accepted_timeout_s", accepted_timeout_s_, 2.0);

    point_sub_ = nh_.subscribe("/plant/points", 10, &WeedDecisionNode::pointCallback, this);
    serial_status_sub_ = nh_.subscribe("/serial/status", 10, &WeedDecisionNode::serialStatusCallback, this);
    arm_state_sub_ = nh_.subscribe("/arm/state", 10, &WeedDecisionNode::armStateCallback, this);
    chassis_status_sub_ = nh_.subscribe("/chassis/status", 10, &WeedDecisionNode::chassisStatusCallback, this);

    target_pub_ = nh_.advertise<plant_msgs::PlantPoint>("/weed/target", 10);
    state_pub_ = nh_.advertise<std_msgs::String>("/weed/state", 10, true);

    timeout_timer_ = nh_.createTimer(ros::Duration(0.1), &WeedDecisionNode::checkTimeout, this);

    publishState();
  }

private:
  enum State
  {
    IDLE,
    SENT,
    WAIT_ARM,
    EMERGENCY
  };

  struct Candidate
  {
    geometry_msgs::Point point;
    int stable_count;
  };

  void pointCallback(const plant_msgs::PlantPointConstPtr& msg)
  {
    if (state_ == EMERGENCY) {
      return;
    }
    if (msg->class_name != "weed" || !msg->depth_valid) {
      return;
    }

    geometry_msgs::Point arm_point;
    if (!transformToArm(*msg, arm_point)) {
      return;
    }
    if (!inWorkspace(arm_point)) {
      candidates_.erase(msg->track_id);
      return;
    }

    Candidate& candidate = candidates_[msg->track_id];
    candidate.point = arm_point;
    ++candidate.stable_count;

    if (state_ == IDLE) {
      selectAndSend();
    }
  }

  void selectAndSend()
  {
    std::uint32_t best_id = 0;
    double best_distance = std::numeric_limits<double>::max();
    bool found = false;

    const double center_x = (x_min_ + x_max_) * 0.5;
    const double center_y = (y_min_ + y_max_) * 0.5;

    for (const auto& entry : candidates_) {
      if (entry.second.stable_count < stable_frames_) {
        continue;
      }
      if (processed_.count(entry.first) || failed_.count(entry.first)) {
        continue;
      }
      const double dx = entry.second.point.x - center_x;
      const double dy = entry.second.point.y - center_y;
      const double distance = dx * dx + dy * dy;
      if (distance < best_distance) {
        best_distance = distance;
        best_id = entry.first;
        found = true;
      }
    }

    if (!found || !serial_online_) {
      return;
    }

    plant_msgs::PlantPoint target;
    target.header.stamp = ros::Time::now();
    target.header.frame_id = arm_frame_;
    target.track_id = best_id;
    target.class_name = "weed";
    target.point = candidates_[best_id].point;
    target.depth_valid = true;
    target_pub_.publish(target);

    active_id_ = best_id;
    state_ = SENT;
    accepted_deadline_ = ros::Time::now() + ros::Duration(accepted_timeout_s_);
    publishState();
  }

  void armStateCallback(const std_msgs::StringConstPtr& msg)
  {
    if (state_ == IDLE || state_ == EMERGENCY) {
      return;
    }

    const std::string& arm_state = msg->data;
    if (arm_state == "ACCEPTED" || arm_state == "BUSY") {
      state_ = WAIT_ARM;
    } else if (arm_state == "DONE") {
      processed_.insert(active_id_);
      candidates_.erase(active_id_);
      returnToIdle();
    } else if (arm_state == "ERROR") {
      failed_.insert(active_id_);
      candidates_.erase(active_id_);
      returnToIdle();
    }
  }

  void serialStatusCallback(const std_msgs::StringConstPtr& msg)
  {
    serial_online_ = (msg->data == "online");
    evaluateEmergency();
  }

  void chassisStatusCallback(const std_msgs::StringConstPtr& msg)
  {
    chassis_emergency_ = parseChassisEmergency(msg->data);
    evaluateEmergency();
  }

  void evaluateEmergency()
  {
    const bool arm_busy = (state_ == SENT || state_ == WAIT_ARM);
    const bool serial_lost_while_busy = arm_busy && !serial_online_;

    if (chassis_emergency_ || serial_lost_while_busy) {
      if (state_ != EMERGENCY) {
        state_ = EMERGENCY;
        publishState();
      }
    } else if (state_ == EMERGENCY) {
      state_ = IDLE;
      publishState();
    }
  }

  void returnToIdle()
  {
    state_ = IDLE;
    active_id_ = 0;
    publishState();
    selectAndSend();
  }

  bool transformToArm(const plant_msgs::PlantPoint& msg, geometry_msgs::Point& out)
  {
    geometry_msgs::PointStamped in;
    in.header = msg.header;
    in.point = msg.point;

    try {
      geometry_msgs::PointStamped transformed;
      tf_buffer_.transform(in, transformed, arm_frame_, ros::Duration(0.1));
      out = transformed.point;
      return true;
    } catch (const tf2::TransformException& e) {
      ROS_WARN_THROTTLE(5.0, "weed_decision: tf transform failed %s", e.what());
      return false;
    }
  }

  bool inWorkspace(const geometry_msgs::Point& point) const
  {
    return point.x >= x_min_ && point.x <= x_max_ &&
           point.y >= y_min_ && point.y <= y_max_ &&
           point.z >= z_min_ && point.z <= z_max_;
  }

  static bool parseChassisEmergency(const std::string& status)
  {
    const std::size_t pos = status.find("emergency_state=");
    if (pos == std::string::npos) {
      return false;
    }
    const char value = status[pos + 16];
    return value != '0';
  }

  void publishState()
  {
    std_msgs::String msg;
    switch (state_) {
      case IDLE:
        msg.data = "IDLE";
        break;
      case SENT:
        msg.data = "SENT";
        break;
      case WAIT_ARM:
        msg.data = "WAIT_ARM";
        break;
      case EMERGENCY:
        msg.data = "EMERGENCY";
        break;
    }
    state_pub_.publish(msg);
  }

  void checkTimeout(const ros::TimerEvent&)
  {
    if (state_ == SENT && ros::Time::now() > accepted_deadline_) {
      returnToIdle();
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber point_sub_;
  ros::Subscriber serial_status_sub_;
  ros::Subscriber arm_state_sub_;
  ros::Subscriber chassis_status_sub_;
  ros::Publisher target_pub_;
  ros::Publisher state_pub_;
  ros::Timer timeout_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::map<std::uint32_t, Candidate> candidates_;
  std::set<std::uint32_t> processed_;
  std::set<std::uint32_t> failed_;

  State state_;
  std::uint32_t active_id_;
  bool serial_online_;
  bool chassis_emergency_;
  ros::Time accepted_deadline_;

  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double z_min_;
  double z_max_;
  int stable_frames_;
  std::string arm_frame_;
  double accepted_timeout_s_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "weed_decision");
  WeedDecisionNode node;
  ros::spin();
  return 0;
}
