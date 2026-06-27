#include <cstdint>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <plant_msgs/PlantPoint.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

class MonitorFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    point_pub_ = nh_.advertise<plant_msgs::PlantPoint>("/plant/points", 10);
    target_pub_ = nh_.advertise<plant_msgs::PlantPoint>("/weed/target", 10);
    weed_state_pub_ = nh_.advertise<std_msgs::String>("/weed/state", 10, true);
    serial_pub_ = nh_.advertise<std_msgs::String>("/serial/status", 10, true);
    arm_pub_ = nh_.advertise<std_msgs::String>("/arm/state", 10, true);
    chassis_pub_ = nh_.advertise<std_msgs::String>("/chassis/status", 10, true);

    marker_sub_ = nh_.subscribe("/weed/markers", 10, &MonitorFixture::markerCb, this);
    waitForConnection();
  }

  void markerCb(const visualization_msgs::MarkerArrayConstPtr& msg)
  {
    last_markers_ = *msg;
    ++marker_count_;
  }

  void waitForConnection()
  {
    ros::Time deadline = ros::Time::now() + ros::Duration(5.0);
    ros::Rate rate(50);
    while (ros::ok() && ros::Time::now() < deadline) {
      if (marker_sub_.getNumPublishers() > 0 && point_pub_.getNumSubscribers() > 0 &&
          target_pub_.getNumSubscribers() > 0) {
        return;
      }
      ros::spinOnce();
      rate.sleep();
    }
    FAIL() << "weed_monitor node did not connect";
  }

  plant_msgs::PlantPoint makePoint(std::uint32_t id, const std::string& cls, double x, double y, double z)
  {
    plant_msgs::PlantPoint p;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = "arm_base";
    p.track_id = id;
    p.class_name = cls;
    p.point.x = x;
    p.point.y = y;
    p.point.z = z;
    p.depth_valid = true;
    return p;
  }

  void publishString(ros::Publisher& pub, const std::string& value)
  {
    std_msgs::String msg;
    msg.data = value;
    pub.publish(msg);
  }

  void spinFor(double seconds)
  {
    ros::Time deadline = ros::Time::now() + ros::Duration(seconds);
    ros::Rate rate(100);
    while (ros::ok() && ros::Time::now() < deadline) {
      ros::spinOnce();
      rate.sleep();
    }
  }

  std::set<std::string> namespaces() const
  {
    std::set<std::string> ns;
    for (const auto& m : last_markers_.markers) {
      ns.insert(m.ns);
    }
    return ns;
  }

  ros::NodeHandle nh_;
  ros::Publisher point_pub_;
  ros::Publisher target_pub_;
  ros::Publisher weed_state_pub_;
  ros::Publisher serial_pub_;
  ros::Publisher arm_pub_;
  ros::Publisher chassis_pub_;
  ros::Subscriber marker_sub_;

  visualization_msgs::MarkerArray last_markers_;
  int marker_count_ = 0;
};

TEST_F(MonitorFixture, AlwaysPublishesWorkspaceAndStatus)
{
  spinFor(0.4);
  ASSERT_GT(marker_count_, 0);
  std::set<std::string> ns = namespaces();
  EXPECT_EQ(ns.count("arm_workspace"), 1u);
  EXPECT_EQ(ns.count("status_text"), 1u);
}

TEST_F(MonitorFixture, ShowsValidPointAsMarker)
{
  point_pub_.publish(makePoint(1, "weed", 0.3, 0.0, 0.1));
  spinFor(0.4);

  std::set<std::string> ns = namespaces();
  EXPECT_EQ(ns.count("plant_points"), 1u);
  EXPECT_EQ(ns.count("plant_labels"), 1u);
}

TEST_F(MonitorFixture, ShowsActiveTarget)
{
  target_pub_.publish(makePoint(2, "weed", 0.3, 0.0, 0.1));
  spinFor(0.4);

  std::set<std::string> ns = namespaces();
  EXPECT_EQ(ns.count("active_target"), 1u);
}

TEST_F(MonitorFixture, StatusTextReflectsInputs)
{
  publishString(weed_state_pub_, "SENT");
  publishString(serial_pub_, "online");
  publishString(arm_pub_, "BUSY");
  publishString(chassis_pub_, "sys_state=1;emergency_state=0");
  spinFor(0.4);

  std::string status_text;
  for (const auto& m : last_markers_.markers) {
    if (m.ns == "status_text") {
      status_text = m.text;
    }
  }
  ASSERT_FALSE(status_text.empty());
  EXPECT_NE(status_text.find("SENT"), std::string::npos);
  EXPECT_NE(status_text.find("online"), std::string::npos);
  EXPECT_NE(status_text.find("BUSY"), std::string::npos);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "weed_monitor_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
