#include <cstdint>
#include <string>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <gtest/gtest.h>
#include <plant_msgs/PlantPoint.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2_ros/static_transform_broadcaster.h>

class DecisionFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    point_pub_ = nh_.advertise<plant_msgs::PlantPoint>("/plant/points", 10);
    serial_pub_ = nh_.advertise<std_msgs::String>("/serial/status", 10, true);
    arm_pub_ = nh_.advertise<std_msgs::String>("/arm/state", 10, true);
    chassis_pub_ = nh_.advertise<std_msgs::String>("/chassis/status", 10, true);

    target_sub_ = nh_.subscribe("/weed/target", 10, &DecisionFixture::targetCb, this);
    state_sub_ = nh_.subscribe("/weed/state", 10, &DecisionFixture::stateCb, this);

    broadcastIdentityTf();
    waitForConnection();
  }

  void targetCb(const plant_msgs::PlantPointConstPtr& msg)
  {
    targets_.push_back(*msg);
  }

  void stateCb(const std_msgs::StringConstPtr& msg)
  {
    states_.push_back(msg->data);
    last_state_ = msg->data;
  }

  // identity transform: camera_color_optical_frame -> arm_base
  void broadcastIdentityTf()
  {
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = "camera_color_optical_frame";
    tf.child_frame_id = "arm_base";
    tf.transform.rotation.w = 1.0;
    static_broadcaster_.sendTransform(tf);
  }

  void waitForConnection()
  {
    ros::Time deadline = ros::Time::now() + ros::Duration(5.0);
    ros::Rate rate(50);
    while (ros::ok() && ros::Time::now() < deadline) {
      if (point_pub_.getNumSubscribers() > 0 && target_sub_.getNumPublishers() > 0) {
        return;
      }
      ros::spinOnce();
      rate.sleep();
    }
    FAIL() << "weed_decision node did not connect";
  }

  plant_msgs::PlantPoint makePoint(std::uint32_t id, const std::string& cls, bool valid,
                                   double x, double y, double z)
  {
    plant_msgs::PlantPoint p;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = "camera_color_optical_frame";
    p.track_id = id;
    p.class_name = cls;
    p.point.x = x;
    p.point.y = y;
    p.point.z = z;
    p.depth_valid = valid;
    return p;
  }

  void publishStable(std::uint32_t id, const std::string& cls, bool valid,
                     double x, double y, double z, int frames)
  {
    for (int i = 0; i < frames; ++i) {
      point_pub_.publish(makePoint(id, cls, valid, x, y, z));
      spinFor(0.1);
    }
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

  ros::NodeHandle nh_;
  ros::Publisher point_pub_;
  ros::Publisher serial_pub_;
  ros::Publisher arm_pub_;
  ros::Publisher chassis_pub_;
  ros::Subscriber target_sub_;
  ros::Subscriber state_sub_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;

  std::vector<plant_msgs::PlantPoint> targets_;
  std::vector<std::string> states_;
  std::string last_state_;
};

TEST_F(DecisionFixture, SendsTargetAfterStableFramesWhenSerialOnline)
{
  publishString(serial_pub_, "online");
  spinFor(0.2);

  // in-workspace weed point, 3 stable frames
  publishStable(1, "weed", true, 0.3, 0.0, 0.1, 3);
  spinFor(0.3);

  ASSERT_FALSE(targets_.empty());
  EXPECT_EQ(targets_.back().track_id, 1u);
  EXPECT_EQ(targets_.back().class_name, "weed");
  EXPECT_EQ(last_state_, "SENT");
}

TEST_F(DecisionFixture, DoesNotSendWhenSerialOffline)
{
  publishString(serial_pub_, "offline");
  spinFor(0.2);

  publishStable(2, "weed", true, 0.3, 0.0, 0.1, 4);
  spinFor(0.3);

  EXPECT_TRUE(targets_.empty());
}

TEST_F(DecisionFixture, IgnoresCropAndOutOfWorkspace)
{
  publishString(serial_pub_, "online");
  spinFor(0.2);

  publishStable(3, "crop", true, 0.3, 0.0, 0.1, 4);   // crop ignored
  publishStable(4, "weed", true, 2.0, 0.0, 0.1, 4);   // out of workspace
  spinFor(0.3);

  EXPECT_TRUE(targets_.empty());
}

TEST_F(DecisionFixture, FullHandshakeReturnsToIdle)
{
  publishString(serial_pub_, "online");
  spinFor(0.2);
  publishStable(5, "weed", true, 0.3, 0.0, 0.1, 3);
  spinFor(0.3);
  ASSERT_EQ(last_state_, "SENT");

  publishString(arm_pub_, "ACCEPTED");
  spinFor(0.2);
  EXPECT_EQ(last_state_, "WAIT_ARM");

  publishString(arm_pub_, "DONE");
  spinFor(0.2);
  EXPECT_EQ(last_state_, "IDLE");
}

TEST_F(DecisionFixture, ChassisEmergencyEntersAndClears)
{
  publishString(serial_pub_, "online");
  spinFor(0.2);

  publishString(chassis_pub_, "sys_state=1;emergency_state=1;fault_code=0");
  spinFor(0.3);
  EXPECT_EQ(last_state_, "EMERGENCY");

  publishString(chassis_pub_, "sys_state=1;emergency_state=0;fault_code=0");
  spinFor(0.3);
  EXPECT_EQ(last_state_, "IDLE");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "weed_decision_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
