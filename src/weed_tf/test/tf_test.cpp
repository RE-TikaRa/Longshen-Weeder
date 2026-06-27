#include <gtest/gtest.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

TEST(WeedTfTest, PublishesStaticCameraToArmTransform)
{
  ros::NodeHandle nh;
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);

  ros::Time deadline = ros::Time::now() + ros::Duration(5.0);
  ros::Rate rate(20);
  bool found = false;
  geometry_msgs::TransformStamped tf;

  while (ros::ok() && ros::Time::now() < deadline) {
    if (buffer.canTransform("camera_color_optical_frame", "arm_base", ros::Time(0))) {
      tf = buffer.lookupTransform("camera_color_optical_frame", "arm_base", ros::Time(0));
      found = true;
      break;
    }
    ros::spinOnce();
    rate.sleep();
  }

  ASSERT_TRUE(found) << "camera_color_optical_frame -> arm_base transform not available";
  // default config is identity (all zeros)
  EXPECT_NEAR(tf.transform.translation.x, 0.0, 1e-6);
  EXPECT_NEAR(tf.transform.translation.y, 0.0, 1e-6);
  EXPECT_NEAR(tf.transform.translation.z, 0.0, 1e-6);
  EXPECT_NEAR(tf.transform.rotation.w, 1.0, 1e-6);
  EXPECT_NEAR(tf.transform.rotation.x, 0.0, 1e-6);
  EXPECT_NEAR(tf.transform.rotation.y, 0.0, 1e-6);
  EXPECT_NEAR(tf.transform.rotation.z, 0.0, 1e-6);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "weed_tf_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
