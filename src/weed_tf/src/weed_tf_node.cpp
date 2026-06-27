#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "weed_tf");
  ros::NodeHandle nh("~");

  std::string parent_frame;
  std::string child_frame;
  nh.param<std::string>("/tf_camera_to_arm/parent_frame", parent_frame, "camera_color_optical_frame");
  nh.param<std::string>("/tf_camera_to_arm/child_frame", child_frame, "arm_base");

  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  nh.param("/tf_camera_to_arm/translation/x", x, 0.0);
  nh.param("/tf_camera_to_arm/translation/y", y, 0.0);
  nh.param("/tf_camera_to_arm/translation/z", z, 0.0);
  nh.param("/tf_camera_to_arm/rotation_rpy/roll", roll, 0.0);
  nh.param("/tf_camera_to_arm/rotation_rpy/pitch", pitch, 0.0);
  nh.param("/tf_camera_to_arm/rotation_rpy/yaw", yaw, 0.0);

  geometry_msgs::TransformStamped transform;
  transform.header.stamp = ros::Time::now();
  transform.header.frame_id = parent_frame;
  transform.child_frame_id = child_frame;
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = z;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  tf2_ros::StaticTransformBroadcaster broadcaster;
  broadcaster.sendTransform(transform);

  ros::spin();
  return 0;
}
