#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>
#include <plant_msgs/PlantPoint.h>
#include <plant_msgs/PlantTrack.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

class DepthFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    track_pub_ = nh_.advertise<plant_msgs::PlantTrack>("/plant/tracks", 10);
    depth_pub_ = nh_.advertise<sensor_msgs::Image>("/camera/depth/image_raw", 1);
    info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/camera/color/camera_info", 1);
    point_sub_ = nh_.subscribe("/plant/points", 10, &DepthFixture::pointCb, this);

    width_ = 640;
    height_ = 480;
    fx_ = 500.0;
    fy_ = 500.0;
    cx_ = 320.0;
    cy_ = 240.0;
    waitForConnection();
  }

  void pointCb(const plant_msgs::PlantPointConstPtr& msg)
  {
    points_.push_back(*msg);
  }

  void waitForConnection()
  {
    ros::Time deadline = ros::Time::now() + ros::Duration(5.0);
    ros::Rate rate(50);
    while (ros::ok() && ros::Time::now() < deadline) {
      if (point_sub_.getNumPublishers() > 0 && track_pub_.getNumSubscribers() > 0 &&
          depth_pub_.getNumSubscribers() > 0 && info_pub_.getNumSubscribers() > 0) {
        return;
      }
      ros::spinOnce();
      rate.sleep();
    }
    FAIL() << "plant_depth node did not connect";
  }

  sensor_msgs::CameraInfo makeInfo()
  {
    sensor_msgs::CameraInfo info;
    info.width = width_;
    info.height = height_;
    info.K[0] = fx_;
    info.K[4] = fy_;
    info.K[2] = cx_;
    info.K[5] = cy_;
    info.K[8] = 1.0;
    return info;
  }

  // uniform depth in millimeters as 16UC1
  sensor_msgs::Image makeUniformDepth(std::uint16_t depth_mm)
  {
    sensor_msgs::Image img;
    img.width = width_;
    img.height = height_;
    img.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    img.is_bigendian = 0;
    img.step = width_ * sizeof(std::uint16_t);
    img.data.resize(img.step * height_);
    auto* p = reinterpret_cast<std::uint16_t*>(img.data.data());
    for (std::size_t i = 0; i < static_cast<std::size_t>(width_) * height_; ++i) {
      p[i] = depth_mm;
    }
    return img;
  }

  sensor_msgs::Image makeZeroDepth()
  {
    return makeUniformDepth(0);
  }

  plant_msgs::PlantTrack makeTrack(const std::string& cls, int xmin, int ymin, int xmax, int ymax)
  {
    plant_msgs::PlantTrack t;
    t.header.stamp = ros::Time::now();
    t.header.frame_id = "camera_color_optical_frame";
    t.track_id = 7;
    t.class_name = cls;
    t.xmin = xmin;
    t.ymin = ymin;
    t.xmax = xmax;
    t.ymax = ymax;
    return t;
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
  ros::Publisher track_pub_;
  ros::Publisher depth_pub_;
  ros::Publisher info_pub_;
  ros::Subscriber point_sub_;
  std::vector<plant_msgs::PlantPoint> points_;

  int width_;
  int height_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
};

TEST_F(DepthFixture, ReprojectsUniformDepth)
{
  info_pub_.publish(makeInfo());
  depth_pub_.publish(makeUniformDepth(500));  // 0.5 m
  spinFor(0.4);

  // box center column: u = (xmin+xmax)/2 = 320 => x ~ 0
  // sample row: v = ymin + (ymax-ymin)*0.75 = 200 + 100*0.75 = 275 => y = (275-240)*0.5/500
  track_pub_.publish(makeTrack("weed", 300, 200, 340, 300));
  spinFor(0.4);

  ASSERT_FALSE(points_.empty());
  const plant_msgs::PlantPoint& pt = points_.back();
  ASSERT_TRUE(pt.depth_valid);
  EXPECT_NEAR(pt.point.z, 0.5, 1e-3);
  EXPECT_NEAR(pt.point.x, (320.0 - cx_) * 0.5 / fx_, 1e-3);
  EXPECT_NEAR(pt.point.y, (275.0 - cy_) * 0.5 / fy_, 1e-3);
}

TEST_F(DepthFixture, ZeroDepthMarkedInvalid)
{
  info_pub_.publish(makeInfo());
  depth_pub_.publish(makeZeroDepth());
  spinFor(0.4);

  track_pub_.publish(makeTrack("weed", 300, 200, 340, 300));
  spinFor(0.4);

  ASSERT_FALSE(points_.empty());
  EXPECT_FALSE(points_.back().depth_valid);
}

TEST_F(DepthFixture, PreservesTrackIdAndClass)
{
  info_pub_.publish(makeInfo());
  depth_pub_.publish(makeUniformDepth(500));
  spinFor(0.4);

  track_pub_.publish(makeTrack("weed", 300, 200, 340, 300));
  spinFor(0.4);

  ASSERT_FALSE(points_.empty());
  EXPECT_EQ(points_.back().track_id, 7u);
  EXPECT_EQ(points_.back().class_name, "weed");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "plant_depth_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
