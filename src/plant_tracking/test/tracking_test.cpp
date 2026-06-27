#include <cstdint>
#include <map>
#include <vector>

#include <gtest/gtest.h>
#include <plant_msgs/PlantDetection.h>
#include <plant_msgs/PlantDetectionArray.h>
#include <plant_msgs/PlantTrack.h>
#include <ros/ros.h>

class TrackingFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    pub_ = nh_.advertise<plant_msgs::PlantDetectionArray>("/plant/detections", 10);
    sub_ = nh_.subscribe("/plant/tracks", 50, &TrackingFixture::trackCb, this);
    waitForConnection();
  }

  void trackCb(const plant_msgs::PlantTrackConstPtr& msg)
  {
    tracks_.push_back(*msg);
  }

  void waitForConnection()
  {
    ros::Time deadline = ros::Time::now() + ros::Duration(5.0);
    ros::Rate rate(50);
    while (ros::ok() && ros::Time::now() < deadline) {
      if (pub_.getNumSubscribers() > 0 && sub_.getNumPublishers() > 0) {
        return;
      }
      ros::spinOnce();
      rate.sleep();
    }
    FAIL() << "plant_tracking node did not connect";
  }

  plant_msgs::PlantDetection makeDet(const std::string& cls, int xmin, int ymin, int xmax, int ymax)
  {
    plant_msgs::PlantDetection det;
    det.class_name = cls;
    det.confidence = 0.9f;
    det.xmin = xmin;
    det.ymin = ymin;
    det.xmax = xmax;
    det.ymax = ymax;
    return det;
  }

  void publishFrame(const std::vector<plant_msgs::PlantDetection>& dets)
  {
    plant_msgs::PlantDetectionArray array;
    array.header.stamp = ros::Time::now();
    array.detections = dets;
    pub_.publish(array);
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
  ros::Publisher pub_;
  ros::Subscriber sub_;
  std::vector<plant_msgs::PlantTrack> tracks_;
};

TEST_F(TrackingFixture, AssignsIdAndKeepsItAcrossOverlappingFrames)
{
  publishFrame({makeDet("weed", 100, 100, 200, 200)});
  spinFor(0.3);
  // overlapping box (high IOU) -> same track id reused
  publishFrame({makeDet("weed", 105, 105, 205, 205)});
  spinFor(0.3);

  ASSERT_GE(tracks_.size(), 2u);
  EXPECT_EQ(tracks_.front().class_name, "weed");
  EXPECT_EQ(tracks_.front().track_id, tracks_.back().track_id);
  EXPECT_EQ(tracks_.back().state, 0);  // CANDIDATE_2D
}

TEST_F(TrackingFixture, DifferentClassDoesNotMatch)
{
  publishFrame({makeDet("weed", 100, 100, 200, 200)});
  spinFor(0.3);
  // same location but different class -> must be a new track id
  publishFrame({makeDet("crop", 100, 100, 200, 200)});
  spinFor(0.3);

  ASSERT_GE(tracks_.size(), 2u);
  std::map<std::string, std::uint32_t> id_by_class;
  for (const auto& t : tracks_) {
    id_by_class[t.class_name] = t.track_id;
  }
  ASSERT_EQ(id_by_class.count("weed"), 1u);
  ASSERT_EQ(id_by_class.count("crop"), 1u);
  EXPECT_NE(id_by_class["weed"], id_by_class["crop"]);
}

TEST_F(TrackingFixture, NonOverlappingBoxGetsNewId)
{
  publishFrame({makeDet("weed", 0, 0, 50, 50)});
  spinFor(0.3);
  publishFrame({makeDet("weed", 400, 400, 450, 450)});
  spinFor(0.3);

  ASSERT_GE(tracks_.size(), 2u);
  EXPECT_NE(tracks_.front().track_id, tracks_.back().track_id);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "plant_tracking_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
