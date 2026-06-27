#include <algorithm>
#include <cstdint>
#include <limits>
#include <vector>

#include <plant_msgs/PlantDetection.h>
#include <plant_msgs/PlantDetectionArray.h>
#include <plant_msgs/PlantTrack.h>
#include <ros/ros.h>

class PlantTrackingNode
{
public:
  PlantTrackingNode()
    : private_nh_("~")
    , next_track_id_(0)
  {
    private_nh_.param("/tracker/iou_threshold", iou_threshold_, 0.3);
    private_nh_.param("/tracker/max_age", max_age_, 5);

    detection_sub_ = nh_.subscribe("/plant/detections", 10, &PlantTrackingNode::detectionCallback, this);
    track_pub_ = nh_.advertise<plant_msgs::PlantTrack>("/plant/tracks", 10);
  }

private:
  struct Track
  {
    std::uint32_t id;
    std::string class_name;
    int xmin;
    int ymin;
    int xmax;
    int ymax;
    int age;
  };

  void detectionCallback(const plant_msgs::PlantDetectionArrayConstPtr& msg)
  {
    const std::size_t num_tracks = tracks_.size();
    std::vector<bool> track_matched(num_tracks, false);

    for (const plant_msgs::PlantDetection& det : msg->detections) {
      int best_track = -1;
      double best_iou = iou_threshold_;

      for (std::size_t i = 0; i < num_tracks; ++i) {
        if (track_matched[i] || tracks_[i].class_name != det.class_name) {
          continue;
        }
        const double iou = computeIou(det, tracks_[i]);
        if (iou >= best_iou) {
          best_iou = iou;
          best_track = static_cast<int>(i);
        }
      }

      Track* track = nullptr;
      if (best_track >= 0) {
        track_matched[best_track] = true;
        track = &tracks_[best_track];
      } else {
        Track created;
        created.id = next_track_id_++;
        created.class_name = det.class_name;
        created.age = 0;
        tracks_.push_back(created);
        track = &tracks_.back();
      }

      track->class_name = det.class_name;
      track->xmin = det.xmin;
      track->ymin = det.ymin;
      track->xmax = det.xmax;
      track->ymax = det.ymax;
      track->age = 0;

      publishTrack(*track, det.confidence, msg->header);
    }

    for (std::size_t i = 0; i < num_tracks; ++i) {
      if (!track_matched[i]) {
        ++tracks_[i].age;
      }
    }

    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [this](const Track& track) { return track.age > max_age_; }),
                  tracks_.end());
  }

  double computeIou(const plant_msgs::PlantDetection& det, const Track& track) const
  {
    const int ix1 = std::max(det.xmin, track.xmin);
    const int iy1 = std::max(det.ymin, track.ymin);
    const int ix2 = std::min(det.xmax, track.xmax);
    const int iy2 = std::min(det.ymax, track.ymax);

    const int iw = ix2 - ix1;
    const int ih = iy2 - iy1;
    if (iw <= 0 || ih <= 0) {
      return 0.0;
    }

    const double intersection = static_cast<double>(iw) * ih;
    const double area_det = static_cast<double>(det.xmax - det.xmin) * (det.ymax - det.ymin);
    const double area_track = static_cast<double>(track.xmax - track.xmin) * (track.ymax - track.ymin);
    const double denom = area_det + area_track - intersection;
    if (denom <= 0.0) {
      return 0.0;
    }
    return intersection / denom;
  }

  void publishTrack(const Track& track, float confidence, const std_msgs::Header& header)
  {
    plant_msgs::PlantTrack msg;
    msg.header = header;
    msg.track_id = track.id;
    msg.class_name = track.class_name;
    msg.confidence = confidence;
    msg.xmin = track.xmin;
    msg.ymin = track.ymin;
    msg.xmax = track.xmax;
    msg.ymax = track.ymax;
    msg.state = 0;
    track_pub_.publish(msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber detection_sub_;
  ros::Publisher track_pub_;
  std::vector<Track> tracks_;
  std::uint32_t next_track_id_;
  double iou_threshold_;
  int max_age_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plant_tracking");
  PlantTrackingNode node;
  ros::spin();
  return 0;
}
