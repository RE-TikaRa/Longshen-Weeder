#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <plant_msgs/PlantDetection.h>
#include <plant_msgs/PlantDetectionArray.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace
{
class TrtLogger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char* msg) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      ROS_WARN("[TensorRT] %s", msg);
    }
  }
};

struct Detection
{
  float xmin;
  float ymin;
  float xmax;
  float ymax;
  float score;
  int class_id;
};
}  // namespace

class PlantDetectionNode
{
public:
  PlantDetectionNode()
    : private_nh_("~")
    , input_index_(-1)
    , output_index_(-1)
    , input_size_(0)
    , output_size_(0)
  {
    std::string engine_rel;
    std::string classes_rel;
    private_nh_.param<std::string>("/detector/engine_path", engine_rel, "models/plant_detector.engine");
    private_nh_.param<std::string>("/detector/classes_path", classes_rel, "models/classes.txt");
    private_nh_.param("/detector/input_width", input_width_, 640);
    private_nh_.param("/detector/input_height", input_height_, 640);
    private_nh_.param("/detector/confidence_threshold", confidence_threshold_, 0.35);
    private_nh_.param("/detector/nms_threshold", nms_threshold_, 0.45);

    const std::string package_path = ros::package::getPath("weed_bringup");
    engine_path_ = package_path + "/" + engine_rel;
    classes_path_ = package_path + "/" + classes_rel;

    loadClasses();

    if (!loadEngine()) {
      ROS_ERROR("plant_detection: failed to load engine %s", engine_path_.c_str());
      return;
    }

    detection_pub_ = nh_.advertise<plant_msgs::PlantDetectionArray>("/plant/detections", 10);
    image_sub_ = nh_.subscribe("/camera/color/image_raw", 1, &PlantDetectionNode::imageCallback, this);
  }

  ~PlantDetectionNode()
  {
    for (void* buffer : device_buffers_) {
      if (buffer) {
        cudaFree(buffer);
      }
    }
    if (stream_) {
      cudaStreamDestroy(stream_);
    }
  }

private:
  void loadClasses()
  {
    std::ifstream file(classes_path_);
    std::string line;
    while (std::getline(file, line)) {
      if (!line.empty()) {
        classes_.push_back(line);
      }
    }
  }

  bool loadEngine()
  {
    std::ifstream file(engine_path_, std::ios::binary);
    if (!file.good()) {
      return false;
    }

    file.seekg(0, std::ios::end);
    const std::size_t size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> data(size);
    file.read(data.data(), size);

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) {
      return false;
    }
    engine_.reset(runtime_->deserializeCudaEngine(data.data(), size));
    if (!engine_) {
      return false;
    }
    context_.reset(engine_->createExecutionContext());
    if (!context_) {
      return false;
    }

    device_buffers_.resize(engine_->getNbBindings(), nullptr);
    for (int i = 0; i < engine_->getNbBindings(); ++i) {
      const nvinfer1::Dims dims = engine_->getBindingDimensions(i);
      std::size_t volume = 1;
      for (int d = 0; d < dims.nbDims; ++d) {
        volume *= static_cast<std::size_t>(dims.d[d]);
      }
      cudaMalloc(&device_buffers_[i], volume * sizeof(float));
      if (engine_->bindingIsInput(i)) {
        input_index_ = i;
        input_size_ = volume;
      } else {
        output_index_ = i;
        output_size_ = volume;
      }
    }

    cudaStreamCreate(&stream_);
    return input_index_ >= 0 && output_index_ >= 0;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& e) {
      ROS_WARN("plant_detection: cv_bridge error %s", e.what());
      return;
    }

    const cv::Mat& frame = cv_ptr->image;
    float scale = 0.0f;
    int pad_x = 0;
    int pad_y = 0;
    std::vector<float> input = letterbox(frame, scale, pad_x, pad_y);

    cudaMemcpyAsync(device_buffers_[input_index_], input.data(), input_size_ * sizeof(float),
                    cudaMemcpyHostToDevice, stream_);
    context_->enqueueV2(device_buffers_.data(), stream_, nullptr);

    std::vector<float> output(output_size_);
    cudaMemcpyAsync(output.data(), device_buffers_[output_index_], output_size_ * sizeof(float),
                    cudaMemcpyDeviceToHost, stream_);
    cudaStreamSynchronize(stream_);

    std::vector<Detection> detections = decode(output, scale, pad_x, pad_y, frame.cols, frame.rows);
    publish(detections, msg->header);
  }

  std::vector<float> letterbox(const cv::Mat& frame, float& scale, int& pad_x, int& pad_y) const
  {
    scale = std::min(static_cast<float>(input_width_) / frame.cols,
                     static_cast<float>(input_height_) / frame.rows);
    const int resized_w = static_cast<int>(std::round(frame.cols * scale));
    const int resized_h = static_cast<int>(std::round(frame.rows * scale));
    pad_x = (input_width_ - resized_w) / 2;
    pad_y = (input_height_ - resized_h) / 2;

    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(resized_w, resized_h));
    cv::Mat canvas(input_height_, input_width_, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(canvas(cv::Rect(pad_x, pad_y, resized_w, resized_h)));

    cv::Mat rgb;
    cv::cvtColor(canvas, rgb, cv::COLOR_BGR2RGB);

    std::vector<float> input(input_size_);
    const int channel_size = input_width_ * input_height_;
    for (int y = 0; y < input_height_; ++y) {
      for (int x = 0; x < input_width_; ++x) {
        const cv::Vec3b pixel = rgb.at<cv::Vec3b>(y, x);
        const int idx = y * input_width_ + x;
        input[idx] = pixel[0] / 255.0f;
        input[channel_size + idx] = pixel[1] / 255.0f;
        input[2 * channel_size + idx] = pixel[2] / 255.0f;
      }
    }
    return input;
  }

  std::vector<Detection> decode(const std::vector<float>& output, float scale, int pad_x, int pad_y,
                                int img_w, int img_h) const
  {
    const int num_classes = static_cast<int>(classes_.size());
    const int attrs = 4 + num_classes;
    const int num_boxes = static_cast<int>(output_size_) / attrs;

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> class_ids;

    for (int i = 0; i < num_boxes; ++i) {
      const float* row = output.data() + i * attrs;
      int best_class = 0;
      float best_score = 0.0f;
      for (int c = 0; c < num_classes; ++c) {
        const float score = row[4 + c];
        if (score > best_score) {
          best_score = score;
          best_class = c;
        }
      }
      if (best_score < confidence_threshold_) {
        continue;
      }

      const float cx = row[0];
      const float cy = row[1];
      const float w = row[2];
      const float h = row[3];
      const float left = (cx - w * 0.5f - pad_x) / scale;
      const float top = (cy - h * 0.5f - pad_y) / scale;
      const float width = w / scale;
      const float height = h / scale;

      boxes.emplace_back(cv::Rect(static_cast<int>(left), static_cast<int>(top),
                                  static_cast<int>(width), static_cast<int>(height)));
      scores.push_back(best_score);
      class_ids.push_back(best_class);
    }

    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, scores, static_cast<float>(confidence_threshold_),
                      static_cast<float>(nms_threshold_), keep);

    std::vector<Detection> detections;
    detections.reserve(keep.size());
    for (int idx : keep) {
      Detection det;
      det.xmin = std::max(0, boxes[idx].x);
      det.ymin = std::max(0, boxes[idx].y);
      det.xmax = std::min(img_w - 1, boxes[idx].x + boxes[idx].width);
      det.ymax = std::min(img_h - 1, boxes[idx].y + boxes[idx].height);
      det.score = scores[idx];
      det.class_id = class_ids[idx];
      detections.push_back(det);
    }
    return detections;
  }

  void publish(const std::vector<Detection>& detections, const std_msgs::Header& header)
  {
    plant_msgs::PlantDetectionArray array;
    array.header = header;
    array.detections.reserve(detections.size());

    for (const Detection& det : detections) {
      plant_msgs::PlantDetection msg;
      msg.header = header;
      msg.class_name = (det.class_id >= 0 && det.class_id < static_cast<int>(classes_.size()))
                           ? classes_[det.class_id]
                           : "";
      msg.confidence = det.score;
      msg.xmin = static_cast<int>(det.xmin);
      msg.ymin = static_cast<int>(det.ymin);
      msg.xmax = static_cast<int>(det.xmax);
      msg.ymax = static_cast<int>(det.ymax);
      array.detections.push_back(msg);
    }

    detection_pub_.publish(array);
  }

  template <typename T>
  struct TrtDeleter
  {
    void operator()(T* ptr) const
    {
      if (ptr) {
        ptr->destroy();
      }
    }
  };

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber image_sub_;
  ros::Publisher detection_pub_;

  TrtLogger logger_;
  std::unique_ptr<nvinfer1::IRuntime, TrtDeleter<nvinfer1::IRuntime>> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine, TrtDeleter<nvinfer1::ICudaEngine>> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext, TrtDeleter<nvinfer1::IExecutionContext>> context_;
  std::vector<void*> device_buffers_;
  cudaStream_t stream_ = nullptr;

  std::vector<std::string> classes_;
  std::string engine_path_;
  std::string classes_path_;
  int input_index_;
  int output_index_;
  std::size_t input_size_;
  std::size_t output_size_;
  int input_width_;
  int input_height_;
  double confidence_threshold_;
  double nms_threshold_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plant_detection");
  PlantDetectionNode node;
  ros::spin();
  return 0;
}
