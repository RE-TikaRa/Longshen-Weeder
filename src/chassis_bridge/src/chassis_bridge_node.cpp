#include <boost/asio.hpp>
#include <cstdint>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "muses.pb.h"

class ChassisBridgeNode
{
public:
  ChassisBridgeNode()
    : private_nh_("~")
    , socket_(io_)
    , session_id_(0)
    , seq_(0)
    , logged_in_(false)
  {
    private_nh_.param<std::string>("/chassis/host", host_, "192.168.1.100");
    private_nh_.param("/chassis/port", port_, 9000);
    private_nh_.param<std::string>("/chassis/username", username_, "admin");
    private_nh_.param<std::string>("/chassis/password_md5", password_md5_, "");
    private_nh_.param("/chassis/heartbeat_interval_ms", heartbeat_interval_ms_, 10000);
    private_nh_.param<std::string>("/chassis/map_name", map_name_, "default");
    private_nh_.param<std::string>("/chassis/task_name", task_name_, "");

    status_pub_ = nh_.advertise<std_msgs::String>("/chassis/status", 10, true);
    task_state_pub_ = nh_.advertise<std_msgs::String>("/chassis/task_state", 10, true);

    poll_timer_ = nh_.createTimer(ros::Duration(0.02), &ChassisBridgeNode::poll, this);
    connect_timer_ = nh_.createTimer(ros::Duration(1.0), &ChassisBridgeNode::ensureConnected, this);
    heartbeat_timer_ = nh_.createTimer(ros::Duration(heartbeat_interval_ms_ / 1000.0), &ChassisBridgeNode::heartbeat, this);
  }

private:
  void ensureConnected(const ros::TimerEvent&)
  {
    if (socket_.is_open()) {
      return;
    }

    boost::system::error_code ec;
    boost::asio::ip::tcp::resolver resolver(io_);
    boost::asio::ip::tcp::resolver::results_type endpoints =
        resolver.resolve(host_, std::to_string(port_), ec);
    if (ec) {
      publishStatus("offline");
      return;
    }

    boost::asio::connect(socket_, endpoints, ec);
    if (ec) {
      publishStatus("offline");
      socket_.close();
      return;
    }

    logged_in_ = false;
    session_id_ = 0;
    rx_buffer_.clear();
    sendLogin();
  }

  void poll(const ros::TimerEvent&)
  {
    if (!socket_.is_open()) {
      return;
    }

    boost::system::error_code ec;
    const std::size_t available = socket_.available(ec);
    if (ec) {
      handleDisconnect();
      return;
    }
    if (available == 0) {
      return;
    }

    char data[4096];
    const std::size_t length = socket_.read_some(boost::asio::buffer(data), ec);
    if (ec) {
      handleDisconnect();
      return;
    }

    rx_buffer_.append(data, length);
    parseFrames();
  }

  void parseFrames()
  {
    while (rx_buffer_.size() >= 4) {
      const std::size_t body_size = frameBodySize(rx_buffer_);
      if (body_size == 0 || body_size + 4 > kMaxFrameSize) {
        rx_buffer_.clear();
        return;
      }
      if (rx_buffer_.size() < body_size + 4) {
        return;
      }

      proto::Message msg;
      if (msg.ParseFromArray(rx_buffer_.data() + 4, static_cast<int>(body_size))) {
        handleMessage(msg);
      }
      rx_buffer_.erase(0, body_size + 4);
    }
  }

  void handleMessage(const proto::Message& msg)
  {
    if (msg.type() == proto::MSG_RESPONSE) {
      const proto::Response& response = msg.response();
      if (response.response_type() == proto::Response::RESPONSE_LOGIN) {
        if (response.result().result_state() == proto::Result::RESPONSE_OK) {
          session_id_ = response.login_info().session_id();
          logged_in_ = true;
          publishStatus("online");
          queryStartup();
          startMapTask();
        } else {
          publishStatus("login_failed");
        }
      } else if (response.response_type() == proto::Response::RESPONSE_MAPNAME) {
        publishStatus("map=" + response.map_name());
      } else if (response.response_type() == proto::Response::RESPONSE_TASKING) {
        publishTaskState(response.task_run() ? "task_run=1;task_name=" + response.task_name()
                                             : "task_run=0");
      } else if (response.response_type() == proto::Response::RESPONSE_SYSTEM_STATE) {
        publishSystemState(response.system_state());
      }
    } else if (msg.type() == proto::MSG_NOTIFICATION) {
      const proto::Notification& notify = msg.notification();
      if (notify.notify_type() == proto::Notification::NOTIFY_MOVE_TASK_FINISHED) {
        publishMovementTask(notify.movement_task());
      }
    }
  }

  void sendLogin()
  {
    proto::Message msg;
    msg.set_type(proto::MSG_REQUEST);
    msg.set_seq(seq_++);

    proto::Request* request = msg.mutable_request();
    request->set_request_type(proto::Request::REQUEST_LOGIN);
    proto::LoginRequest* login = request->mutable_login_request();
    login->set_username(username_);
    login->set_password(password_md5_);

    sendMessage(msg);
  }

  void queryStartup()
  {
    sendRequest(proto::Request::REQUEST_MAPNAME);
    sendRequest(proto::Request::REQUEST_TASKING);
    sendRequest(proto::Request::REQUEST_SYSTEM_STATE);
    sendRequest(proto::Request::REQUEST_HARDWARE_STATE);
  }

  void startMapTask()
  {
    proto::Message msg;
    msg.set_type(proto::MSG_COMMAND);
    msg.set_seq(seq_++);
    msg.set_session_id(session_id_);

    proto::Command* command = msg.mutable_command();
    command->set_command(proto::CMD_FULL_MOVEMENT_TASK);
    command->set_task_file_name(task_name_);

    sendMessage(msg);
  }

  void heartbeat(const ros::TimerEvent&)
  {
    if (!socket_.is_open() || !logged_in_) {
      return;
    }

    proto::Message msg;
    msg.set_type(proto::MSG_COMMAND);
    msg.set_seq(seq_++);
    msg.set_session_id(session_id_);
    msg.mutable_command()->set_command(proto::CMD_HEARTBEATS_UPDATE);
    sendMessage(msg);
  }

  void sendRequest(proto::Request::RequestType type)
  {
    proto::Message msg;
    msg.set_type(proto::MSG_REQUEST);
    msg.set_seq(seq_++);
    msg.set_session_id(session_id_);
    msg.mutable_request()->set_request_type(type);
    sendMessage(msg);
  }

  void sendMessage(const proto::Message& msg)
  {
    const std::string body = msg.SerializeAsString();
    const std::size_t body_size = body.size();

    std::string frame;
    frame.resize(4);
    frame[0] = static_cast<char>((body_size >> 24) & 0xff);
    frame[1] = static_cast<char>((body_size >> 16) & 0xff);
    frame[2] = static_cast<char>((body_size >> 8) & 0xff);
    frame[3] = static_cast<char>(body_size & 0xff);
    frame.append(body);

    boost::system::error_code ec;
    boost::asio::write(socket_, boost::asio::buffer(frame), ec);
    if (ec) {
      handleDisconnect();
    }
  }

  void publishSystemState(const proto::SystemState& state)
  {
    std::ostringstream ss;
    ss << "sys_state=" << state.sys_state()
       << ";location_state=" << state.location_state()
       << ";emergency_state=" << state.emergency_state()
       << ";emergency_reason=" << state.emergency_reason()
       << ";gnss_msg_state=" << state.gnss_msg_state()
       << ";fault_code=" << state.fault_code()
       << ";v_x=" << state.mc_state().v_x()
       << ";v_y=" << state.mc_state().v_y()
       << ";w=" << state.mc_state().w();

    std_msgs::String msg;
    msg.data = ss.str();
    status_pub_.publish(msg);

    publishMovementTask(state.movement_state());
  }

  void publishMovementTask(const proto::MovementTask& task)
  {
    std::ostringstream ss;
    ss << "state=" << task.state()
       << ";type=" << task.type()
       << ";remain_distance=" << task.remain_distance()
       << ";remain_time=" << task.remain_time()
       << ";result=" << task.result();

    publishTaskState(ss.str());
  }

  void publishTaskState(const std::string& value)
  {
    std_msgs::String msg;
    msg.data = value;
    task_state_pub_.publish(msg);
  }

  void publishStatus(const std::string& value)
  {
    std_msgs::String msg;
    msg.data = value;
    status_pub_.publish(msg);
  }

  void handleDisconnect()
  {
    logged_in_ = false;
    session_id_ = 0;
    rx_buffer_.clear();
    boost::system::error_code ec;
    socket_.close(ec);
    publishStatus("offline");
  }

  static std::size_t frameBodySize(const std::string& buffer)
  {
    const std::size_t size =
        (static_cast<std::size_t>(static_cast<std::uint8_t>(buffer[0])) << 24) |
        (static_cast<std::size_t>(static_cast<std::uint8_t>(buffer[1])) << 16) |
        (static_cast<std::size_t>(static_cast<std::uint8_t>(buffer[2])) << 8) |
        (static_cast<std::size_t>(static_cast<std::uint8_t>(buffer[3])));
    return size;
  }

  static const std::size_t kMaxFrameSize = 128 * 1024;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher status_pub_;
  ros::Publisher task_state_pub_;
  ros::Timer poll_timer_;
  ros::Timer connect_timer_;
  ros::Timer heartbeat_timer_;
  boost::asio::io_service io_;
  boost::asio::ip::tcp::socket socket_;
  std::string rx_buffer_;
  std::string host_;
  std::string username_;
  std::string password_md5_;
  std::string map_name_;
  std::string task_name_;
  int port_;
  int heartbeat_interval_ms_;
  std::uint64_t session_id_;
  std::uint32_t seq_;
  bool logged_in_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "chassis_bridge");
  ChassisBridgeNode node;
  ros::spin();
  return 0;
}
