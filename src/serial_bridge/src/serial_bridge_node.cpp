#include <boost/asio.hpp>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

#include <plant_msgs/PlantPoint.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

class SerialBridgeNode
{
public:
  SerialBridgeNode()
    : private_nh_("~")
    , serial_(io_)
  {
    private_nh_.param<std::string>("/serial/port", port_, "/dev/ttyTHS1");
    private_nh_.param("/serial/baudrate", baudrate_, 115200);
    private_nh_.param("/serial/ping_interval_ms", ping_interval_ms_, 1000);
    private_nh_.param("/serial/pong_timeout_ms", pong_timeout_ms_, 3000);

    target_sub_ = nh_.subscribe("/weed/target", 10, &SerialBridgeNode::targetCallback, this);
    status_pub_ = nh_.advertise<std_msgs::String>("/serial/status", 10, true);
    arm_state_pub_ = nh_.advertise<std_msgs::String>("/arm/state", 10, true);

    open();
    poll_timer_ = nh_.createTimer(ros::Duration(0.02), &SerialBridgeNode::poll, this);
    ping_timer_ = nh_.createTimer(ros::Duration(ping_interval_ms_ / 1000.0), &SerialBridgeNode::ping, this);
  }

private:
  void open()
  {
    boost::system::error_code ec;
    serial_.open(port_, ec);
    if (ec) {
      setOnline(false);
      return;
    }

    serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
    serial_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    last_pong_ = ros::Time::now();
    setOnline(true);
  }

  void targetCallback(const plant_msgs::PlantPointConstPtr& msg)
  {
    if (!serial_.is_open() || !msg->depth_valid || msg->class_name != "weed") {
      return;
    }

    const std::uint16_t id = static_cast<std::uint16_t>(msg->track_id);
    const std::int16_t x = metersToMillimeters(msg->point.x);
    const std::int16_t y = metersToMillimeters(msg->point.y);
    const std::int16_t z = metersToMillimeters(msg->point.z);
    writeFrame(0x01, id, x, y, z, 0x00);
  }

  void poll(const ros::TimerEvent&)
  {
    if (!serial_.is_open()) {
      return;
    }

    boost::system::error_code ec;
    const std::size_t available = serial_.available(ec);
    if (ec || available == 0) {
      return;
    }

    char data[256];
    const std::size_t length = serial_.read_some(boost::asio::buffer(data), ec);
    if (ec) {
      setOnline(false);
      serial_.close();
      return;
    }

    rx_buffer_.append(data, length);
    std::size_t pos = std::string::npos;
    while ((pos = rx_buffer_.find('\n')) != std::string::npos) {
      const std::string line = rx_buffer_.substr(0, pos);
      rx_buffer_.erase(0, pos + 1);
      parseLine(line);
    }
  }

  void ping(const ros::TimerEvent&)
  {
    if (!serial_.is_open()) {
      open();
      return;
    }

    if ((ros::Time::now() - last_pong_).toSec() > pong_timeout_ms_ / 1000.0) {
      setOnline(false);
    }

    writeFrame(0x05, 0, 0, 0, 0, 0);
  }

  void writeFrame(std::uint8_t cmd, std::uint16_t id, std::int16_t x, std::int16_t y, std::int16_t z, std::uint8_t err)
  {
    const std::uint8_t sum = checksum(cmd, id, x, y, z, err);
    std::ostringstream ss;
    ss << "AA55";
    appendHex(ss, cmd, 2);
    appendHex(ss, id, 4);
    appendHex(ss, static_cast<std::uint16_t>(x), 4);
    appendHex(ss, static_cast<std::uint16_t>(y), 4);
    appendHex(ss, static_cast<std::uint16_t>(z), 4);
    appendHex(ss, err, 2);
    appendHex(ss, sum, 2);
    ss << '\n';

    const std::string frame = ss.str();
    boost::asio::write(serial_, boost::asio::buffer(frame));
  }

  void parseLine(const std::string& line)
  {
    if (line.size() < 26 || line.substr(0, 4) != "AA55") {
      return;
    }

    last_pong_ = ros::Time::now();
    setOnline(true);
    const std::uint8_t cmd = static_cast<std::uint8_t>(readHex(line, 4, 2));
    std_msgs::String state;
    state.data = stateName(cmd);
    arm_state_pub_.publish(state);
  }

  std::string stateName(std::uint8_t cmd) const
  {
    switch (cmd) {
      case 0x81:
        return "ACCEPTED";
      case 0x82:
        return "BUSY";
      case 0x83:
        return "DONE";
      case 0x84:
        return "ERROR";
      case 0x85:
        return "READY";
      case 0x86:
        return "PONG";
      default:
        return "UNKNOWN";
    }
  }

  void setOnline(bool online)
  {
    if (online_initialized_ && online == online_) {
      return;
    }
    online_ = online;
    online_initialized_ = true;
    std_msgs::String msg;
    msg.data = online ? "online" : "offline";
    status_pub_.publish(msg);
  }

  static std::int16_t metersToMillimeters(double value)
  {
    return static_cast<std::int16_t>(std::lround(value * 1000.0));
  }

  static std::uint8_t checksum(std::uint8_t cmd, std::uint16_t id, std::int16_t x, std::int16_t y, std::int16_t z, std::uint8_t err)
  {
    std::uint32_t sum = cmd;
    sum += (id >> 8) & 0xff;
    sum += id & 0xff;
    const std::uint16_t ux = static_cast<std::uint16_t>(x);
    const std::uint16_t uy = static_cast<std::uint16_t>(y);
    const std::uint16_t uz = static_cast<std::uint16_t>(z);
    sum += (ux >> 8) & 0xff;
    sum += ux & 0xff;
    sum += (uy >> 8) & 0xff;
    sum += uy & 0xff;
    sum += (uz >> 8) & 0xff;
    sum += uz & 0xff;
    sum += err;
    return static_cast<std::uint8_t>(sum & 0xff);
  }

  static void appendHex(std::ostringstream& ss, std::uint32_t value, int width)
  {
    ss << std::uppercase << std::hex << std::setw(width) << std::setfill('0') << value;
  }

  static std::uint32_t readHex(const std::string& line, std::size_t pos, std::size_t count)
  {
    return static_cast<std::uint32_t>(std::stoul(line.substr(pos, count), nullptr, 16));
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber target_sub_;
  ros::Publisher status_pub_;
  ros::Publisher arm_state_pub_;
  ros::Timer poll_timer_;
  ros::Timer ping_timer_;
  boost::asio::io_service io_;
  boost::asio::serial_port serial_;
  std::string rx_buffer_;
  std::string port_;
  int baudrate_;
  int ping_interval_ms_;
  int pong_timeout_ms_;
  ros::Time last_pong_;
  bool online_ = false;
  bool online_initialized_ = false;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "serial_bridge");
  SerialBridgeNode node;
  ros::spin();
  return 0;
}
