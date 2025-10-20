#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <cerrno>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using namespace std::chrono_literals;

class SerialSender : public rclcpp::Node
{
public:
  SerialSender()
  : Node("serial_sender"), count_(0)
  {
    // 读取launch参数
    this->declare_parameter<std::string>("serial_port", "/dev/ttyAMA0");
    this->declare_parameter<int>("baud_rate", 921600);

    serial_port_ = this->get_parameter("serial_port").as_string();
    int baud_rate_param = this->get_parameter("baud_rate").as_int();

    baud_rate_ = getBaud(baud_rate_param);

    if (!openSerialPort()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
      rclcpp::shutdown();
      return;
    }

    tf_timer_ = this->create_wall_timer(100ms, std::bind(&SerialSender::sendTF, this)); // 10Hz
    scan_timer_ = this->create_wall_timer(1s, std::bind(&SerialSender::sendScan, this)); // 1Hz
    map_timer_ = this->create_wall_timer(5s, std::bind(&SerialSender::sendMap, this)); // 0.2Hz

    RCLCPP_INFO(this->get_logger(), "SerialSender initialized on port %s at %d baud", serial_port_.c_str(), baud_rate_param);
  }

  ~SerialSender()
  {
    if (fd_ != -1) close(fd_);
  }

private:
  speed_t getBaud(int baud)
  {
    switch(baud)
    {
      case 921600: return B921600;
      case 115200: return B115200;
      case 460800: return B460800;
      case 230400: return B230400;
      default: return B115200;
    }
  }

  bool openSerialPort()
  {
    fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error opening serial port: %s", strerror(errno));
      return false;
    }

    struct termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr error: %s", strerror(errno));
      close(fd_);
      return false;
    }

    cfsetospeed(&tty, baud_rate_);
    cfsetispeed(&tty, baud_rate_);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr error: %s", strerror(errno));
      close(fd_);
      return false;
    }

    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void sendTF()
  {
    std::vector<uint8_t> data = {0x01};
    appendData(data, "tf example");
    sendPacket(data);
  }

  void sendScan()
  {
    std::vector<uint8_t> data = {0x02};
    appendData(data, "scan example");
    sendPacket(data);
  }

  void sendMap()
  {
    std::vector<uint8_t> data = {0x03};
    appendData(data, "map example");
    sendPacket(data);
  }

  void appendData(std::vector<uint8_t> &packet, const std::string &msg)
  {
    for (char c : msg) packet.push_back(static_cast<uint8_t>(c));
  }

  void sendPacket(const std::vector<uint8_t> &data)
  {
    std::vector<uint8_t> packet;
    packet.push_back(0xAA);
    packet.insert(packet.end(), data.begin(), data.end());
    packet.push_back(0x0A);

    if (write(fd_, packet.data(), packet.size()) == -1) {
      RCLCPP_WARN(this->get_logger(), "Failed to write to serial port: %s", strerror(errno));
    }
  }

  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  std::string serial_port_;
  speed_t baud_rate_;
  int fd_{-1};
  int count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialSender>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}