#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <cerrno>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class SerialOdomSender : public rclcpp::Node
{
public:
  SerialOdomSender()
  : Node("serial_odom_sender"), count_(0)
  {
    // 串口路径和波特率
    serial_port_ = "/dev/ttyAMA0";  // 根据你的实际串口设备修改
    baud_rate_ = B115200;

    RCLCPP_INFO(this->get_logger(), "Attempting to open serial port: %s at %d baud", 
                serial_port_.c_str(), baud_rate_);

    if (!openSerialPort()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
      rclcpp::shutdown();
    } else {
      RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
    }

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom_rf2o", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_INFO_ONCE(this->get_logger(), "Received first odometry message");
        odomCallback(msg);
      });

    RCLCPP_INFO(this->get_logger(), "Node initialized. Waiting for odometry data...");
    RCLCPP_INFO(this->get_logger(), "Data format: [0xAA] [x_high] [x_low] [y_high] [y_low] [z_high] [z_low] [yaw_high] [yaw_low] [0x0A]");
  }

  ~SerialOdomSender()
  {
    if (fd_ != -1) {
      close(fd_);
      RCLCPP_INFO(this->get_logger(), "Serial port closed");
    }
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // 记录接收消息的频率
    if (++count_ % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), "Received %d odometry messages", count_);
    }

    // 转换为厘米和度
    int16_t x_cm = static_cast<int16_t>(msg->pose.pose.position.x * 100.0);
    int16_t y_cm = static_cast<int16_t>(msg->pose.pose.position.y * 100.0);
    int16_t z_cm = static_cast<int16_t>(msg->pose.pose.position.z * 100.0);

    // 提取四元数，转换为欧拉角，取 yaw 并转换为度
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    double roll, pitch, yaw_rad;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_rad);
    int16_t yaw_deg = static_cast<int16_t>(yaw_rad * (180.0 / M_PI));

    // 构建二进制数据包
    uint8_t packet[10];
    packet[0] = 0xAA; // 帧头
    
    // X坐标 (厘米)
    packet[1] = static_cast<uint8_t>((x_cm >> 8) & 0xFF); // 高八位
    packet[2] = static_cast<uint8_t>(x_cm & 0xFF);        // 低八位
    
    // Y坐标 (厘米)
    packet[3] = static_cast<uint8_t>((y_cm >> 8) & 0xFF); // 高八位
    packet[4] = static_cast<uint8_t>(y_cm & 0xFF);        // 低八位
    
    // Z坐标 (厘米)
    packet[5] = static_cast<uint8_t>((z_cm >> 8) & 0xFF); // 高八位
    packet[6] = static_cast<uint8_t>(z_cm & 0xFF);        // 低八位
    
    // Yaw角度 (度)
    packet[7] = static_cast<uint8_t>((yaw_deg >> 8) & 0xFF); // 高八位
    packet[8] = static_cast<uint8_t>(yaw_deg & 0xFF);        // 低八位
    
    packet[9] = 0x0A; // 帧尾

    // 发送数据包
    if (write(fd_, packet, sizeof(packet)) == -1) {
      RCLCPP_WARN(this->get_logger(), "Failed to write to serial port: %s", strerror(errno));
    } else {
      // 每10次发送打印一次数据详情
      if (count_ % 10 == 0) {
        RCLCPP_INFO(this->get_logger(), "Sent [%d]: X=%.2fcm, Y=%.2fcm, Z=%.2fcm, Yaw=%.2f°", 
                    count_,
                    msg->pose.pose.position.x * 100.0,
                    msg->pose.pose.position.y * 100.0,
                    msg->pose.pose.position.z * 100.0,
                    yaw_rad * (180.0 / M_PI));
        logPacketHex(packet, sizeof(packet));
      }
      RCLCPP_DEBUG(this->get_logger(), "Sent binary packet");
    }
  }

  // 记录数据包十六进制内容
  void logPacketHex(const uint8_t* packet, size_t length)
  {
    std::string hex_str;
    for (size_t i = 0; i < length; ++i) {
      char buf[4];
      snprintf(buf, sizeof(buf), "%02X ", packet[i]);
      hex_str += buf;
    }
    RCLCPP_INFO(this->get_logger(), "Packet HEX: %s", hex_str.c_str());
  }

  bool openSerialPort()
  {
    fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error opening serial port: %s", strerror(errno));
      return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr error: %s", strerror(errno));
      close(fd_);
      return false;
    }

    cfsetospeed(&tty, baud_rate_);
    cfsetispeed(&tty, baud_rate_);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                         // disable break processing
    tty.c_lflag = 0;                                // no signaling chars, no echo
    tty.c_oflag = 0;                                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;                            // read doesn't block
    tty.c_cc[VTIME] = 5;                            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls
    tty.c_cflag &= ~(PARENB | PARODD);              // no parity
    tty.c_cflag &= ~CSTOPB;                         // one stop bit
    tty.c_cflag &= ~CRTSCTS;                        // no hardware flow control

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr error: %s", strerror(errno));
      close(fd_);
      return false;
    }

    // 刷新串口缓冲区
    tcflush(fd_, TCIOFLUSH);
    RCLCPP_INFO(this->get_logger(), "Serial port configured successfully");
    return true;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

  std::string serial_port_;
  speed_t baud_rate_;
  int fd_{-1};
  int count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("main"), "Starting serial_odom_sender node");
  auto node = std::make_shared<SerialOdomSender>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  RCLCPP_INFO(rclcpp::get_logger("main"), "Node shutdown completed");
  return 0;
}
