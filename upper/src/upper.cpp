#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <set>
#include <mutex>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <cerrno>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;

class SerialSender : public rclcpp::Node
{
public:
    SerialSender() : Node("serial_sender"), fd_(-1), listen_sock_(-1)
    {
        // ---- 参数声明 ----
        this->declare_parameter<std::string>("serial_port", "/dev/ttyS3");
        this->declare_parameter<int>("baud_rate", 921600);
        this->declare_parameter<std::string>("tcp_ip", "0.0.0.0");
        this->declare_parameter<int>("tcp_port", 9999);

        serial_port_ = this->get_parameter("serial_port").as_string();
        baud_rate_   = this->get_parameter("baud_rate").as_int();
        tcp_ip_      = this->get_parameter("tcp_ip").as_string();
        tcp_port_    = this->get_parameter("tcp_port").as_int();

        openSerialPort();
        startTcpServer();

        // ---- 订阅话题 ----
        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10, std::bind(&SerialSender::tfCallback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SerialSender::scanCallback, this, std::placeholders::_1));
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 1, std::bind(&SerialSender::mapCallback, this, std::placeholders::_1));

        // ---- TCP 连接检测 ----
        tcp_accept_timer_ = this->create_wall_timer(200ms, std::bind(&SerialSender::acceptPendingClients, this));

        // ---- 速率统计 ----
        rate_timer_ = this->create_wall_timer(1s, std::bind(&SerialSender::logRates, this));
    }

    ~SerialSender()
    {
        if (fd_ != -1) close(fd_);
        if (listen_sock_ != -1) close(listen_sock_);
        for (int c : clients_) close(c);
    }

private:
    // ======== 串口函数 ========
    void openSerialPort()
    {
        fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to open serial port %s: %s",
                        serial_port_.c_str(), strerror(errno));
            return;
        }

        struct termios tty {};
        if (tcgetattr(fd_, &tty) != 0)
        {
            RCLCPP_WARN(this->get_logger(), "tcgetattr failed: %s", strerror(errno));
            return;
        }

        cfsetospeed(&tty, baud_rate_);
        cfsetispeed(&tty, baud_rate_);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag = IGNPAR;
        tty.c_oflag = 0;
        tty.c_lflag = 0;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 1;

        tcsetattr(fd_, TCSANOW, &tty);
        RCLCPP_INFO(this->get_logger(), "Serial port %s opened, baud %d", serial_port_.c_str(), baud_rate_);
    }

    // ======== TCP Server ========
    void startTcpServer()
    {
        listen_sock_ = socket(AF_INET, SOCK_STREAM, 0);
        if (listen_sock_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket: %s", strerror(errno));
            return;
        }

        int opt = 1;
        setsockopt(listen_sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        sockaddr_in addr {};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(tcp_port_);
        addr.sin_addr.s_addr = inet_addr(tcp_ip_.c_str());

        if (bind(listen_sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Bind failed: %s", strerror(errno));
            return;
        }

        listen(listen_sock_, 5);
        fcntl(listen_sock_, F_SETFL, O_NONBLOCK);
        RCLCPP_INFO(this->get_logger(), "TCP server listening on %s:%d", tcp_ip_.c_str(), tcp_port_);
    }

    void acceptPendingClients()
    {
        sockaddr_in caddr {};
        socklen_t clen = sizeof(caddr);
        int client_fd = accept(listen_sock_, (struct sockaddr *)&caddr, &clen);
        if (client_fd < 0) return;

        fcntl(client_fd, F_SETFL, O_NONBLOCK);
        clients_.insert(client_fd);
        RCLCPP_INFO(this->get_logger(), "New TCP client connected, fd=%d", client_fd);
    }

    // ======== ROS2 话题回调 (收到即发送) ========
    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        if (msg->transforms.empty()) return;

        const auto &t = msg->transforms[0];
        json j;
        j["topic"] = "tf";
        j["frame_id"] = t.header.frame_id;
        j["child_frame_id"] = t.child_frame_id;
        j["x"] = t.transform.translation.x;
        j["y"] = t.transform.translation.y;
        j["z"] = t.transform.translation.z;
        j["qx"] = t.transform.rotation.x;
        j["qy"] = t.transform.rotation.y;
        j["qz"] = t.transform.rotation.z;
        j["qw"] = t.transform.rotation.w;

        sendJsonPacket(0x01, j.dump());
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (msg->ranges.empty()) return;

        json j;
        j["topic"] = "scan";
        j["angle_min"] = msg->angle_min;
        j["angle_max"] = msg->angle_max;
        j["angle_increment"] = msg->angle_increment;
        j["range_count"] = msg->ranges.size();
        j["ranges"] = json::array();

        for (size_t i = 0; i < std::min<size_t>(msg->ranges.size(), 30); ++i)
            j["ranges"].push_back(msg->ranges[i]);

        sendJsonPacket(0x02, j.dump());
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (msg->data.empty()) return;

        json j;
        j["topic"] = "map";
        j["width"] = msg->info.width;
        j["height"] = msg->info.height;
        j["resolution"] = msg->info.resolution;
        j["origin_x"] = msg->info.origin.position.x;
        j["origin_y"] = msg->info.origin.position.y;

        // RLE 压缩
        std::vector<std::pair<int, int>> rle;
        const auto &data = msg->data;
        if (!data.empty()) {
            int current = data[0];
            int count = 1;
            for (size_t i = 1; i < data.size(); ++i) {
                if (data[i] == current && count < 255)
                    count++;
                else {
                    rle.emplace_back(current, count);
                    current = data[i];
                    count = 1;
                }
            }
            rle.emplace_back(current, count);
        }

        json rle_json = json::array();
        for (auto &p : rle)
            rle_json.push_back({p.first, p.second});
        j["rle"] = rle_json;

        sendJsonPacket(0x03, j.dump());
    }

    // ======== JSON 发送 ========
    void sendJsonPacket(uint8_t topic_id, const std::string &json_str)
    {
        std::vector<uint8_t> packet;
        packet.push_back(0xAA);
        packet.push_back(topic_id);
        packet.insert(packet.end(), json_str.begin(), json_str.end());
        packet.push_back(0x0A);
        sendPacket(packet);
    }

    // ======== 实际发送 ========
    void sendPacket(const std::vector<uint8_t> &packet)
    {
        // 串口
        if (fd_ != -1)
        {
            ssize_t n = write(fd_, packet.data(), packet.size());
            if (n > 0) bytes_sent_serial_ += n;
        }

        // TCP 广播
        for (auto it = clients_.begin(); it != clients_.end();)
        {
            ssize_t n = send(*it, packet.data(), packet.size(), 0);
            if (n <= 0)
            {
                close(*it);
                it = clients_.erase(it);
            }
            else
            {
                bytes_sent_tcp_ += n;
                ++it;
            }
        }
    }

    // ======== 打印速率 ========
    void logRates()
    {
        RCLCPP_INFO(this->get_logger(),
                    "Tx rates (bytes/sec): serial=%zu, tcp=%zu, clients=%zu",
                    bytes_sent_serial_, bytes_sent_tcp_, clients_.size());
        bytes_sent_serial_ = bytes_sent_tcp_ = 0;
    }

private:
    std::string serial_port_;
    int baud_rate_;
    int fd_;

    std::string tcp_ip_;
    int tcp_port_;
    int listen_sock_;
    std::set<int> clients_;

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr rate_timer_, tcp_accept_timer_;

    size_t bytes_sent_serial_ = 0;
    size_t bytes_sent_tcp_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialSender>());
    rclcpp::shutdown();
    return 0;
}

