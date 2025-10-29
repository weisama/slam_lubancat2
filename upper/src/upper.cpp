#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <set>
#include <mutex>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <cerrno>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <nlohmann/json.hpp>
#include <tinyxml2.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;
using namespace tinyxml2;

class TcpSender : public rclcpp::Node
{
public:
    TcpSender() : Node("tcp_sender"), listen_sock_(-1)
    {
        // ---- 参数声明 ----
        this->declare_parameter<std::string>("tcp_ip", "0.0.0.0");
        this->declare_parameter<int>("tcp_port", 6666);

        tcp_ip_ = this->get_parameter("tcp_ip").as_string();
        tcp_port_ = this->get_parameter("tcp_port").as_int();

        startTcpServer();

        // ---- 订阅话题 ----
        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10, std::bind(&TcpSender::tfCallback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&TcpSender::scanCallback, this, std::placeholders::_1));
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 1, std::bind(&TcpSender::mapCallback, this, std::placeholders::_1));

        // ---- TCP 连接 & 接收检查 ----
        tcp_accept_timer_ = this->create_wall_timer(200ms, std::bind(&TcpSender::acceptPendingClients, this));
        tcp_recv_timer_ = this->create_wall_timer(100ms, std::bind(&TcpSender::checkTcpRecv, this));

        // ---- 速率统计 ----
        rate_timer_ = this->create_wall_timer(1s, std::bind(&TcpSender::logRates, this));
    }

    ~TcpSender()
    {
        if (listen_sock_ != -1) close(listen_sock_);
        for (int c : clients_) close(c);
    }

private:
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

    // ======== TCP 数据接收与解析 ========
    void checkTcpRecv()
    {
        uint8_t buf[256];
        for (auto it = clients_.begin(); it != clients_.end();)
        {
            ssize_t n = recv(*it, buf, sizeof(buf), 0);
            if (n <= 0)
            {
                ++it;
                continue;
            }

            // === 通信测试命令: AA 00 01 0A ===
            for (ssize_t i = 0; i < n - 3; ++i)
            {
                if (buf[i] == 0xAA && buf[i + 1] == 0x00 &&
                    buf[i + 2] == 0x01 && buf[i + 3] == 0x0A)
                {
                    uint8_t reply[4] = {0xAA, 0x00, 0x01, 0x0A};
                    send(*it, reply, sizeof(reply), 0);
                    RCLCPP_INFO(this->get_logger(), "Recv test cmd, echo back AA 00 01 0A");
                }
            }

            // === 参数修改命令: AA 10 param_id val_high val_low 0A ===
            for (ssize_t i = 0; i < n - 5; ++i)
            {
                if (buf[i] == 0xAA && buf[i + 1] == 0x10 && buf[i + 5] == 0x0A)
                {
                    uint8_t param_id = buf[i + 2];
                    uint16_t value = (buf[i + 3] << 8) | buf[i + 4];
                    handleParamCommand(param_id, value);
                    RCLCPP_INFO(this->get_logger(), "Recv param cmd: id=%d value=%d", param_id, value);
                }
            }

            ++it;
        }
    }

    // ======== 修改 param.xml ========
    void handleParamCommand(uint8_t param_id, uint16_t value)
    {
        std::string file_path = std::string(getenv("HOME")) + "/slam_ws/src/upper/config/param.xml";
        XMLDocument doc;
        if (doc.LoadFile(file_path.c_str()) != XML_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", file_path.c_str());
            return;
        }

        XMLElement *root = doc.RootElement();
        if (!root)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid XML root");
            return;
        }

        auto setVal = [&](const char *name, const std::string &val)
        {
            XMLElement *elem = root->FirstChildElement(name);
            if (elem)
                elem->SetText(val.c_str());
        };

        switch (param_id)
        {
            case 0x00: // 雷达型号
                if (value == 0) setVal("lidar_name", "N10");
                else setVal("lidar_name", "N10_P");
                break;
            case 0x01: setVal("x", std::to_string(value)); break;
            case 0x02: setVal("y", std::to_string(value)); break;
            case 0x03: setVal("z", std::to_string(value)); break;
            case 0x04: setVal("roll", std::to_string(value)); break;
            case 0x05: setVal("pitch", std::to_string(value)); break;
            case 0x06: setVal("yaw", std::to_string(value)); break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown param id: %d", param_id);
                return;
        }

        doc.SaveFile(file_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Updated %s successfully.", file_path.c_str());
    }

    // ======== ROS2 回调 ========
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
        j["ranges"] = msg->ranges;
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

    void logRates()
    {
        RCLCPP_INFO(this->get_logger(),
                    "Tx rates (bytes/sec): tcp=%zu, clients=%zu",
                    bytes_sent_tcp_, clients_.size());
        bytes_sent_tcp_ = 0;
    }

private:
    std::string tcp_ip_;
    int tcp_port_;
    int listen_sock_;
    std::set<int> clients_;

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr rate_timer_, tcp_accept_timer_, tcp_recv_timer_;

    size_t bytes_sent_tcp_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TcpSender>());
    rclcpp::shutdown();
    return 0;
}

