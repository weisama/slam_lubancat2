#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <set>
#include <mutex>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <cerrno>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <nlohmann/json.hpp>
#include <tinyxml2.h>
#include <filesystem>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;
using namespace tinyxml2;
namespace fs = std::filesystem;

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

            bytes_recv_tcp_ += n;

            std::ostringstream oss;
            oss << "Recv (" << n << " bytes): ";
            for (ssize_t i = 0; i < n; ++i)
                oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                    << static_cast<int>(buf[i]) << " ";
            RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

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

            // === 保存地图命令: AA 00 02 0A ===
            for (ssize_t i = 0; i < n - 3; ++i)
            {
                if (buf[i] == 0xAA && buf[i + 1] == 0x00 &&
                    buf[i + 2] == 0x02 && buf[i + 3] == 0x0A)
                {
                    handleMapSaveCommand();
                    RCLCPP_INFO(this->get_logger(), "Recv save map command AA 00 02 0A");
                }
            }

            // === 发送参数命令: AA 00 03 0A ===
            for (ssize_t i = 0; i < n - 3; ++i)
            {
                if (buf[i] == 0xAA && buf[i + 1] == 0x00 &&
                    buf[i + 2] == 0x03 && buf[i + 3] == 0x0A)
                {
                    handleSendParamsCommand();
                    RCLCPP_INFO(this->get_logger(), "Recv send params command AA 00 03 0A");
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

    // ======== 发送参数到上位机 ========
    void handleSendParamsCommand()
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

        auto getVal = [&](const char *name, const std::string &default_val = "0") -> std::string
        {
            XMLElement *elem = root->FirstChildElement(name);
            return elem ? (elem->GetText() ? elem->GetText() : default_val) : default_val;
        };

        // 读取所有参数值
        std::string lidar_name = getVal("lidar_name", "N10");
        float x = std::stof(getVal("x", "0"));
        float y = std::stof(getVal("y", "0"));
        float z = std::stof(getVal("z", "0"));
        float roll = std::stof(getVal("roll", "0"));
        float pitch = std::stof(getVal("pitch", "0"));
        float yaw = std::stof(getVal("yaw", "0"));
        std::string mode = getVal("mode", "mapping");

        // 发送所有参数到客户端
        sendParamToClients(0x00, (lidar_name == "N10") ? 0 : 1);  // 雷达类型
        sendParamToClients(0x01, static_cast<uint16_t>(x));        // X坐标
        sendParamToClients(0x02, static_cast<uint16_t>(y));        // Y坐标  
        sendParamToClients(0x03, static_cast<uint16_t>(z));        // Z坐标
        sendParamToClients(0x04, static_cast<uint16_t>(roll));     // 横滚角
        sendParamToClients(0x05, static_cast<uint16_t>(pitch));    // 俯仰角
        sendParamToClients(0x06, static_cast<uint16_t>(yaw));      // 偏航角
        sendParamToClients(0x10, (mode == "mapping") ? 0 : 1);     // 工作模式

        RCLCPP_INFO(this->get_logger(), "All parameters sent to clients");
    }

    void sendParamToClients(uint8_t param_id, uint16_t value)
    {
        std::vector<uint8_t> packet;
        packet.push_back(0xAA);
        packet.push_back(0x10);  // 参数命令标识
        packet.push_back(param_id);
        packet.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));  // 高字节
        packet.push_back(static_cast<uint8_t>(value & 0xFF));         // 低字节
        packet.push_back(0x0A);  // 结束符

        sendPacket(packet);
        
        RCLCPP_DEBUG(this->get_logger(), "Sent param: id=0x%02X value=%d", param_id, value);
    }

    // ======== 保存地图为YAML和PGM格式 ========
    void handleMapSaveCommand()
    {
        if (!last_map_)
        {
            RCLCPP_WARN(this->get_logger(), "No map data received yet!");
            return;
        }

        std::string dir_path = std::string(getenv("HOME")) + "/slam_ws/src/upper/map/";
        if (!fs::exists(dir_path))
            fs::create_directories(dir_path);

        std::string yaml_path = dir_path + "my_map.yaml";
        std::string pgm_path = dir_path + "my_map.pgm";

        // 保存PGM文件
        if (!saveMapAsPGM(pgm_path))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save PGM map");
            return;
        }

        // 保存YAML文件
        if (!saveMapAsYAML(yaml_path, pgm_path))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save YAML map");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Map saved to %s and %s", yaml_path.c_str(), pgm_path.c_str());
    }

    bool saveMapAsPGM(const std::string& file_path)
    {
        std::ofstream ofs(file_path, std::ios::binary);
        if (!ofs.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open PGM file: %s", file_path.c_str());
            return false;
        }

        const auto& map = last_map_;
        int width = map->info.width;
        int height = map->info.height;

        // 写入PGM头
        ofs << "P5\n" << width << " " << height << "\n255\n";

        // 写入地图数据（转换为0-255范围）
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                int index = y * width + x;
                int8_t value = map->data[index];
                
                // 转换规则：
                // -1 (未知) -> 205
                // 0-100 (占用) -> 0-254 (0=完全占用, 100=完全自由)
                uint8_t pgm_value;
                if (value == -1)
                {
                    pgm_value = 205;  // 未知区域
                }
                else if (value < 0 || value > 100)
                {
                    pgm_value = 205;  // 无效值也当作未知
                }
                else
                {
                    // 将0-100转换为0-254，0表示占用，100表示自由
                    pgm_value = static_cast<uint8_t>((100 - value) * 254 / 100);
                }
                
                ofs << pgm_value;
            }
        }

        ofs.close();
        return true;
    }

    bool saveMapAsYAML(const std::string& yaml_path, const std::string& pgm_path)
    {
        std::ofstream ofs(yaml_path);
        if (!ofs.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file: %s", yaml_path.c_str());
            return false;
        }

        const auto& map = last_map_;
        
        // 只获取PGM文件名，不包含路径
        std::string pgm_filename = "my_map.pgm";

        // 写入YAML内容
        ofs << "image: " << pgm_filename << "\n"
            << "resolution: " << map->info.resolution << "\n"
            << "origin: [" << map->info.origin.position.x << ", " 
                          << map->info.origin.position.y << ", " 
                          << map->info.origin.position.z << "]\n"
            << "negate: 0\n"
            << "occupied_thresh: 0.65\n"
            << "free_thresh: 0.196\n";

        ofs.close();
        return true;
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
            case 0x00: if (value == 0) setVal("lidar_name", "N10"); else setVal("lidar_name", "N10_P"); break;
            case 0x01: setVal("x", std::to_string(value)); break;
            case 0x02: setVal("y", std::to_string(value)); break;
            case 0x03: setVal("z", std::to_string(value)); break;
            case 0x04: setVal("roll", std::to_string(value)); break;
            case 0x05: setVal("pitch", std::to_string(value)); break;
            case 0x06: setVal("yaw", std::to_string(value)); break;
            case 0x10:
                if (value == 0) setVal("mode", "mapping");
                else if (value == 1) setVal("mode", "localization");
                else RCLCPP_WARN(this->get_logger(), "Unknown mode value: %d", value);
                break;
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

        last_map_ = msg; // 保存最新地图

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

    // ======== 日志统计 ========
    void logRates()
    {
        RCLCPP_INFO(this->get_logger(),
                    "Tx/Rx rates (bytes/sec): sent=%zu, recv=%zu, clients=%zu",
                    bytes_sent_tcp_, bytes_recv_tcp_, clients_.size());
        bytes_sent_tcp_ = 0;
        bytes_recv_tcp_ = 0;
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
    size_t bytes_recv_tcp_ = 0;

    nav_msgs::msg::OccupancyGrid::SharedPtr last_map_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TcpSender>());
    rclcpp::shutdown();
    return 0;
}
