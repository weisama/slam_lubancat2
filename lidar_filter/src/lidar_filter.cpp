#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <algorithm>

using std::placeholders::_1;

class LidarFilter : public rclcpp::Node
{
public:
  LidarFilter()
  : Node("lidar_filter")
  {
    // 订阅原始scan
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan_raw", 10, std::bind(&LidarFilter::scanCallback, this, _1));

    // 发布过滤后的scan
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    // 设置定时器，10Hz 发布最新一帧
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LidarFilter::publishFilteredScan, this));

    RCLCPP_INFO(this->get_logger(), "LidarFilter node started (10Hz).");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_scan_ = *msg;
  }

  void publishFilteredScan()
  {
    if (last_scan_.ranges.empty()) return;

    auto filtered = last_scan_;

    int n = static_cast<int>(filtered.ranges.size());
    if (n >= 445 && n <= 455)
    {
      const int target_size = 450;
      std::vector<float> new_ranges(target_size);
      std::vector<float> new_intensities(target_size);

      for (int i = 0; i < target_size; ++i)
      {
        float t = static_cast<float>(i) / (target_size - 1);
        float src_index = t * (n - 1);
        int i0 = static_cast<int>(src_index);
        int i1 = std::min(i0 + 1, n - 1);
        float alpha = src_index - i0;

        // 线性插值
        new_ranges[i] = filtered.ranges[i0] * (1 - alpha) + filtered.ranges[i1] * alpha;
        if (!filtered.intensities.empty())
          new_intensities[i] = filtered.intensities[i0] * (1 - alpha) + filtered.intensities[i1] * alpha;
      }

      filtered.ranges = new_ranges;
      filtered.intensities = new_intensities;

      // 更新角度信息
      float total_angle = filtered.angle_max - filtered.angle_min;
      filtered.angle_increment = total_angle / (target_size - 1);
    }
    else
    {
      // 点数不在范围内，跳过
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Received scan size=%d, out of range [345,355]", n);
      return;
    }

    pub_->publish(filtered);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::LaserScan last_scan_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFilter>());
  rclcpp::shutdown();
  return 0;
}
