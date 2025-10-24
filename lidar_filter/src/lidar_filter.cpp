#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <algorithm>
#include <string>
#include <cmath>

using std::placeholders::_1;

class LidarFilter : public rclcpp::Node
{
public:
  LidarFilter()
  : Node("lidar_filter")
  {
    // 参数：目标点数和输入话题
    this->declare_parameter<int>("target_size", 450);
    this->declare_parameter<std::string>("scan_topic", "/scan_raw");

    this->get_parameter("target_size", target_size_);
    this->get_parameter("scan_topic", scan_topic_);

    RCLCPP_INFO(this->get_logger(), "LidarFilter started: target_size=%d, scan_topic=%s", 
                target_size_, scan_topic_.c_str());

    // 发布过滤后的scan
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    // 订阅原始scan，并在回调函数中立即处理并发布
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10, std::bind(&LidarFilter::scanCallback, this, _1));
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // 立即处理并发布过滤后的数据
    auto filtered_scan = processScan(*msg);
    pub_->publish(filtered_scan);
  }

  sensor_msgs::msg::LaserScan processScan(const sensor_msgs::msg::LaserScan& input_scan)
  {
    auto output_scan = input_scan;
    size_t original_size = input_scan.ranges.size();
    
    // 如果目标尺寸与原始尺寸相同，直接返回
    if (static_cast<size_t>(target_size_) == original_size) {
      return output_scan;
    }

    // 准备输出数组
    std::vector<float> new_ranges(target_size_, std::numeric_limits<float>::infinity());
    std::vector<float> new_intensities;
    if (!input_scan.intensities.empty()) {
      new_intensities.resize(target_size_, 0.0);
    }

    // 计算角度参数
    float original_angle_min = input_scan.angle_min;
    float original_angle_increment = input_scan.angle_increment;
    float output_angle_increment = (input_scan.angle_max - input_scan.angle_min) / (target_size_ - 1);
    
    output_scan.angle_increment = output_angle_increment;
    output_scan.ranges = new_ranges;
    output_scan.intensities = new_intensities;

    // 为每个输出点找到对应的原始点
    for (int out_idx = 0; out_idx < target_size_; ++out_idx) {
      float target_angle = original_angle_min + out_idx * output_angle_increment;
      
      // 找到最近的原始点索引
      int closest_idx = findClosestIndex(target_angle, original_angle_min, original_angle_increment, original_size);
      
      if (closest_idx >= 0 && static_cast<size_t>(closest_idx) < original_size) {
        // 使用最近邻插值
        output_scan.ranges[out_idx] = input_scan.ranges[closest_idx];
        if (!input_scan.intensities.empty() && !output_scan.intensities.empty()) {
          output_scan.intensities[out_idx] = input_scan.intensities[closest_idx];
        }
      }
    }

    return output_scan;
  }

  int findClosestIndex(float target_angle, float angle_min, float angle_increment, size_t size)
  {
    // 计算目标角度对应的索引
    float raw_index = (target_angle - angle_min) / angle_increment;
    
    // 四舍五入到最近的整数索引
    int index = static_cast<int>(std::round(raw_index));
    
    // 确保索引在有效范围内
    if (index < 0) return 0;
    if (static_cast<size_t>(index) >= size) return static_cast<int>(size) - 1;
    
    return index;
  }

  bool isValidMeasurement(float range)
  {
    return std::isfinite(range) && range >= 0.0f;
  }

  std::string scan_topic_;
  int target_size_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFilter>());
  rclcpp::shutdown();
  return 0;
}