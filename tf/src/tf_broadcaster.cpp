#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include <tf2/LinearMath/Quaternion.h>

class StaticTFBroadcaster : public rclcpp::Node
{
public:
    StaticTFBroadcaster()
    : Node("tf_broadcaster")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        // 发布静态变换
        publish_transforms();
    }

private:
    void publish_transforms()
    {
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        
        // map -> odom 变换
        auto map_to_odom = create_transform("map", "odom", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        transforms.push_back(map_to_odom);
        
        // odom -> base_link 变换
        auto odom_to_base = create_transform("odom", "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        transforms.push_back(odom_to_base);
        
        // base_link -> laser_link 变换
        auto base_to_laser = create_transform("base_link", "laser_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        transforms.push_back(base_to_laser);
        
        // 发布所有变换
        tf_broadcaster_->sendTransform(transforms);
        
        RCLCPP_INFO(this->get_logger(), "Published static transforms");
    }
    
    geometry_msgs::msg::TransformStamped create_transform(
        const std::string& parent_frame,
        const std::string& child_frame,
        double x, double y, double z,
        double roll, double pitch, double yaw)
    {
        geometry_msgs::msg::TransformStamped transform;
        
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = parent_frame;
        transform.child_frame_id = child_frame;
        
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = z;
        
        // 将欧拉角转换为四元数
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        return transform;
    }
    
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticTFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
