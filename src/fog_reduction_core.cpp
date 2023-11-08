#include "fog_reduction/fog_reduction_core.hpp"

#include <iomanip>
#include <sstream>


FogReduction::FogReduction()
: Node("fog_reduction"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // objects_feature_sub_ = thisx->create_subscription<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
  //   "objects_with_feature", 10,
  //   std::bind(&FogReduction::callback_objects_with_feature, this, std::placeholders::_1)
  // );

  this->declare_parameter<int>("intensity_threshold", 5);
  this->declare_parameter<int>("density_threshold", 300);
  this->declare_parameter<float>("density_radius", 0.1);


  this->get_parameter("intensity_threshold", intensity_threshold);
  this->get_parameter("density_threshold", density_threshold);
  this->get_parameter("density_radius", density_radius);

  sensor_cloud_sub_ = 
    this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", rclcpp::SensorDataQoS(),
      std::bind(&FogReduction::callback_sensor_cloud, this, std::placeholders::_1)
    );

  sensor_cloud_pub_ = 
    this->create_publisher<sensor_msgs::msg::PointCloud2>("output", 10);
}

// void FogReduction::callback_objects_with_feature(tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr objects_feature_ptr)
// {
//   // for (const auto & object_feature)
//   return 
// }
void FogReduction::callback_sensor_cloud
(sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_cloud_ptr)
{
  pcl::PointCloud<pcl::PointXYZI> pcl_cluster;
  pcl::fromROSMsg(*sensor_cloud_ptr, pcl_cluster);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>(pcl_cluster));
  pcl::PointCloud<pcl::PointXYZI>::Ptr reduction_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  RCLCPP_INFO(this->get_logger(), "reduce fog");
  for (const auto& point : cloud->points)
  {
    // RCLCPP_INFO(this->get_logger(), "x : %f", point.x);
    // RCLCPP_INFO(this->get_logger(), "y : %f", point.y);
    // RCLCPP_INFO(this->get_logger(), "z : %f", point.z);
    // RCLCPP_INFO(this->get_logger(), "intensity : %f", point.intensity);

    if (point.intensity > intensity_threshold) {
      // reduction_cloud にpointを追加
      reduction_cloud->push_back(point); 
    }
  }
  RCLCPP_INFO(this->get_logger(), "publish pointcloud");
  // PointCloud2 にデータを変換
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*reduction_cloud, output);
  output.header.stamp = sensor_cloud_ptr->header.stamp;
  output.header.frame_id = sensor_cloud_ptr->header.frame_id;

  sensor_cloud_pub_->publish(output);
}
