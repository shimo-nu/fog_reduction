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
//   sensor_msgs::msg::PointCloud2 filtered_cloud;
//   filter_intensity(input, filtered_cloud);
//   sensor_cloud_pub_->publish(filtered_cloud);

// }

// void FogReduction::filter_intensity(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input,
//                       sensor_msgs::msg::PointCloud2& output) {

  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT> pcl_cluster;
  pcl::fromROSMsg(*sensor_cloud_ptr, pcl_cluster);
  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr cloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>(pcl_cluster));
  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr reduction_cloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  RCLCPP_INFO(this->get_logger(), "reduce fog");
  for (const auto& point : cloud->points)
  {
    // RCLCPP_INFO(this->get_logger(), "x : %f", point.x);
    // RCLCPP_INFO(this->get_logger(), "y : %f", point.y);
    // RCLCPP_INFO(this->get_logger(), "z : %f", point.z);
    // RCLCPP_INFO(this->get_logger(), "intensity : %f", point.intensity);

    if (point.intensity > intensity_threshold) {
      // reduction_cloud にpointを追加
      reduction_cloud->points.push_back(point);
    }
  }
  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*reduction_cloud, output_msg);

  output_msg.header.stamp = sensor_cloud_ptr->header.stamp;
  output_msg.header.frame_id = sensor_cloud_ptr->header.frame_id;
  
  sensor_cloud_pub_->publish(output_msg);

  // int x_offset = input->fields[pcl::getFieldIndex(*input, "x")].offset;
  // int y_offset = input->fields[pcl::getFieldIndex(*input, "y")].offset;
  // int z_offset = input->fields[pcl::getFieldIndex(*input, "z")].offset;
  // (void)x_offset;
  // (void)y_offset;
  // (void)z_offset;
  // int intensity_offset = input->fields[pcl::getFieldIndex(*input, "intensity")].offset;

  // output.data.resize(input->data.size());
  // size_t output_size = 0;

  // double intensity_sum = 0;

  // for (size_t global_offset = 0; global_offset + input->point_step <= input->data.size();
  //      global_offset += input->point_step) {
  //   float intensity = *reinterpret_cast<const float *>(&input->data[global_offset + intensity_offset]);

  //   intensity_sum += intensity;
  //   if (intensity < intensity_threshold) {
  //     continue;
  //   }

  //   memcpy(&output.data[output_size], &input->data[global_offset], input->point_step);
  //   output_size += input->point_step;
  // }

  // RCLCPP_INFO(this->get_logger(), "Average Intensity : %f", intensity_sum);

  // output.data.resize(output_size);
  // output.header = input->header;
  // output.height = 1;
  // output.fields = input->fields;
  // output.is_bigendian = input->is_bigendian;
  // output.point_step = input->point_step;
  // output.is_dense = false; // NaN値を含む可能性があるため
  // output.width = static_cast<uint32_t>(output_size / input->point_step);
  // output.row_step = output.width * output.point_step;
}