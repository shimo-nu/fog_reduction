#ifndef FOG_REDUCTION__FOG_REDUCTION_CORE_HPP_
#define FOG_REDUCTION__FOG_REDUCTION_CORE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/header.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/transform_datatypes.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <array>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

struct Point {
  double x, y, z;
};

class FogReduction : public rclcpp::Node
{
  public:
    FogReduction();
    // ~FogReduction();

  private:
    void callback_sensor_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_cloud_ptr);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_cloud_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    int intensity_threshold;
    int density_threshold;
    float density_radius;
};


#endif 