#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/int32.hpp"

#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <vector>

#include "Frame.h"
#include "Map.h"
#include "MapPoint.h"
#include "System.h"
#include "Tracking.h"

#include "utility.hpp"

class MonocularSlamNode : public rclcpp::Node {
public:
  MonocularSlamNode(ORB_SLAM3::System *pSLAM);

  ~MonocularSlamNode();

private:
  using ImageMsg = sensor_msgs::msg::Image;
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

  ORB_SLAM3::System *m_SLAM;

  cv_bridge::CvImagePtr m_cvImPtr;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;

  rclcpp::Publisher<PointCloudMsg>::SharedPtr m_point_cloud_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_tracking_state_publisher;
};

#endif
