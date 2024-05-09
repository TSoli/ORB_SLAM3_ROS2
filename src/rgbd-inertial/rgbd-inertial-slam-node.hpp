#ifndef __RGBD_INERTIAL_SLAM_NODE_HPP__
#define __RGBD_INERTIAL_SLAM_NODE_HPP__

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include <cv_bridge/cv_bridge.h>
#include <memory>

#include "Frame.h"
#include "Map.h"
#include "System.h"
#include "Tracking.h"

#include "utility.hpp"

class RgbdInertialNode : public rclcpp::Node {
public:
  RgbdInertialNode(ORB_SLAM3::System *pSLAM);

  ~RgbdInertialNode();

private:
  using ImageMsg = sensor_msgs::msg::Image;
  using ImuMsg = sensor_msgs::msg::Imu;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>
      approximate_sync_policy;

  void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB,
                const sensor_msgs::msg::Image::SharedPtr msgD);
  void GrabImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  queue<ImuMsg::SharedPtr> imuBuf_;
  std::mutex imuBufMutex_;
  queue<ImageMsg::SharedPtr> rgbBuf_;
  std::mutex rgbBufMutex_;
  queue<ImageMsg::SharedPtr> depthBuf;
  std::mutex depthBufMutex_;

  ORB_SLAM3::System *m_SLAM;

  cv_bridge::CvImageConstPtr cv_ptrRGB;
  cv_bridge::CvImageConstPtr cv_ptrD;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>
      depth_sub;
  rclcpp::Subscription<ImuMsg>::SharedPtr m_imu_subscriber;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_tracking_state_publisher;

  std::unique_ptr<tf2_ros::TransformBroadcaster> pose_broadcaster;

  std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>>
      syncApproximate;

  Sophus::SE3f pose_transform; // Makes the visualisation in nvblox nicer
};

#endif
