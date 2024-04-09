#ifndef __MONO_INERTIAL_NODE__
#define __MONO_INERTIAL_NODE__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
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

class MonoInertialNode : public rclcpp::Node {
public:
  MonoInertialNode(ORB_SLAM3::System *pSLAM);

  ~MonoInertialNode();

private:
  using ImageMsg = sensor_msgs::msg::Image;
  using ImuMsg = sensor_msgs::msg::Imu;
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
  void GrabImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void SyncWithImu();
  queue<ImuMsg::SharedPtr> imuBuf_;
  std::mutex imuBufMutex_;

  std::thread *syncThread_;

  // Image
  queue<ImageMsg::SharedPtr> imgBuf_;
  std::mutex imgBufMutex_;

  ORB_SLAM3::System *m_SLAM;

  cv_bridge::CvImagePtr m_cvImPtr;

  rclcpp::Subscription<ImageMsg>::SharedPtr m_image_subscriber;
  rclcpp::Subscription<ImuMsg>::SharedPtr m_imu_subscriber;

  rclcpp::Publisher<PointCloudMsg>::SharedPtr m_point_cloud_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_tracking_state_publisher;
};

#endif
