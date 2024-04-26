#include "monocular-slam-node.hpp"
#include "Tracking.h"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System *pSLAM)
    : Node("ORB_SLAM3_ROS2") {
  m_SLAM = pSLAM;
  // std::cout << "slam changed" << std::endl;
  m_image_subscriber = this->create_subscription<ImageMsg>(
      "camera", 10,
      std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));

  m_point_cloud_publisher =
      this->create_publisher<PointCloudMsg>("pointcloud", 10);
  std::cout << "slam changed" << std::endl;

  m_tracking_state_publisher =
      this->create_publisher<std_msgs::msg::Int32>("tracking_state", 10);
}

MonocularSlamNode::~MonocularSlamNode() {
  // Stop all threads
  m_SLAM->Shutdown();

  // Save camera trajectory
  m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg) {
  // Copy the ros image message to cv::Mat.
  try {
    m_cvImPtr = cv_bridge::toCvCopy(msg);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  auto img = m_cvImPtr->image;
  std::cout << "one frame has been sent" << std::endl;
  m_SLAM->TrackMonocular(img, Utility::StampToSec(msg->header.stamp));

  int tracking_state = m_SLAM->GetTrackingState();
  auto tracking_msg = std_msgs::msg::Int32();
  tracking_msg.data = tracking_state;
  m_tracking_state_publisher->publish(tracking_msg);

  if (tracking_state != ORB_SLAM3::Tracking::OK) {
    return;
  }

  std::vector<ORB_SLAM3::MapPoint *> map_points = m_SLAM->GetTrackedMapPoints();

  if (map_points.empty()) {
    return;
  }

  int num_points = static_cast<int>(map_points.size());
  std::cout << "There are " << num_points << " points\n";

  PointCloudMsg pcl_msg;
  pcl_msg.header = std_msgs::msg::Header();
  pcl_msg.header.stamp = msg->header.stamp;
  pcl_msg.header.frame_id = msg->header.frame_id;

  pcl_msg.height = 1;
  pcl_msg.width = num_points;
  pcl_msg.is_dense = true;

  // There are six values below
  int n_fields = 6;
  // floats are 4 bytes
  pcl_msg.point_step = n_fields * 4;
  pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;

  sensor_msgs::PointCloud2Modifier mod(pcl_msg);
  mod.setPointCloud2Fields(n_fields, "x", 1,
                           sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                           sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                           sensor_msgs::msg::PointField::FLOAT32, "norm_x", 1,
                           sensor_msgs::msg::PointField::FLOAT32, "norm_y", 1,
                           sensor_msgs::msg::PointField::FLOAT32, "norm_z", 1,
                           sensor_msgs::msg::PointField::FLOAT32);
  mod.resize(pcl_msg.height * pcl_msg.width);

  sensor_msgs::PointCloud2Iterator<float> iter_x(pcl_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pcl_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pcl_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_norm_x(pcl_msg, "norm_x");
  sensor_msgs::PointCloud2Iterator<float> iter_norm_y(pcl_msg, "norm_y");
  sensor_msgs::PointCloud2Iterator<float> iter_norm_z(pcl_msg, "norm_z");

  for (int i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z,
           ++iter_norm_x, ++iter_norm_y, ++iter_norm_z, ++i) {
    if (!map_points[i]) {
      std::cout << "Empty map point!\n";
      continue;
    }

    Eigen::Vector3f point = map_points[i]->GetWorldPos();
    Eigen::Vector3f norm = map_points[i]->GetNormal();

    *iter_x = point(0);
    *iter_y = point(1);
    *iter_z = point(2);
    *iter_norm_x = norm(0);
    *iter_norm_y = norm(1);
    *iter_norm_z = norm(2);
    *iter_x = 0;

    std::cout << i << "\n";
  }

  m_point_cloud_publisher->publish(pcl_msg);
}
