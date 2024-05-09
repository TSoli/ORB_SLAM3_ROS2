#include "rgbd-inertial-slam-node.hpp"
#include "sophus/se3.hpp"

#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <cmath>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>
#include <opencv2/core/core.hpp>

using std::placeholders::_1;

RgbdInertialNode::RgbdInertialNode(ORB_SLAM3::System *pSLAM)
    : Node("ORB_SLAM3_ROS2"), m_SLAM(pSLAM) {
  // rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg>
  // >(shared_ptr<rclcpp::Node>(this), "camera/rgb"); depth_sub =
  // std::make_shared<message_filters::Subscriber<ImageMsg>
  // >(shared_ptr<rclcpp::Node>(this), "camera/depth"); according to
  // https://github.com/zang09/ORB_SLAM3_ROS2/issues/24
  rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(
      this, "/camera/image");
  depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(
      this, "/camera/depth");
  m_imu_subscriber = this->create_subscription<ImuMsg>(
      "imu_resampled", 10,
      std::bind(&RgbdInertialNode::GrabImu, this, std::placeholders::_1));

  m_tracking_state_publisher =
      this->create_publisher<std_msgs::msg::Int32>("tracking_state", 10);
  pose_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  syncApproximate =
      std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(
          approximate_sync_policy(10), *rgb_sub, *depth_sub);
  syncApproximate->registerCallback(&RgbdInertialNode::GrabRGBD, this);

  Eigen::Vector3f T(0.0, 0.0, 0.0);
  Eigen::Matrix3f R =
      Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitX()).toRotationMatrix();
  pose_transform = Sophus::SE3f(R, T);
}

RgbdInertialNode::~RgbdInertialNode() {
  // Stop all threads
  m_SLAM->Shutdown();

  // Save camera trajectory
  m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdInertialNode::GrabImu(const ImuMsg::SharedPtr msg) {
  imuBufMutex_.lock();
  imuBuf_.push(msg);
  imuBufMutex_.unlock();
}

void RgbdInertialNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB,
                            const ImageMsg::SharedPtr msgD) {
  // Copy the ros rgb image message to cv::Mat.
  try {
    cv_ptrRGB = cv_bridge::toCvCopy(msgRGB, "bgr8");
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Copy the ros depth image message to cv::Mat.
  try {
    cv_ptrD = cv_bridge::toCvCopy(msgD);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  const Sophus::SE3f pose =
      pose_transform *
      m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image,
                        Utility::StampToSec(msgRGB->header.stamp));

  int tracking_state = m_SLAM->GetTrackingState();
  auto tracking_msg = std_msgs::msg::Int32();
  tracking_msg.data = tracking_state;
  m_tracking_state_publisher->publish(tracking_msg);

  if (tracking_state != ORB_SLAM3::Tracking::OK) {
    return;
  }

  const Eigen::Quaternionf q = pose.so3().unit_quaternion();
  const Sophus::Vector3f t = pose.translation();

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = msgD->header.stamp;
  tf.header.frame_id = "odom_vslam";
  tf.child_frame_id = msgD->header.frame_id;

  tf.transform.translation.x = t.x();
  tf.transform.translation.y = t.y();
  tf.transform.translation.z = t.z();

  tf.transform.rotation.w = q.w();
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();

  pose_broadcaster->sendTransform(tf);
}
