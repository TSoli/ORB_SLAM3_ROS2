#include "rgbd-slam-node.hpp"
#include "sophus/se3.hpp"

#include <Eigen/src/Geometry/Quaternion.h>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>
#include <opencv2/core/core.hpp>

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System *pSLAM)
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

  pose_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  syncApproximate =
      std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(
          approximate_sync_policy(10), *rgb_sub, *depth_sub);
  syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);
}

RgbdSlamNode::~RgbdSlamNode() {
  // Stop all threads
  m_SLAM->Shutdown();

  // Save camera trajectory
  m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB,
                            const ImageMsg::SharedPtr msgD) {
  // Copy the ros rgb image message to cv::Mat.
  try {
    cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Copy the ros depth image message to cv::Mat.
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  const Sophus::SE3f pose =
      m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image,
                        Utility::StampToSec(msgRGB->header.stamp));
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
