#include "mono-inertial-node.hpp"
#include "Tracking.h"
#include "utility.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

MonoInertialNode::MonoInertialNode(ORB_SLAM3::System *pSLAM)
    : Node("ORB_SLAM3_ROS2") {
  m_SLAM = pSLAM;
  // std::cout << "slam changed" << std::endl;
  m_image_subscriber = this->create_subscription<ImageMsg>(
      "camera", 10,
      std::bind(&MonoInertialNode::GrabImage, this, std::placeholders::_1));

  m_imu_subscriber = this->create_subscription<ImuMsg>(
      "imu", 10,
      std::bind(&MonoInertialNode::GrabImu, this, std::placeholders::_1));

  m_point_cloud_publisher =
      this->create_publisher<PointCloudMsg>("pointcloud", 10);
  std::cout << "slam changed" << std::endl;

  m_tracking_state_publisher =
      this->create_publisher<std_msgs::msg::Int32>("tracking_state", 10);

  syncThread_ = new std::thread(&MonoInertialNode::SyncWithImu, this);
}

MonoInertialNode::~MonoInertialNode() {
  // Stop all threads
  syncThread_->join();
  delete syncThread_;

  m_SLAM->Shutdown();

  // Save camera trajectory
  m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonoInertialNode::GrabImage(const ImageMsg::SharedPtr msg) {
  imgBufMutex_.lock();
  imgBuf_.push(msg);
  imgBufMutex_.unlock();
}

void MonoInertialNode::GrabImu(const ImuMsg::SharedPtr msg) {
  imuBufMutex_.lock();
  imuBuf_.push(msg);
  imuBufMutex_.unlock();
}

void MonoInertialNode::SyncWithImu() {
  while (true) {
    imgBufMutex_.lock();
    if (imgBuf_.empty()) {
      imgBufMutex_.unlock();
      continue;
    }

    ImageMsg::SharedPtr imgMsg = imgBuf_.front();
    // Copy the ros image message to cv::Mat.
    try {
      m_cvImPtr = cv_bridge::toCvCopy(imgMsg);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat img = m_cvImPtr->image.clone();
    auto imgTimeStamp = imgMsg->header.stamp;
    auto imgFrameId = imgMsg->header.frame_id;
    double tIm = Utility::StampToSec(imgTimeStamp);
    imgBuf_.pop();
    imgBufMutex_.unlock();

    // TODO: Use the size specified in the config file
    if (img.size[0] != 720 || img.size[1] != 960) {
      std::cout << "Wrong img size: " << img.size[0] << " " << img.size[1]
                << std::endl;
      continue;
    }

    std::cout << "one frame has been received" << std::endl;

    vector<ORB_SLAM3::IMU::Point> vImuMeas;

    imuBufMutex_.lock();
    std::cout << "imubuf has " << imuBuf_.size() << std::endl;
    while (!imuBuf_.empty() &&
           Utility::StampToSec(imuBuf_.front()->header.stamp) <= tIm) {
      double t = Utility::StampToSec(imuBuf_.front()->header.stamp);

      ImuMsg::SharedPtr imuMsg = imuBuf_.front();
      cv::Point3f acc(imuMsg->linear_acceleration.x,
                      imuMsg->linear_acceleration.y,
                      imuMsg->linear_acceleration.z);
      cv::Point3f gyr(imuMsg->angular_velocity.x, imuMsg->angular_velocity.y,
                      imuMsg->angular_velocity.z);

      vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
      std::cout << "IMU data received" << std::endl;
      imuBuf_.pop();
    }
    imuBufMutex_.unlock();

    if (vImuMeas.size() == 0) {
      continue;
    }

    std::cout << "Attempting Tracking" << std::endl;
    std::cout << "img: " << img.size() << std::endl;
    std::cout << "tIm: " << tIm << std::endl;
    std::cout << "vImuMeas: " << vImuMeas.size() << std::endl;

    m_SLAM->TrackMonocular(img, tIm, vImuMeas);

    std::cout << "Finished Tracking step" << std::endl;

    int tracking_state = m_SLAM->GetTrackingState();
    auto tracking_msg = std_msgs::msg::Int32();
    tracking_msg.data = tracking_state;
    m_tracking_state_publisher->publish(tracking_msg);

    if (tracking_state != ORB_SLAM3::Tracking::OK) {
      continue;
    }

    std::vector<ORB_SLAM3::MapPoint *> map_points =
        m_SLAM->GetTrackedMapPoints();

    if (map_points.empty()) {
      continue;
    }

    int num_points = static_cast<int>(map_points.size());
    std::cout << "There are " << num_points << " points\n";

    PointCloudMsg pcl_msg;
    pcl_msg.header = std_msgs::msg::Header();
    pcl_msg.header.stamp = imgTimeStamp;
    pcl_msg.header.frame_id = imgFrameId;

    pcl_msg.height = 1;
    pcl_msg.width = num_points;
    pcl_msg.is_dense = true;

    // There are six values below - x, y, z, norm_x e.t.c
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
    }

    m_point_cloud_publisher->publish(pcl_msg);
  }
}
