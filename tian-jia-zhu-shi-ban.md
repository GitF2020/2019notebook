# 添加注释版

```cpp
/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>

#include <sawyer_arm_kinect_calibration/detect_calibration_pattern.h>

using namespace std;
using namespace Eigen;

tf::Transform tfFromEigen(Eigen::Matrix4f trans)
{
  tf::Matrix3x3 btm;
  btm.setValue(trans(0, 0), trans(0, 1), trans(0, 2),
               trans(1, 0), trans(1, 1), trans(1, 2),
               trans(2, 0), trans(2, 1), trans(2, 2));
  tf::Transform ret;
  ret.setOrigin(tf::Vector3(trans(0, 3), trans(1, 3), trans(2, 3))); //通过调用类transform的成员函数设置坐标原点
  ret.setBasis(btm);//设定基准坐标系
  return ret;
}

Eigen::Matrix4f EigenFromTF(tf::Transform trans)
{
  Eigen::Matrix4f out;
  tf::Quaternion quat = trans.getRotation();  //通过旋转矩阵求四元数
  tf::Vector3 origin = trans.getOrigin();  //相对坐标原点，即xyz平移的偏移量

  Eigen::Quaternionf quat_out(quat.w(), quat.x(), quat.y(), quat.z());
  Eigen::Vector3f origin_out(origin.x(), origin.y(), origin.z());

  out.topLeftCorner<3, 3>() = quat_out.toRotationMatrix();//转成旋转矩阵形式
  out.topRightCorner<3, 1>() = origin_out;
  out(3, 3) = 1;

  return out;
}

class CalibrateKinectCheckerboard
{
  // Nodes and publishers/subscribers
  ros::NodeHandle nh_;                           //创建一份节点句柄，方便对节点资源的使用和管理
  image_transport::ImageTransport it_;
  image_transport::Publisher pub_;
  ros::Publisher detector_pub_;               //发布了两个节点，探测
  ros::Publisher physical_pub_;               //物理的   

  // Image and camera info subscribers;         
  ros::Subscriber image_sub_;					//订阅
  ros::Subscriber info_sub_;

  // Structures for interacting with ROS messages  和ros消息互动的结构
  cv_bridge::CvImagePtr input_bridge_;
  cv_bridge::CvImagePtr output_bridge_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  image_geometry::PinholeCameraModel cam_model_;

  // Calibration objects 校准对象
  PatternDetector pattern_detector_;

  // The optimized transform  优化的转换
  Eigen::Transform<float, 3, Eigen::Affine> transform_;//4*4齐次矩阵变换

  // Visualization for markers  标记可视化 调用了pcl库
  pcl::PointCloud<pcl::PointXYZ> detector_points_;
  pcl::PointCloud<pcl::PointXYZ> ideal_points_;
  pcl::PointCloud<pcl::PointXYZ> image_points_;
  pcl::PointCloud<pcl::PointXYZ> physical_points_;  //机械臂点云

  // Have we calibrated the camera yet?
  bool calibrated;


  ros::Timer timer_;

  // Parameters 因素
  std::string fixed_frame;  //tf config中配置的架构的名称
  std::string camera_frame;
  std::string target_frame;
  std::string tip_frame;
  std::string touch_frame;

  int checkerboard_width; //声明棋盘用的3个变量
  int checkerboard_height;
  double checkerboard_grid;//棋盘网格

  // Gripper tip position
  geometry_msgs::PointStamped gripper_tip;




public:
  CalibrateKinectCheckerboard() :
      nh_("~"), it_(nh_), calibrated(false)
  {
    // Load parameters from the server.
    nh_.param<std::string>("fixed_frame", fixed_frame, "/base");                           //机械臂坐标系
    nh_.param<std::string>("camera_frame", camera_frame, "/kinect2_rgb_optical_frame");    //相机坐标系
    nh_.param<std::string>("target_frame", target_frame, "/calibration_pattern");			//标定板坐标系
    nh_.param<std::string>("tip_frame", tip_frame, "/right_gripper_l_finger_tip");           //机械臂夹子坐标系

    nh_.param<int>("checkerboard_width", checkerboard_width, 7); //声明棋盘用的3个参数
    nh_.param<int>("checkerboard_height", checkerboard_height, 5);//7和5为格子的个数，6，5是角点的数目
    nh_.param<double>("checkerboard_grid", checkerboard_grid, 0.03);//识别棋盘长8格，宽6格，单个边长为3cm即0.03m

    // Set pattern detector sizes 设置图案检测的大小
    pattern_detector_.setPattern(cv::Size(checkerboard_width, checkerboard_height),//调用了opencv的函数处理
                                 checkerboard_grid, CHESSBOARD);

    transform_.translation().setZero();
    transform_.matrix().topLeftCorner<3, 3>() = Quaternionf().setIdentity().toRotationMatrix();  //矩阵初始化

    // Create subscriptions 创建订阅
    info_sub_ = nh_.subscribe("/kinect2/hd/camera_info", 1, &CalibrateKinectCheckerboard::infoCallback, this);//订阅Kinect2内参信息/kinect2/hd/camera_info

    // Also publishers
    pub_ = it_.advertise("calibration_pattern_out", 1);  //订阅话题和发布话题，只订阅一次
    detector_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("detector_cloud", 1);
    physical_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("physical_points_cloud", 1);

    // Create ideal points 理想点？？？
    ideal_points_.push_back(pcl::PointXYZ(0, 0, 0));
    ideal_points_.push_back(pcl::PointXYZ((checkerboard_width - 1) * checkerboard_grid, 0, 0)); //6*0.03        6个角点
    ideal_points_.push_back(pcl::PointXYZ(0, (checkerboard_height - 1) * checkerboard_grid, 0)); //4*0.03       4个角点
    ideal_points_.push_back(pcl::PointXYZ((checkerboard_width - 1) * checkerboard_grid,
                                          (checkerboard_height - 1) * checkerboard_grid, 0)); //

    // Create proper gripper tip point 创建合适的夹爪尖端
    nh_.param<double>("gripper_tip_x", gripper_tip.point.x, 0.0);
    nh_.param<double>("gripper_tip_y", gripper_tip.point.y, 0.0);
    nh_.param<double>("gripper_tip_z", gripper_tip.point.z, 0.0);
    gripper_tip.header.frame_id = tip_frame;

    ROS_INFO("[calibrate] Initialized.");//ROS_INFO() 函数将接受到的信息打印出来

    ROS_INFO("[calibrate] Succeed to init Gripper_tip Info:");
    cout << "frame_id:" << gripper_tip.header.frame_id << endl;
    cout << "x:" << gripper_tip.point.x << endl;
    cout << "y:" << gripper_tip.point.y << endl;
    cout << "z:" << gripper_tip.point.z << endl << endl;
  }

  void infoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)//信息回调函数？
  {
    if (calibrated)
      return;
    cam_model_.fromCameraInfo(info_msg);
    pattern_detector_.setCameraMatrices(cam_model_.intrinsicMatrix(), cam_model_.distortionCoeffs());//相机内参，内部矩阵 ，[k1，k2，p1，p2，k3]
    ROS_INFO("[calibrate] Camera IntrinsicMatrix: ");
    cout << endl << cam_model_.intrinsicMatrix() << endl << endl;
    ROS_INFO("[calibrate] Camera DistortionCoeffs: ");
    cout << endl << cam_model_.distortionCoeffs() << endl << endl;

    calibrated = true;
    image_sub_ = nh_.subscribe("/kinect2/hd/image_color_rect", 1, &CalibrateKinectCheckerboard::imageCallback, this);

    ROS_INFO("[calibrate] Got image info!");
  }

  void pointcloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)//点云回调？
  {
    sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(*msg, cloud);
    pcl::toROSMsg(cloud, *image_msg);

    imageCallback(image_msg);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg)//图像回调
  {
    try
    {
      input_bridge_ = cv_bridge::toCvCopy(image_msg, "mono8");  //灰度图读取
      output_bridge_ = cv_bridge::toCvCopy(image_msg, "bgr8");  //彩色图输出，一瞬间之后变暗
    }
    catch (cv_bridge::Exception& ex)//异常情况
    {
      ROS_ERROR("[calibrate] Failed to convert image:\n%s", ex.what());
      return;
    }

    Eigen::Vector3f translation; //1*3的列
    Eigen::Quaternionf orientation;//四元数(w,x,y,z)

    if (!pattern_detector_.detectPattern(input_bridge_->image, translation, orientation, output_bridge_->image))
    {
      ROS_INFO("[calibrate] Couldn't detect checkerboard, make sure it's visible in the image.");//无法检测棋盘，请确保它在图像中可见。
      return;
    }

    tf::Transform target_transform;
    tf::StampedTransform base_transform;
    try
    {
      ros::Time acquisition_time = image_msg->header.stamp;
      ros::Duration timeout(1.0 / 30.0);

      target_transform.setOrigin(tf::Vector3(translation.x(), translation.y(), translation.z()));
      target_transform.setRotation(tf::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w()));
      tf_broadcaster_.sendTransform(
          tf::StampedTransform(target_transform, image_msg->header.stamp, image_msg->header.frame_id, target_frame));
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[calibrate] TF exception:\n%s", ex.what());
      return;
    }
    publishCloud(ideal_points_, target_transform, image_msg->header.frame_id); //发布相机坐标下的点云

    overlayPoints(ideal_points_, target_transform, output_bridge_);  //画圈 编号

    // Publish calibration image
    pub_.publish(output_bridge_->toImageMsg());

    pcl_ros::transformPointCloud(ideal_points_, image_points_, target_transform);

    cout << "Got an image callback!" << endl;
    cout<< image_msg->header.frame_id<< endl;
    calibrate(image_msg->header.frame_id);

    ros::shutdown();//到这里全部停止，防止占用资源
  }

  void publishCloud(pcl::PointCloud<pcl::PointXYZ> detector_points, tf::Transform &transform,
                    const std::string frame_id)
  {
    // Display to rviz
    pcl::PointCloud < pcl::PointXYZ > transformed_detector_points;

    pcl_ros::transformPointCloud(detector_points, transformed_detector_points, transform);

    transformed_detector_points.header.frame_id = frame_id;
    detector_pub_.publish(transformed_detector_points);
  }

  void overlayPoints(pcl::PointCloud<pcl::PointXYZ> detector_points, tf::Transform &transform,
                     cv_bridge::CvImagePtr& image)
  {
    // Overlay calibration points on the image
    pcl::PointCloud < pcl::PointXYZ > transformed_detector_points;

    pcl_ros::transformPointCloud(detector_points, transformed_detector_points, transform);

    int font_face = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    double font_scale = 1;
    int thickness = 2;
    int radius = 5;

    for (unsigned int i = 0; i < transformed_detector_points.size(); i++)
    {
      pcl::PointXYZ pt = transformed_detector_points[i];
      cv::Point3d pt_cv(pt.x, pt.y, pt.z);
      cv::Point2d uv;
      uv = cam_model_.project3dToPixel(pt_cv); //3d转像素

      cv::circle(image->image, uv, radius, CV_RGB(255, 0, 0), -1); //画圆
      cv::Size text_size;
      int baseline = 0;
      std::stringstream out;
      out << i + 1;

      text_size = cv::getTextSize(out.str(), font_face, font_scale, thickness, &baseline);

      cv::Point origin = cvPoint(uv.x - text_size.width / 2, uv.y - radius - baseline/* - thickness*/);
      cv::putText(image->image, out.str(), origin, font_face, font_scale, CV_RGB(255, 0, 0), thickness);
    }
  }

  bool calibrate(const std::string frame_id)//将机械臂放在四个边缘角点
  {
    physical_points_.empty();
    physical_points_.header.frame_id = fixed_frame;
    cout << "Is the checkerboard correct? " << endl;
    cout << "Move edge of gripper to point 1 in image and press Enter. " << endl;
    cin.ignore();
    addPhysicalPoint();
    cout << "Move edge of gripper to point 2 in image and press Enter. " << endl;
    cin.ignore();
    addPhysicalPoint();
    cout << "Move edge of gripper to point 3 in image and press Enter. " << endl;
    cin.ignore();
    addPhysicalPoint();
    cout << "Move edge of gripper to point 4 in image and press Enter. " << endl;
    cin.ignore();
    addPhysicalPoint();

    Eigen::Matrix4f t;

    physical_pub_.publish(physical_points_);

    pcl::registration::TransformationEstimationSVD < pcl::PointXYZ, pcl::PointXYZ > svd_estimator;
    svd_estimator.estimateRigidTransformation(physical_points_, image_points_, t);

    // Output       
    tf::Transform transform = tfFromEigen(t), trans_full, camera_transform_unstamped;
    tf::StampedTransform camera_transform;

    cout << "Resulting transform (camera frame -> fixed frame): " << endl << t << endl << endl;//完成相机坐标系到机械臂fixed frame坐标系的转换

    try
    {
      tf_listener_.lookupTransform(frame_id, camera_frame, ros::Time(0), camera_transform); // 获得frame_id到camera_frame的转换
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[calibrate] TF exception:\n%s", ex.what());
      return false;
    }

    camera_transform_unstamped = camera_transform;
    trans_full = camera_transform_unstamped.inverse() * transform;

    Eigen::Matrix4f t_full = EigenFromTF(trans_full);
    Eigen::Matrix4f t_full_inv = (Eigen::Transform<float, 3, Affine>(t_full).inverse()).matrix();

    cout << "Resulting transform (fixed frame -> camera frame): " << endl << t_full << endl << endl;//机械臂到相机坐标系转换
    printStaticTransform(t_full_inv, fixed_frame, camera_frame);

    return true;
  }

  void printStaticTransform(Eigen::Matrix4f& transform, const std::string frame1, const std::string frame2)
  {
    Eigen::Quaternionf quat(transform.topLeftCorner<3, 3>());
    Eigen::Vector3f translation(transform.topRightCorner<3, 1>());

    cout << "Static transform publisher (use for external kinect): " << endl;//静态转换发布器(用于外部kinect)

    cout << "rosrun tf static_transform_publisher x y z qx qy qz qw frame_id child_frame_id period_in_ms" << endl;
    cout << "rosrun tf static_transform_publisher " << translation.x() << " " << translation.y() << " "
         << translation.z() << " " << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w()
         << " " << frame1 << " " << frame2 << " 100" << endl << endl;

    tf::Transform temp_tf_trans = tfFromEigen(transform);

    double yaw, pitch, roll;

    std::string fixed_frame_urdf(fixed_frame);

    // If there's a leading '/' character, remove it, as xacro can't deal with 
    // extra characters in the link name.
    if (fixed_frame_urdf.size() > 0 && fixed_frame_urdf[0] == '/')
      fixed_frame_urdf.erase(0, 1);

    temp_tf_trans.getBasis().getEulerYPR(yaw, pitch, roll);

    cout << "URDF output (use for kinect on robot): " << endl;

    cout << "<?xml version=\"1.0\"?>\n<robot>\n" << "\t<property name=\"turtlebot_calib_cam_x\" value=\""
         << translation.x() << "\" />\n" << "\t<property name=\"turtlebot_calib_cam_y\" value=\"" << translation.y()
         << "\" />\n" << "\t<property name=\"turtlebot_calib_cam_z\" value=\"" << translation.z() << "\" />\n"
         << "\t<property name=\"turtlebot_calib_cam_rr\" value=\"" << roll << "\" />\n"
         << "\t<property name=\"turtlebot_calib_cam_rp\" value=\"" << pitch << "\" />\n"
         << "\t<property name=\"turtlebot_calib_cam_ry\" value=\"" << yaw << "\" />\n"
         << "\t<property name=\"turtlebot_kinect_frame_name\" value=\"" << fixed_frame_urdf << "\" />\n" << "</robot>"
         << endl << endl;
  }

  void addPhysicalPoint()
  {
    geometry_msgs::PointStamped pt_out;

    try
    {
      tf_listener_.transformPoint(fixed_frame, gripper_tip, pt_out);
      cout<< pt_out;
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[calibrate] TF exception:\n%s", ex.what());
      return;
    }
    // cin >> pt_out.point.x;
    // cin >> pt_out.point.y;
    // cin >> pt_out.point.z;
    
    physical_points_.push_back(pcl::PointXYZ(pt_out.point.x, pt_out.point.y, pt_out.point.z));
  }

  void convertIdealPointstoPointcloud()
  {
    detector_points_.points.resize(pattern_detector_.ideal_points.size());
    for (unsigned int i = 0; i < pattern_detector_.ideal_points.size(); i++)
    {
      cv::Point3f pt = pattern_detector_.ideal_points[i];
      detector_points_[i].x = pt.x;
      detector_points_[i].y = pt.y;
      detector_points_[i].z = pt.z;
    }
  }

};

int main(int argc, char** argv)
{
  //ROS节点初始化，calibrate_kinect_arm为Publisher的节点名
  ros::init(argc, argv, "calibrate_kinect_arm");

  CalibrateKinectCheckerboard cal;//这里是什么意思？？？类，到进去类看
  ros::spin();                     //处理节点订阅话题的所有回调函数
}

```

```text

```

