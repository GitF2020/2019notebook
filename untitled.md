---
description: 和turtlebot_arm_kinect_calibration对比
---

# 1.	代码对比

{% hint style="info" %}
ROS标定官方教程                                   [http://wiki.ros.org/turtlebot\_arm\_kinect\_calibration](http://wiki.ros.org/turtlebot_arm_kinect_calibration) turtlebot\_arm\_kinect\_calibration源码        [https://github.com/turtlebot/turtlebot\_arm](https://github.com/turtlebot/turtlebot_arm)
{% endhint %}

在线代码对比网站                                            [http://tool.oschina.net/diff](http://tool.oschina.net/diff)

## 

```cpp
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

```

{% code-tabs %}
{% code-tabs-item title="修改第五行" %}
```cpp
kinect2_rgb_optical_frame
```
{% endcode-tabs-item %}
{% endcode-tabs %}

{% code-tabs %}
{% code-tabs-item title="修改第九行" %}
```cpp
right_gripper_l_finger_tip
```
{% endcode-tabs-item %}
{% endcode-tabs %}

{% code-tabs %}
{% code-tabs-item title="修改了标定板的信息" %}
```cpp
nh_.param<int>("checkerboard_width", checkerboard_width, 7);
nh_.param<int>("checkerboard_height", checkerboard_height, 5);
nh_.param<double>("checkerboard_grid", checkerboard_grid, 0.03);        //识别棋盘长8格，宽6格，单个边长为3cm
```
{% endcode-tabs-item %}
{% endcode-tabs %}

## 主要修改了关于kinect2的一些Toptic名称

{% code-tabs %}
{% code-tabs-item title="如/kinect2/hd/camera\_info 和/kinect2/hd/image\_color\_rect" %}
```cpp
// Create subscriptions
info_sub_ = nh_.subscribe("/kinect2/hd/camera_info", 1, &CalibrateKinectCheckerboard::infoCallback, this);

image_sub_ = nh_.subscribe("/kinect2/hd/image_color_rect", 1, &CalibrateKinectCheckerboard::imageCallback, this);

```
{% endcode-tabs-item %}
{% endcode-tabs %}

