# 3. 相机内参和内部举着

### 信息回调函数

```cpp
  void infoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)//信息回调函数
  {
    if (calibrated)
      return;
    cam_model_.fromCameraInfo(info_msg);
    //相机内参，内部矩阵 ，[k1，k2，p1，p2，k3]
    pattern_detector_.setCameraMatrices(cam_model_.intrinsicMatrix(), cam_model_.distortionCoeffs());
    ROS_INFO("[calibrate] Camera IntrinsicMatrix: ");
    cout << endl << cam_model_.intrinsicMatrix() << endl << endl;
    ROS_INFO("[calibrate] Camera DistortionCoeffs: ");
    cout << endl << cam_model_.distortionCoeffs() << endl << endl;

    calibrated = true;
    image_sub_ = nh_.subscribe("/kinect2/hd/image_color_rect", 1, &CalibrateKinectCheckerboard::imageCallback, this);

    ROS_INFO("[calibrate] Got image info!");
  }

```

### 调用相机标定里面的内部矩阵和相机内参

```cpp
//相机内参，内部矩阵 ，[k1，k2，p1，p2，k3]
pattern_detector_.setCameraMatrices(cam_model_.intrinsicMatrix(), cam_model_.distortionCoeffs());
```

### 内部矩阵

```cpp
cam_model_.intrinsicMatrix()
```

### 相机内参

```cpp
cam_model_.distortionCoeffs()
```



