---
description: cv_bridge API
---

# 2. ROS下调用OpenCV

## 

### ros消息互动的结构       定义了OpenCV的input\_brige\_ 和output\_bridge\_

```cpp
// Structures for interacting with ROS messages 
cv_bridge::CvImagePtr input_bridge_;
cv_bridge::CvImagePtr output_bridge_;
```

## 调用Opencv检测标定板的大小\(width,height等\)

```cpp
pattern_detector_.setPattern(cv::Size(checkerboard_width, checkerboard_height),
checkerboard_grid, CHESSBOARD);
```

## 回调\(callback\)函数，调用OpenCV将标定板以灰度图读进来，以彩色图输出

```cpp
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
```

