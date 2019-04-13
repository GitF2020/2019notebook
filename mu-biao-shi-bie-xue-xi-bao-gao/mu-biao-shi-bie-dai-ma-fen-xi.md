# 目标识别代码分析

## 1.初始化ROS节点

封装ROS节点的第一步是加入ROS节点的初始化

```cpp
 rospy.init_node('tfobject')          
```

## 2.设置ROS参数（这一步被注释掉）

将图像话题名作为参数传入节点中

```cpp
#rospy.get_param("~image_topic", "") 
```

## 3.加入Subscriber 和 Publisher

创建订阅图像消息的Subscriber

```cpp
self._sub = rospy.Subscriber(image_topic, ROSImage, self.imgprogress, queue_size=10) 
```

发布最终识别结果的Publisher

```cpp
self._pub = rospy.Publisher('object_detection', ROSImage, queue_size=1)
```

## 4.OpenCV处理图像

接收到图像后，使用cv\_bridge将ROS图像转换成OpenCV的图像格式

```python
cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "rgb8") 
```

## 5.发布识别结果

图像处理完成后，发布识别结果

```python
self._pub.publish(ROSImage_pro)                 #发布图像信息
```

稍作延迟，等待下一次识别

```python
    def shutdown(self): 
        rospy.loginfo("Stopping the tensorflow object detection...") 
        rospy.sleep(1) 
```

