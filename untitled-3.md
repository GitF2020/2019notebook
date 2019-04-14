# 4. TF坐标转换

## frame id 

先对ROS中的坐标简要说明，ROS的坐标通过 / tf topic 来管理，tf是一个多叉树结构，每一个坐标通过一个frame id识别，使用代码来管理tf，所有坐标都是通过 / tf由程序员自己管理。 按照官方指南，tf通过broadcast, listen, add fixed frame。

## 关于TF 和 在标定板画角点和标1，2，3，4

```cpp
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
	
	    ros::shutdown();
	  }

```

## frame\_id 的作用

​



