#include "image_converter.h"

ImageConverter::ImageConverter() : it_(nh_)
{
  this->imge_pub_ = this->it_.advertise("/CNN/kj/img", 1);
  this->imge_pubd_ = this->it_.advertise("/CNN/kj/img_d", 1);
}

void ImageConverter::SubImgFromCamera()
{
  while (ros::ok)
  {
    this->image_sub_ = this->it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::ImageCb, this);
    this->depth_sub_ = this->it_.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &ImageConverter::DepthCb, this);
    this->conf_sub_ = this->it_.subscribe("/zed/confidence/confidence_image", 1, &ImageConverter::ConfCb, this);
    this->confmap_sub_ = this->it_.subscribe("/zed/confidence/confidence_map", 1, &ImageConverter::ConfmapCb, this);
    this->Cam_Calib_sub_ = this->n_ros.subscribe("/zed/rgb/camera_info", 1, &ImageConverter::Cam_Calib_sub, this);
    
    this->intelD435i_image_sub_ = this->it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::intelD435i_ImageCb, this);
    this->intelD435i_depth_sub_ = this->it_.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &ImageConverter::intelD435i_DepthCb, this);
    this->intelD435i_Cam_Calib_sub_ = this->n_ros.subscribe("/camera/color/camera_info", 1, &ImageConverter::intelD435i_Cam_Calib_sub, this);

    ros::spinOnce();
  }
}
void ImageConverter::PubImg()
{
  cv::Mat image = this->image_to_pub;
  // cv::Mat image = cv::imread("/home/kj/catkin_ws/src/may/src/Vision/img.jpg", CV_LOAD_IMAGE_COLOR);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  cv::Mat image_d = this->image_d_to_pub;
  // cv::Mat image_d = cv::imread("/home/kj/catkin_ws/src/may/src/Vision/img.jpg", CV_LOAD_IMAGE_COLOR);
  sensor_msgs::ImagePtr msg_d = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_d).toImageMsg();
  
  ros::Rate loop_rate(10);
  int i = 0;
  while (this->nh_.ok() && i < 1)
  {
    // this->imge_pub_.publish(msg);
    this->imge_pub_.publish(msg);
    this->imge_pubd_.publish(msg_d);
    ros::spinOnce();
    loop_rate.sleep();
    i++;
  }
}
void ImageConverter::ImageCb(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    this->img_from_camera = cv_ptr->image.clone();
    this->image_sub_flag = true;
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void ImageConverter::DepthCb(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    this->depth_from_camera = cv_ptr->image.clone();

    this->depth_sub_flag = true;
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void ImageConverter::ConfCb(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    this->conf_from_camera = cv_ptr->image.clone();
    this->conf_sub_flag = true;
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void ImageConverter::ConfmapCb(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    this->confmap_from_camera = cv_ptr->image.clone();
    this->confmap_sub_flag = true;
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}
void ImageConverter::Cam_Calib_sub(const sensor_msgs::CameraInfo &msg)
{
  try
  {
    {
      this->fx = msg.K[0];
      this->fy = msg.K[4];
      this->cx = msg.K[2];
      this->cy = msg.K[5];
      this->k1 = msg.D[0];
      this->k2 = msg.D[1];
      this->p1 = msg.D[2];
      this->p2 = msg.D[3];
      this->k3 = msg.D[4];
    }
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("Cameral Calibration Parameter exception: %s", e.what());
    return;
  }
}
void ImageConverter::GetImgForProcessing(int img_type = 0)
{
  if (img_type == 0)
  {
    if (image_sub_flag)
    {
      this->img_for_processing = this->img_from_camera.clone();
    }
    else
    {
      cout << "可能為攝影機無法讀入影像/未開啟SubImgFromCamera()讀取影像" << endl;
    }
  }
  else if (img_type == 1)
  {
    if (image_sub_flag)
    {
      this->depth_for_processing = this->depth_from_camera.clone();

      for (int i = 0; i < this->depth_for_processing.size().width; i++)
      {
        for (int j = 0; j < this->depth_for_processing.size().height; j++)
        {
          this->depth_for_processing.at<float>(j, i)=this->depth_for_processing.at<float>(j, i)/1000;
        }
      }
    }
    else
    {
      cout << "可能為攝影機無法讀入影像/未開啟SubImgFromCamera()讀取影像" << endl;
    }
  }
  else if (img_type == 2)
  {
    if (image_sub_flag)
    {
      this->conf_for_processing = this->conf_from_camera.clone();
    }
    else
    {
      cout << "可能為攝影機無法讀入影像/未開啟SubImgFromCamera()讀取影像" << endl;
    }
  }

  else if (img_type == 3)
  {
    if (image_sub_flag)
    {
      this->confmap_for_processing = this->confmap_from_camera.clone();
    }
    else
    {
      cout << "可能為攝影機無法讀入影像/未開啟SubImgFromCamera()讀取影像" << endl;
    }
  }
}

// void ImageConverter::GetDepthForProcessing()
// {
//   if(image_sub_flag)
//   {
//     this->depth_for_processing=this->depth_from_camera.clone();
//   }
//   else
//   {
//     cout << "可能為攝影機無法讀入影像/未開啟SubImgFromCamera()讀取影像" << endl;
//   }
// }
// // image_pub_ = it_.advertise("/image_converter/output_video", 1);
// // image_pub_.publish(cv_ptr->toImageMsg());

void ImageConverter::intelD435i_ImageCb(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    this->intelD435i_img_from_camera = cv_ptr->image.clone();
    this->intelD435i_image_sub_flag = true;
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void ImageConverter::intelD435i_GetImgForProcessing(int img_type = 0)
{
  if (img_type == 0)
  {
    if (intelD435i_image_sub_flag)
    {
      this->intelD435i_img_for_processing = this->intelD435i_img_from_camera.clone();
    }
    else
    {
      cout << "可能為攝影機無法讀入影像/未開啟SubImgFromCamera()讀取影像" << endl;
    }
  }
  else if (img_type == 1)
  {
    if (intelD435i_image_sub_flag)
    {
      this->intelD435i_depth_for_processing = this->intelD435i_depth_from_camera.clone();
    }
    else
    {
      cout << "可能為攝影機無法讀入影像/未開啟SubImgFromCamera()讀取影像" << endl;
    }
  }
}

void ImageConverter::intelD435i_DepthCb(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    this->intelD435i_depth_from_camera = cv_ptr->image.clone();
    this->intelD435i_depth_sub_flag = true;
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void ImageConverter::intelD435i_Cam_Calib_sub(const sensor_msgs::CameraInfo &msg)
{
  try
  {
    {
      this->fx = msg.K[0];
	    this->fy = msg.K[4];
	    this->cx = msg.K[2];
	    this->cy = msg.K[5];
      this->k1 = msg.D[0];
      this->k2 = msg.D[1];
      this->p1 = msg.D[2];
      this->p2 = msg.D[3];
      this->k3 = msg.D[4];
    }
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Cameral Calibration Parameter exception: %s", e.what());
    return;
  }
  
}