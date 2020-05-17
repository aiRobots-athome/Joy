#pragma once
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
  image_transport::Subscriber conf_sub_;
  image_transport::Subscriber confmap_sub_;
  image_transport::Publisher imge_pub_;
  image_transport::Publisher imge_pubd_;
  image_transport::Subscriber intelD435i_image_sub_;
  image_transport::Subscriber intelD435i_depth_sub_;

  ros::NodeHandle n_ros;
  ros::Subscriber Cam_Calib_sub_;
  ros::Subscriber Orientation;
  ros::Subscriber intelD435i_Cam_Calib_sub_;

public:
  //Constructor
  ImageConverter();
  ~ImageConverter(){};

  //function
  void SubImgFromCamera();
  void PubImg();
  void ImageCb(const sensor_msgs::ImageConstPtr &msg);
  void DepthCb(const sensor_msgs::ImageConstPtr &msg);
  void ConfCb(const sensor_msgs::ImageConstPtr &msg);
  void ConfmapCb(const sensor_msgs::ImageConstPtr &msg);
  void Cam_Calib_sub(const sensor_msgs::CameraInfo &msg);

  void GetImgForProcessing(int img_type); //0 for color_img/ 1 for depth_img/ 2 for depth_confidence_img
  // void GetDepthForProcessing();
  // void GetConfForProcessing();
  //parameter
  cv::Mat img_from_camera;
  cv::Mat img_for_processing;
  bool image_sub_flag = false;

  cv::Mat depth_from_camera;
  cv::Mat depth_for_processing;
  bool depth_sub_flag = false;

  cv::Mat conf_from_camera;
  cv::Mat conf_for_processing;
  bool conf_sub_flag = false;

  cv::Mat confmap_from_camera;
  cv::Mat confmap_for_processing;
  bool confmap_sub_flag = false;

  //Intel D435i
  void intelD435i_ImageCb(const sensor_msgs::ImageConstPtr &msg);
  void intelD435i_DepthCb(const sensor_msgs::ImageConstPtr &msg);
  void intelD435i_Cam_Calib_sub(const sensor_msgs::CameraInfo &msg);
  cv::Mat intelD435i_img_from_camera;
  cv::Mat intelD435i_img_for_processing;
  bool intelD435i_image_sub_flag = false;

  cv::Mat intelD435i_depth_from_camera;
  cv::Mat intelD435i_depth_for_processing;
  bool intelD435i_depth_sub_flag = false;

  void intelD435i_GetImgForProcessing(int img_type); //0 for color_img/ 1 for depth_img

  float fx;
  float fy;
  float cx;
  float cy;
  float k1;
  float k2;
  float k3;
  float p1;
  float p2;

  cv::Mat image_to_pub;
  cv::Mat image_d_to_pub;
};
