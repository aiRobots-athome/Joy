#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <spec_msg/spec_points.h>
#include <spec_msg/float32_1d.h>
#include <spec_msg/float32_2d.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>

using namespace std;

class PclConverter
{
  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_;
  ros::Subscriber ori_sub_;
  ros::Publisher pcl_xyz_pub_right_;
  ros::Publisher pcl_xyz_pub_left_;
  ros::Subscriber joint_sub_;
  ros::Publisher pcl_ptr_on_the_top_;
  ros::Subscriber joint_subr_;
  ros::Subscriber intelD435i_pcl_sub_;//JU

public:
  //constructor
  PclConverter();
  ~PclConverter(){};

  //function
  void SubCloudFromCamera();
  void PubCloudInPointRight(spec_msg::spec_points msg);
  void PubCloudInPointLeft(spec_msg::spec_points msg);
  void PubCloudInPoint(spec_msg::spec_points msg);
  void CloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  void GetCloudForProcessing();
  void GetVoxelForProcessing(pcl::PCLPointCloud2 ori);
  void Voxel2PointForPub(pcl::PointCloud<pcl::PointXYZRGB> voxel);
  void Clbk(const spec_msg::float32_2d::ConstPtr &msg);
  void Clbkr(const spec_msg::float32_2d::ConstPtr &msg);

  void SubInfo();
  void Infobk(const spec_msg::float32_1d::ConstPtr &msg);
  void GetOrientation();

  //parameter
  pcl::PCLPointCloud2 cloud_from_camera;
  pcl::PointCloud<pcl::PointXYZRGB> *cloud_for_processing;
  pcl::PointCloud<pcl::PointXYZRGB> *voxel_for_processing;

  bool pcl_sub_flag = false;
  bool get_orientation = false;
  spec_msg::spec_points pub_xyz_points;
  vector<float> Obj_Ori;
  vector<float> Obj_Ori_pass;

  bool joint_sub_flag = false;
  bool joint_subr_flag = false;
  vector<vector<float>> joint_vec;

  // intel D435i
  void intelD435i_CloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  bool intelD435i_pcl_sub_flag = false;
  void intelD435i_GetCloudForProcessing();
  pcl::PCLPointCloud2 intelD435i_cloud_from_camera;
  pcl::PointCloud<pcl::PointXYZRGB>* intelD435i_cloud_for_processing;
};