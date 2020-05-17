#include "pcl_converter.h"

PclConverter::PclConverter()
{
  cloud_for_processing = new pcl::PointCloud<pcl::PointXYZRGB>;
  voxel_for_processing = new pcl::PointCloud<pcl::PointXYZRGB>;
  intelD435i_cloud_for_processing = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl_xyz_pub_right_ = nh_.advertise<spec_msg::spec_points>("pcl_voxel_scene_right", 1);
  pcl_xyz_pub_left_ = nh_.advertise<spec_msg::spec_points>("pcl_voxel_scene", 1);
  pcl_ptr_on_the_top_ = nh_.advertise<spec_msg::spec_points>("/GU_ptr", 1);
}


void PclConverter::SubCloudFromCamera()
{
  while (ros::ok)
  {
    pcl_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &PclConverter::CloudCb, this);
    ori_sub_ = nh_.subscribe("/kj/orientation", 1, &PclConverter::Infobk, this);
    joint_sub_ = nh_.subscribe("Control", 1, &PclConverter::Clbk, this);
    joint_subr_ = nh_.subscribe("Control_r", 1, &PclConverter::Clbkr, this);
    intelD435i_pcl_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &PclConverter::intelD435i_CloudCb, this);

    ros::spinOnce();
  }
}

void PclConverter::CloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  pcl_conversions::toPCL(*cloud_msg, this->cloud_from_camera);
  pcl_sub_flag = true;
}
void PclConverter::Clbk(const spec_msg::float32_2d::ConstPtr &msg)
{
  // ROS_INFO("%d", msg->another_field);
  // ROS_INFO("first point: x=%.2f, y=%.2f", msg->points[0].x, msg->points[0].y);
  if (this->joint_sub_flag == false)
  {
    vector<vector<float>> tmp_2d;
    int x = msg->float32_2d_data.size();
    int y = msg->float32_2d_data[0].float_1d_data.size();

    for (int i = 0; i < msg->float32_2d_data.size(); i++)
    {
      vector<float> tmp_1d;
      for (int j = 0; j < msg->float32_2d_data[i].float_1d_data.size(); j++)
      {
        float tmp_f;
        tmp_f = msg->float32_2d_data[i].float_1d_data[j].data;
        tmp_1d.push_back(tmp_f);
      }
      tmp_2d.push_back(tmp_1d);
      // }
    }
    this->joint_vec.clear();
    this->joint_vec = tmp_2d;
    this->joint_sub_flag = true;
  }
}
void PclConverter::Clbkr(const spec_msg::float32_2d::ConstPtr &msg)
{
  // ROS_INFO("%d", msg->another_field);
  // ROS_INFO("first point: x=%.2f, y=%.2f", msg->points[0].x, msg->points[0].y);
  if (this->joint_subr_flag == false)
  {
    vector<vector<float>> tmp_2d;
    int x = msg->float32_2d_data.size();
    int y = msg->float32_2d_data[0].float_1d_data.size();

    for (int i = 0; i < msg->float32_2d_data.size(); i++)
    {
      vector<float> tmp_1d;
      for (int j = 0; j < msg->float32_2d_data[i].float_1d_data.size(); j++)
      {
        float tmp_f;
        tmp_f = msg->float32_2d_data[i].float_1d_data[j].data;
        tmp_1d.push_back(tmp_f);
      }
      tmp_2d.push_back(tmp_1d);
      // }
    }
    this->joint_vec.clear();
    this->joint_vec = tmp_2d;
    this->joint_subr_flag = true;
  }
}

void PclConverter::GetCloudForProcessing()
{
  if (pcl_sub_flag)
  {
    pcl::fromPCLPointCloud2(this->cloud_from_camera, *this->cloud_for_processing);
    for (int i = 0; i < this->cloud_for_processing->points.size(); i++)
    {
      float t_x = this->cloud_for_processing->points[i].z;
      float t_y = -(this->cloud_for_processing->points[i].x);
      float t_z = -(this->cloud_for_processing->points[i].y);

      this->cloud_for_processing->points[i].x = t_x;
      this->cloud_for_processing->points[i].y = t_y;
      this->cloud_for_processing->points[i].z = t_z;
    }
    cout << "here" << endl;
  }
  else
  {
    cout << "可能為攝影機無法讀入影像/未開啟SubCloudFromCamera()讀取影像" << endl;
  }
}
void PclConverter::GetVoxelForProcessing(pcl::PCLPointCloud2 ori)
{
  if (pcl_sub_flag)
  {
    int x = 1;
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    pcl::PCLPointCloud2::Ptr cloud_from_camera_ptr(new pcl::PCLPointCloud2());
    *cloud_from_camera_ptr = ori;
    //pcl::copyPointCloud(ori,*cloud_from_camera_ptr);
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    sor.setInputCloud(cloud_from_camera_ptr);
    sor.setLeafSize(0.02f, 0.02f, 0.02f);
    sor.filter(*cloud_filtered);
    pcl::fromPCLPointCloud2(*cloud_filtered, *this->voxel_for_processing);
  }
  else
  {
    cout << "可能為攝影機無法讀入影像/未開啟SubCloudFromCamera()讀取影像" << endl;
  }
}

void PclConverter::PubCloudInPointRight(spec_msg::spec_points msg)
{
  this->pcl_xyz_pub_right_.publish(msg);
  ros::spinOnce();
}
void PclConverter::PubCloudInPointLeft(spec_msg::spec_points msg)
{

  this->pcl_xyz_pub_left_.publish(msg);
  ros::spinOnce();
}
void PclConverter::PubCloudInPoint(spec_msg::spec_points msg)
{
  this->pcl_ptr_on_the_top_.publish(msg);
}

void PclConverter::Voxel2PointForPub(pcl::PointCloud<pcl::PointXYZRGB> voxel)
{
  spec_msg::spec_points tmp_points_array;
  for (int i = 0; i < voxel.width; i++)
  {
    geometry_msgs::Point tmp_xyz;
    geometry_msgs::Point tmp_rgb;

    tmp_xyz.x = voxel.points[i].x;
    tmp_xyz.y = voxel.points[i].y;
    tmp_xyz.z = voxel.points[i].z;

    tmp_rgb.x = voxel.points[i].r;
    tmp_rgb.y = voxel.points[i].g;
    tmp_rgb.z = voxel.points[i].b;

    tmp_points_array.points_xyz.push_back(tmp_xyz);
    tmp_points_array.points_rgb.push_back(tmp_rgb);
  }
  tmp_points_array.flag = 1;
  this->pub_xyz_points = tmp_points_array;
}

void PclConverter::SubInfo()
{
  // pcl_sub_ = nh_.subscribe("Orientation", 1, &PclConverter::Infobk, this);
}
void PclConverter::Infobk(const spec_msg::float32_1d::ConstPtr &msg)
{
  vector<float> tmp;
  for (int i = 0; i < msg->float_1d_data.size(); i++)
  {
    tmp.push_back(msg->float_1d_data[i].data);
  }
  this->Obj_Ori_pass = tmp;
  this->get_orientation = true;
  //cout<<"here"<<endl;
}
void PclConverter::GetOrientation()
{
  if (this->get_orientation)
  {
    this->Obj_Ori = this->Obj_Ori_pass;
    this->Obj_Ori_pass.clear();
  }
  else
  {
    cout << "Error Reading Object Orientation!" << endl;
  }
}

void PclConverter::intelD435i_CloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  pcl_conversions::toPCL(*cloud_msg, this->intelD435i_cloud_from_camera);
  intelD435i_pcl_sub_flag = true;
}

void PclConverter::intelD435i_GetCloudForProcessing()
{
  if (intelD435i_pcl_sub_flag)
  {
    pcl::fromPCLPointCloud2(this->intelD435i_cloud_from_camera, *this->intelD435i_cloud_for_processing);
    Eigen::Matrix4f transform2xyz = Eigen::Matrix4f::Identity();
    transform2xyz(0, 0) = 0;
    transform2xyz(0, 1) = 0;
    transform2xyz(0, 2) = 1;
    transform2xyz(0, 3) = 0;

    transform2xyz(1, 0) = -1;
    transform2xyz(1, 1) = 0;
    transform2xyz(1, 2) = 0;
    transform2xyz(1, 3) = 0;

    transform2xyz(2, 0) = 0;
    transform2xyz(2, 1) = -1;
    transform2xyz(2, 2) = 0;
    transform2xyz(2, 3) = 0;

    transform2xyz(3, 0) = 0;
    transform2xyz(3, 1) = 0;
    transform2xyz(3, 2) = 0;
    transform2xyz(3, 3) = 1;

    pcl::transformPointCloud(*this->intelD435i_cloud_for_processing, *this->intelD435i_cloud_for_processing, transform2xyz);
  }
  else
  {
    cout << "可能為攝影機無法讀入影像/未開啟SubCloudFromCamera()讀取影像" << endl;
  }
}