#pragma once
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "../../Robot/MotorUnion/MotorUnion.h"

class ScaraArm : public MotorUnion
{
public:
    static ScaraArm *getScaraArm();
    ~ScaraArm() { inst_ = nullptr; };
    
    void Start();
    void Stop();

    // Arm
    cv::Mat GetKinematics();
    float &GetPresentHeight();
    void GotoPosition(const int &ox, const int &oy, const int &oz, const int &x, const int &y, const float &height);

    // Screw
    bool GoScrewHeight(const float &goal_height);
    void go_straight(float *head, float *tail, float h, float div);
    void go_straight_tmp(float *hed, float *goal, float h, float speed);

    void Reset();

private:
    // Arm
    cv::Mat Calculate_ArmForwardKinematics(const float &J1, const float &J2, const float &J3);
    cv::Mat TransRotate(const float &ox, const float &oy, const float &oz);
    float *Arm_InverseKinematics(const cv::Mat &T);
    void GotoPosition(const cv::Mat &T);
    void GotoPosition(const int &ox, const int &oy, const int &oz, const int &x, const int &y, const int &z);
    void SetPosition(const cv::Mat &T);
    void SetPosition(const int &ox, const int &oy, const int &oz, const int &x, const int &y, const int &z);
    cv::Mat cal_vel(cv::Mat head, cv::Mat tail, float speed);

    // Screw
    void ReadHeight();
    void WriteHeight(const float &height) const;

private:
    ScaraArm();
    static ScaraArm *inst_;

    const unsigned char FIRST_HAND_ID;
    
    const float J2_sign;            // Joint 2 direction, due to the machanic, Joint 2 has to be inverted
    const float Arm1_Length;        // Length from Joint 0 to 1
    const float Arm2_Length;        // Length from Joint 1 to 2
    const float Arm3_Length;        // Length from Joint 2 to 3
    const float Arm4_Length;        // Length from Joint 3 to 4
    const float Degree2Resolution;

    bool USING_BIG;                 // Which kind of scara used, 0 for big, 1 for small
    int Height_Resol;               // Resolution of present motor which control height
    const int PRO200_RESOL;         // Resolution of Pro200
    const int MX106_RESOL;          // Resolution of Mx106
    const int REV_2_SCREW;          // 1 rev equal to how many screw

    float now_height;
    bool ScaraArmMotionEnable;
};