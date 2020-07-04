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
    void GoScrewHeight(const float &goal_height);

private:
    // Arm
    cv::Mat Calculate_ArmForwardKinematics(const float &J1, const float &J2, const float &J3);
    cv::Mat TransRotate(const float &ox, const float &oy, const float &oz);
    float *Arm_InverseKinematics(const cv::Mat &T);
    void GotoPosition(const cv::Mat &T);
    void GotoPosition(const int &ox, const int &oy, const int &oz, const int &x, const int &y, const int &z);

    // Screw
    void ReadHeight();
    void WriteHeight(const float &height) const;

private:
    ScaraArm();
    static ScaraArm *inst_;

    const unsigned char FIRST_HAND_ID;
    
    const char J2_sign;
    const float Arm1_Length;
    const float Arm2_Length;
    const float Arm3_Length;
    const float Arm4_Length;
    const float Degree2Resolution;

    float now_height;
    bool ScaraArmMotionEnable;
};