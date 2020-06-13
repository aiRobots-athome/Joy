#pragma once
#include <fstream>
#include<iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "../../Robot/Body/MotorUnion/MotorUnion.h"

class ScaraArm : public MotorUnion
{
public:
    ScaraArm(const vector<unsigned char> &IDArray,
				 const vector<string> &MotorModelArrayList,
				 vector<unsigned char> &AllPortNumber);
    ~ScaraArm(){};

    cv::Mat *GetKinematics();
    cv::Mat *Calculate_ArmForwardKinematics(float J1, float J2, float J3);
    cv::Mat *TransRotate(const float &x_angle, const float &y_angle, const float &z_angle);
    float *Arm_InverseKinematics(cv::Mat* T);
    void GOScrewHeight(float goal_height);
    float ReadSaveHeight();
    void WriteSaceHeight(float dat);
    void GotoPosition(cv::Mat *&T);
    void GotoPosition(const int &ox, const int &oy, const int &oz, const int &x, const int &y, const int &z);
    void ScaraGO(float ox, float oy, float oz, float x, float y, float z);
    void StopScrew();
    void InitArmMotor();
    void Reset();
    void AllmoterTorque(bool torque);
    void GOHeight1mm(bool direction);
    


private:
    float alength1_ini;
    float alength2_ini;
    float alength3_ini;
    float alength4_ini;
    float tmp_height;
    float delta_height_f;
    int need_position;

protected:
    const unsigned char FIRST_HAND_ID;
    const unsigned char HAND_AMOUNT;
    int round_value;
    bool ScaraArmMotionEnable;
    cv::Mat *ArmForward;
    
};