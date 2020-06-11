#pragma once
#include "../MotorUnion/MotorUnion.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <fstream>

class HeadandLifting : public MotorUnion
{
public:
    static HeadandLifting *getHeadandLifting();
    ~HeadandLifting();

    /* Initialize */
    // Set in proper speed range for head
    void InitHeadMotorVelocity();

    /* Main Move Function */
    void HeadMotorCommand(const int &MotorID, const float &Angle);
    void ResetAllMotorAngle();

    /* Coordinate Transforming */
    void HeadCoordinate(float X_kinect, float Y_kinect, float depth, float &X_real, float &Y_real, float &Z_real);
    void RobotToCamera(float X_real, float Y_real, float Z_real, float &X_kinect, float &Y_kinect, float &depth);
    void HeadCoordinateKJ(float X_kinect, float Y_kinect, float depth, float &X_real, float &Y_real, float &Z_real);
    void Get_T_CamCoord2RobotCoord();

    /* JU */
    void GetObjectRotate_JU();
    float rotate_angle;

private:
    HeadandLifting();
    static HeadandLifting *inst_;

    int FIRST_HEAD_MOTOR_ID,
        LEFT_LIFTING_MOTOR_ID,
        RIGHT_LIFTING_MOTOR_ID,
        DisMotor8ToCenter,
        DisKinectToMotor8,
        DisShoulder2Waist;

    /* Matrix */
    cv::Mat *R_ID8;
    cv::Mat *R_ID9;

    cv::Mat *T_DisID8_ID9;
    cv::Mat *T_DisCamera2ID8;
    cv::Mat *T_Lens2CameraCenter;
    cv::Mat *T_CamCoord2RobotCoord;
    cv::Mat *T_CamCoord2RobotCoord_inv;
};