#pragma once
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "../../Robot/MotorUnion/MotorUnion.h"

#define INSIDE true
#define OUTSIDE false
#define UP true
#define DOWN false

class VisionCar : public MotorUnion
{
public:
    static VisionCar *getVisionCar();
    ~VisionCar() { inst_ = nullptr; };
    
    void Start();
    void Stop();


    void GotoPosition(const float &oz, const int &h,  const int &oc);
    void GoCarAngle(const float &goal_angle);
    bool GoScrewHeight(const bool &dir);
    bool GoCameraIO(const bool &IO);

    void Reset();

private:
    VisionCar();
    static VisionCar *inst_;

    const unsigned char FIRST_MOTOR_ID;

    const float CamInDegree=170;
    const float CamOutDegree=90;

    bool current_cam_io;
    bool current_screw_io;
    bool VisionCarMotionEnable;
};