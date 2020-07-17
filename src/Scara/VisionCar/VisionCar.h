#pragma once
#include <fstream>
#include <iostream>
#include "../../Robot/MotorUnion/MotorUnion.h"

class VisionCar : public MotorUnion
{
public:
    static VisionCar *getVisionCar();
    ~VisionCar() { inst_ = nullptr; };
    
    enum pos_state {INSIDE=true, OUTSIDE=false, UP=true, DOWN=false};

    void Start();
    void Stop();


    void GotoPosition(const float &oz, const int &h,  const int &oc);
    void GoCarAngle(const float &goal_angle);
    bool GoScrewHeight(const bool &dir);
    bool GoCameraIO(const bool &IO);

    bool GetCamPos();
    bool GetScrewPos();

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