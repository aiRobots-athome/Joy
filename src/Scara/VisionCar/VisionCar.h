#pragma once
#include <fstream>
#include <iostream>
#include "../../Robot/MotorUnion/MotorUnion.h"

class VisionCar : public MotorUnion
{
public:
    static VisionCar *getVisionCar();
    ~VisionCar() { inst_ = nullptr; };
    
    enum pos_state {INSIDE=0, OUTSIDE=1, DOWN=0, UP=1, UNDEFINED=2};

    void Start();
    void Stop();

    void GotoPosition(const int &oz, const int &h,  const int &oc);
    void GoCarAngle(const int &goal_angle);
    int GoScrewHeight(const int &dir);
    int GoCameraIO(const int &io);

    int GetCamPos();
    int GetScrewPos();

    void Reset();

private:
    VisionCar();
    static VisionCar *inst_;

    const unsigned char FIRST_MOTOR_ID;

    const float CamInDegree=165.0;
    const float CamOutDegree=100.0;

    int current_cam_io=VisionCar::UNDEFINED;
    int current_screw_io=VisionCar::UNDEFINED;
    bool VisionCarMotionEnable;
};