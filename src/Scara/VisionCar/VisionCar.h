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

    // Car
    void GoCarAngle(const int &goal_angle);
    
    // Screw
    int GoScrewHeight(const int &dir);
    int GetScrewPos();

    // Camera
    int GoCameraIO(const int &io);
    int GetCamPos();

    void Reset();

private:
    void ReadHeight();
    void WriteHeight(const float &height) const;

private:
    VisionCar();
    static VisionCar *inst_;

    const unsigned char FIRST_MOTOR_ID;

    const float CamInDegree=165.0;      // The absolute degree of camera motor when camera inside, in degree.
    const float CamOutDegree=100.0;     // The absolute degree of camera motor when camera outside, in degree.
    const float ScrewUpHeight = 580;      // The absolute height of screw when top, in mm.
    const float ScrewDownHeight = 126;    // The absolute height of screw when bottom, in mm.
    const int REV_2_SCREW;              // 1 rev equal to how many screw

    int current_cam_io=UNDEFINED;
    int current_screw_io=UNDEFINED;

    float now_height;
    bool VisionCarMotionEnable;
};