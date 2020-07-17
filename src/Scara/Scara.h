#pragma once
#include "ScaraArm/ScaraArm.h"
#include "XYPlatform/XYPlatform.h"
#include "VisionCar/VisionCar.h"

class Scara
{
public:
    static Scara *getScara();
    ~Scara();
    void Reconnect();

    ScaraArm *CScaraArm;
    XYPlatform *CXYPlatform;
    VisionCar *CVisionCar;

private:
    static Scara *inst_;
    Scara();
};