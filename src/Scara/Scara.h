#pragma once
#include "ScaraArm/ScaraArm.h"
#include "XYPlatform/XYPlatform.h"

class Scara
{
public:
    static Scara *getScara();
    ~Scara();

    ScaraArm *CScaraArm;
    XYPlatform *CXYPlatform;

private:
    static Scara *inst_;
    Scara();
};