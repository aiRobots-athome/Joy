#pragma once
#include "ScaraArm/ScaraArm.h"
#include "XYPlatform/XYPlatform.h"

class Scara
{
public:
    static Scara *getScara();
    ~Scara();
    void Reconnect();

    ScaraArm *CScaraArm;
    XYPlatform *CXYPlatform;

private:
    static Scara *inst_;
    Scara();
};