#pragma once
#include "./ScaraArm/ScaraArm.h"

class Scara
{
public:
    static Scara *getScara();
    ~Scara();

    ScaraArm *CScaraArm;

private:
    static Scara *inst_;
    Scara();
};

extern Scara *CScara;
extern ScaraArm *CScaraArm;