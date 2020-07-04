#include "Scara.h"

Scara *Scara::inst_ = nullptr;
Scara *Scara::getScara()
{
    if (inst_ == nullptr)
        inst_ = new Scara();
    return inst_;
}

Scara::Scara()
{
    /////////////////////////////////////////////////////////////////////
    /// Construct ScaraArm //////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    CScaraArm = ScaraArm::getScaraArm();

    /////////////////////////////////////////////////////////////////////
    /// Construct XY Platform ///////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    CXYPlatform = XYPlatform::getXYPlatform();
    
    cout << "\tClass constructed: Scara" << endl;
}

Scara::~Scara()
{
    delete CScaraArm;
    delete CXYPlatform;
    inst_ = nullptr;
}

void Scara::Reconnect()
{
    delete CScaraArm;
    delete CXYPlatform;
    MotorUnion::allport = {0, 1};
    CScaraArm = ScaraArm::getScaraArm();
    CXYPlatform = XYPlatform::getXYPlatform();
}