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

    /////////////////////////////////////////////////////////////////////
    /// Construct VisionCar /////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    CVisionCar = VisionCar::getVisionCar();  
    
    cout << "\tClass constructed: Scara" << endl;
}

Scara::~Scara()
{
    delete CScaraArm;
    delete CXYPlatform;
    delete CVisionCar;
    inst_ = nullptr;
}

/**
 * If motors fails to connect, regenerate scara and xy platform object
 */
void Scara::Reconnect() {
    delete CScaraArm;
    delete CXYPlatform;
    delete CVisionCar;
    MotorUnion::allport = {0, 1};
    CScaraArm = ScaraArm::getScaraArm();
    CXYPlatform = XYPlatform::getXYPlatform();
    CVisionCar = VisionCar::getVisionCar();
}