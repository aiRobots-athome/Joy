#include "Robot.h"
Robot *Robot::inst_ = nullptr;

Robot *Robot::getRobot()
{
    if (inst_ == nullptr)
        inst_ = new Robot();
    return inst_;
}

Robot::Robot()
{
    CVision = new Vision();
    CSpeech = new Speech();
    HeadandLifting *CHeadandLifting = HeadandLifting::getHeadandLifting();
    SaleArmLeft *CLeftArm = SaleArmLeft::getSaleArmLeft();
    SaleArmRight *CRightArm = SaleArmRight::getSaleArmRight();
    Mobile *CMobile = Mobile::getMobile();
    Steering *CSteering = Steering::getSteering();
    Wheel *CWheel = Wheel::getWheel();

    cout << "Class constructed: Robot" << endl;
    cout << "================================================================================" << endl;
}

Robot::~Robot()
{
    delete CVision;
    delete CSpeech;
    delete CHeadandLifting;
    delete CLeftArm;
    delete CRightArm;
    delete CMobile;
}