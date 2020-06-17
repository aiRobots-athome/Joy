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
    CHeadandLifting = HeadandLifting::getHeadandLifting();
    CLeftArm = SaleArmLeft::getSaleArmLeft();
    CRightArm = SaleArmRight::getSaleArmRight();
    CMobile = Mobile::getMobile();

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
    inst_ = nullptr;
}