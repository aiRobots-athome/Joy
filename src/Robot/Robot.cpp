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
    CLeftForceSensor = new ForceSensor("/dev/ttyUSB4");
    CRightForceSensor = new ForceSensor("/dev/ttyUSB5");

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
    delete CLeftForceSensor;
    delete CRightForceSensor;
    inst_ = nullptr;
}

void Robot::Reconnect()
{
    delete CHeadandLifting;
	delete CLeftArm;
	delete CRightArm;
	delete CMobile;
	MotorUnion::allport = {0, 1, 2, 3, 4, 5, 6};
	CHeadandLifting = HeadandLifting::getHeadandLifting();
	CLeftArm = SaleArmLeft::getSaleArmLeft();
	CRightArm = SaleArmRight::getSaleArmRight();
	CMobile = Mobile::getMobile();
}