#pragma once
#include "Arm/SaleArmLeft.h"
#include "Arm/SaleArmRight.h"
#include "HeadandLifting/HeadandLifting.h"
#include "Mobile/Mobile.h"
#include "Speech/Speech.h"
#include "Vision/Vision.h"

class Robot
{
public:
	static Robot *getRobot();
	~Robot();
	void Reconnect();
	
	Speech *CSpeech;
	Vision *CVision;
	HeadandLifting *CHeadandLifting;
	SaleArmLeft *CLeftArm;
	SaleArmRight *CRightArm;
	Mobile *CMobile;

private:
	static Robot *inst_;
	Robot();
};