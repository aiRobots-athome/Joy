#pragma once
#include "Body/Body.h"
#include "Speech/Speech.h"
#include "Vision/Vision.h"

class Robot
{
public:
	static Robot *getRobot();
	~Robot();

	Body *CBody;
	Speech *CSpeech;
	Vision *CVision;

private:
	static Robot *inst_;
	Robot();
};

/* Robot */
extern Robot *CRobot;
/* Speech */
extern Speech *CSpeech;

/* Vision */
extern Vision *CVision; 

/* Body */
// Head and Lifting
extern HeadandLiftingPlatform *CHeadandLifting;
// Arm
extern SaleArmLeft *CLeftArm;
extern SaleArmRight *CRightArm;
// MobilePlatform
extern MobilePlatform *CMobilePlatform;
extern Steering *CSteering;
extern Wheel *CWheel;