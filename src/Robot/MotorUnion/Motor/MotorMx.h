#pragma once
#include "Motor.h"

class MotorMx : public Motor
{
public:
	MotorMx();
	MotorMx(const unsigned char &MotorID, const string &MotorModel);
	~MotorMx(){};
};