#pragma once
#include "Motor.h"

class MotorProPlus : public Motor
{
public:
	MotorProPlus();
	MotorProPlus(const unsigned char &MotorID, const string &MotorModel);
	~MotorProPlus(){};
};