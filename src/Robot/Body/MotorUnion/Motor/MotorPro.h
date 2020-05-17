#pragma once
#include "Motor.h"

class MotorPro : public Motor
{
public:
	MotorPro();
	MotorPro(const unsigned char &MotorID, const string &MotorModel);
	~MotorPro(){};
};