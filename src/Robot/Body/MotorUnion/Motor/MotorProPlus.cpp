#include "MotorProPlus.h"

MotorProPlus::MotorProPlus()
	: Motor(57600, 0, 512, 564, 552, 550, 580, 576, 574, 1, 4, 4, 2, 4, 4, 2) {}

MotorProPlus::MotorProPlus(const unsigned char &MotorID, const string &MotorModel)
	: Motor(57600, MotorID, 512, 564, 552, 550, 580, 576, 574, 1, 4, 4, 2, 4, 4, 2)
{
	if (MotorModel == "Pro20+")
	{
		Motor_CenterScale = 0;
		Max_Position_Limit = 303454;
		Min_Position_Limit = -303454;
		Max_Velocity_Limit = 2920;
		Min_Velocity_Limit = -2920;
		Max_Torque_Limit = 4500;
		Min_Torque_Limit = -4500;

		Angle2MotorScale = (Max_Position_Limit - Min_Position_Limit) / 360.0;
		MotorScale2Angle = 1.0 / Angle2MotorScale;
		Scale2RPM = 0.01;

		SetMotor_Velocity(0);
		SetMotor_Torque(Max_Torque_Limit);
		SetMotor_TorqueEnable(false);
	}
}