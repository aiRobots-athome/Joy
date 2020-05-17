#include "MotorMx.h"

MotorMx::MotorMx()
	: Motor(57600, 0, 64, 116, 104, 102, 132, 128, 126,  1, 4, 4, 2, 4, 4, 2) {}

MotorMx::MotorMx(const unsigned char &MotorID, const string &MotorModel)
	: Motor(57600, MotorID, 64, 116, 104, 102, 132, 128, 126, 1, 4, 4, 2, 4, 4, 2)
{
	if (MotorModel == "Mx106" || MotorModel == "Mx64")
	{
		Motor_CenterScale = 2048;
		Max_Position_Limit = 4095;
		Min_Position_Limit = 0;
		Max_Velocity_Limit = 210;
		Min_Velocity_Limit = -210;
		Max_Torque_Limit = 2047;
		Min_Torque_Limit = -2047;

		Angle2MotorScale = (Max_Position_Limit - Min_Position_Limit) / 360.0;
		MotorScale2Angle = 1.0 / Angle2MotorScale;
		Scale2RPM = 0.229;

		SetMotor_Velocity(-Max_Velocity_Limit);
		SetMotor_Torque(-Max_Torque_Limit);
		SetMotor_TorqueEnable(true);
	}
}