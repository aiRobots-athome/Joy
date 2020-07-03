#include "MotorPro.h"

MotorPro::MotorPro()
	: Motor(57600, 0, 512, 552, 556, 560, 564, 574, 576, 580, 1, 4, 4, 4, 4, 2, 4, 4) {}

MotorPro::MotorPro(const unsigned char &MotorID, const string &MotorModel)
	: Motor(57600, MotorID, 512, 552, 556, 560, 564, 574, 576, 580, 1, 4, 4, 4, 4, 2, 4, 4)
{
	if (MotorModel == "Pro200")
	{
		Motor_CenterScale = 0;
		Max_Position_Limit = 501433;
		Min_Position_Limit = -501433;
		Max_Velocity_Limit = 2900;
		Min_Velocity_Limit = -2900;
		Max_Accel_Limit = 9982;
		Max_Torque_Limit = 22740;

		Angle2MotorScale = (Max_Position_Limit - Min_Position_Limit) / 360.0;
		MotorScale2Angle = 1.0 / Angle2MotorScale;
		Scale2RPM = 0.01;
		Scale2RPMM = 1;
	}
	else if (MotorModel == "Pro100")
	{
		Motor_CenterScale = 0;
		Max_Position_Limit = 501433;
		Min_Position_Limit = -501433;
		Max_Velocity_Limit = 2920;
		Min_Velocity_Limit = -2920;
		Max_Accel_Limit = 10639;
		Max_Torque_Limit = 15900;

		Angle2MotorScale = (Max_Position_Limit - Min_Position_Limit) / 360.0;
		MotorScale2Angle = 1.0 / Angle2MotorScale;
		Scale2RPM = 0.01;
		Scale2RPMM = 1;
	}
	else if (MotorModel == "Pro20")
	{
		Motor_CenterScale = 0;
		Max_Position_Limit = 303454;
		Min_Position_Limit = -303454;
		Max_Velocity_Limit = 2920;
		Min_Velocity_Limit = -2920;
		Max_Accel_Limit = 10765;
		Max_Torque_Limit = 4500;

		Angle2MotorScale = (Max_Position_Limit - Min_Position_Limit) / 360.0;
		MotorScale2Angle = 1.0 / Angle2MotorScale;
		Scale2RPM = 0.01;
		Scale2RPMM = 1;
	}
}