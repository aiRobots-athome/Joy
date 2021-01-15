#include "MotorPro.h"

MotorPro::MotorPro()
	: Motor(57600, 0, 512, 552, 556, 560, 564, 574, 576, 580, 11, 1, 4, 4, 4, 4, 2, 4, 4) {}

MotorPro::MotorPro(const unsigned char &MotorID, const string &MotorModel)
	: Motor(4000000, MotorID, 512, 552, 556, 560, 564, 574, 576, 580, 11, 1, 4, 4, 4, 4, 2, 4, 4)
{
	if (MotorModel == "Pro200")
	{
		Motor_CenterScale = 0;
		Max_Position_Limit = 501433;
		Min_Position_Limit = -501433;
		Max_Velocity_Limit = 2900;
		Min_Velocity_Limit = -2900;
		Max_Extend_Limit = 2147483647;
		Min_Extend_Limit = -2147483648;
		Max_Value_In_1_rev = 501923;			// Maximum value in one round in extended mode
		Min_Value_In_1_rev = -501923;			// Minimum value in one round in extended mode
		Max_Accel_Limit = 1000000;
		Max_Torque_Limit = 22740;

		Angle2MotorScale = (Max_Position_Limit - Min_Position_Limit) / 360.0;
		MotorScale2Angle = 1.0 / Angle2MotorScale;
		Rev_Resol_Extended = Max_Value_In_1_rev - Min_Value_In_1_rev;
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
		Max_Extend_Limit = 2147483647;
		Min_Extend_Limit = -2147483648;
		Max_Value_In_1_rev = 501923;			// Maximum value in one round in extended mode
		Min_Value_In_1_rev = -501923;			// Minimum value in one round in extended mode
		Max_Accel_Limit = 10639;
		Max_Torque_Limit = 15900;

		Angle2MotorScale = (Max_Position_Limit - Min_Position_Limit) / 360.0;
		MotorScale2Angle = 1.0 / Angle2MotorScale;
		Rev_Resol_Extended = Max_Value_In_1_rev - Min_Value_In_1_rev;
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
		Max_Extend_Limit = 2147483647;
		Min_Extend_Limit = -2147483647;
		Max_Accel_Limit = 1000000;
		Max_Torque_Limit = 4500;
		Max_Extend_Limit = 2147483647;
		Min_Extend_Limit = -2147483648;
		Max_Value_In_1_rev = 303750;			// Maximum value in one round in extended mode
		Min_Value_In_1_rev = -303750;			// Minimum value in one round in extended mode

		Angle2MotorScale = (Max_Position_Limit - Min_Position_Limit) / 360.0;
		MotorScale2Angle = 1.0 / Angle2MotorScale;
		Rev_Resol_Extended = Max_Value_In_1_rev - Min_Value_In_1_rev;
		Scale2RPM = 0.01;
		Scale2RPMM = 1;
	}
}