#include "MotorPro.h"

MotorPro::MotorPro()
	: Motor(57600, 0, 562, 596, 600, 604, 611, 615, 621, 1, 4, 4, 2, 4, 4, 2) {}

MotorPro::MotorPro(const unsigned char &MotorID, const string &MotorModel)
	: Motor(57600, MotorID, 562, 596, 600, 604, 611, 615, 621, 1, 4, 4, 2, 4, 4, 2)
{
	if (MotorModel == "Pro200")
	{
		Motor_CenterScale = 0;
		Max_Position_Limit = 250961;
		Min_Position_Limit = -250961;
		Max_Velocity_Limit = 17000;
		Min_Velocity_Limit = -17000;
		Max_Torque_Limit = 620;
		Min_Torque_Limit = -620;

		Angle2MotorScale = (Max_Position_Limit - Min_Position_Limit) / 360.0;
		MotorScale2Angle = 1.0 / Angle2MotorScale;
		Scale2RPM = 0.00199234;

		SetMotor_Velocity(2000);
		SetMotor_Torque(Max_Torque_Limit);
		SetMotor_TorqueEnable(true);
	}
	else if (MotorModel == "Pro100")
	{
		Motor_CenterScale = 0;
		Max_Position_Limit = 250961;
		Min_Position_Limit = -250961;
		Max_Velocity_Limit = 17000;
		Min_Velocity_Limit = -17000;
		Max_Torque_Limit = 310;
		Min_Torque_Limit = -310;

		Angle2MotorScale = (Max_Position_Limit - Min_Position_Limit) / 360.0;
		MotorScale2Angle = 1.0 / Angle2MotorScale;
		Scale2RPM = 0.00199234;

		SetMotor_Velocity(0);
		SetMotor_Torque(Max_Torque_Limit);
		SetMotor_TorqueEnable(false);
	}
	else if (MotorModel == "Pro20")
	{
		Motor_CenterScale = 0;
		Max_Position_Limit = 151875;
		Min_Position_Limit = -151875;
		Max_Velocity_Limit = 10300;
		Min_Velocity_Limit = -10300;
		Max_Torque_Limit = 465;
		Min_Torque_Limit = -465;

		Angle2MotorScale = (Max_Position_Limit - Min_Position_Limit) / 360.0;
		MotorScale2Angle = 1.0 / Angle2MotorScale;
		Scale2RPM = 0.00329218;

		SetMotor_Velocity(2000);
		SetMotor_Torque(250);
		SetMotor_TorqueEnable(true);
	}
}