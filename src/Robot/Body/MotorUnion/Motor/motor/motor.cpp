#include "motor.h"
motor::motor()
{
	Angle2MotorScale = 0.0f;
	MotorScale2Angle = 0.0f;
	Scale2RPM = 0.0f;

	Motor_CenterScale = 0;
	Motor_Angle = 0.0f;
	Motor_Scale = 0;
	Motor_Velocity = 0;
	Motor_Present_Angle = 0.0f;
	Motor_Present_Velocity = 0.0f;
	Motor_Present_Torque = 0.0f;
	Motor_Torque = 0;
	Motor_TorqueEnable = false;

	Max_Position_Limit = 0;
	Min_Position_Limit = 0;
	Max_Velocity_Limit = 0;
	Min_Velocity_Limit = 0;
	Max_Torque_Limit = 0;
	Min_Torque_Limit = 0;

	is_Arrival = true;
	is_Write_Scale = false;
	is_Write_Velocity = false;
	is_Write_Torque = false;
	is_Write_TorqueEnable = false;
}
//-----------------------------------------------------//
const float &motor::GetMotor_Scale2RPM() const { return Scale2RPM; }

const short &motor::GetMotor_CenterScale() const { return Motor_CenterScale; }

const float &motor::GetMotor_Angle() const { return Motor_Angle; }

const int &motor::GetMotor_Scale() const { return Motor_Scale; }

const int &motor::GetMotor_Velocity() const { return Motor_Velocity; }

const short &motor::GetMotor_Torque() const { return Motor_Torque; }

const bool &motor::GetMotor_TorqueEnable() const { return Motor_TorqueEnable; }

const float &motor::GetMotor_PresentAngle() const { return Motor_Present_Angle; }

const float &motor::GetMotor_PresentVelocity() const { return Motor_Present_Velocity; }

const float &motor::GetMotor_PresentTorque() const { return Motor_Present_Torque; }

const int &motor::GetMotor_Max_Position_Limit() const { return Max_Position_Limit; }

const int &motor::GetMotor_Min_Position_Limit() const { return Min_Position_Limit; }

const int &motor::GetMotor_Max_Velocity_Limit() const { return Max_Velocity_Limit; }

const int &motor::GetMotor_Min_Velocity_Limit() const { return Min_Velocity_Limit; }

const int &motor::GetMotor_Max_Torque_Limit() const { return Max_Torque_Limit; }

const int &motor::GetMotor_Min_Torque_Limit() const { return Min_Torque_Limit; }

const bool &motor::GetMotor_Arrival() const { return is_Arrival; }
//-----------------------------------------------------//
void motor::SetMotor_CenterScale(const short &a)
{
	Motor_CenterScale = a;
}
void motor::SetMotor_Angle(const float &fAngle)
{
	Motor_Scale = fAngle * Angle2MotorScale + Motor_CenterScale;

	if (Motor_Scale >= Max_Position_Limit)
		Motor_Scale = Max_Position_Limit;
	else if (Motor_Scale <= Min_Position_Limit)
		Motor_Scale = Min_Position_Limit;
	else
		;

	Motor_Angle = (Motor_Scale - Motor_CenterScale) * MotorScale2Angle;
	is_Arrival = false;
	is_Write_Scale = true;
}
void motor::SetMotor_Velocity(const int &velocity)
{
	if (velocity >= Max_Velocity_Limit)
		Motor_Velocity = Max_Velocity_Limit;
	else if (velocity <= Min_Velocity_Limit)
		Motor_Velocity = Min_Velocity_Limit;
	else
		Motor_Velocity = velocity;

	is_Write_Velocity = true;
}
void motor::SetMotor_Torque(const short &torque)
{
	if (torque >= Max_Torque_Limit)
		Motor_Torque = Max_Torque_Limit;
	else if (torque <= Min_Torque_Limit)
		Motor_Torque = Min_Torque_Limit;
	else
		Motor_Torque = torque;

	is_Write_Torque = true;
}
void motor::SetMotor_TorqueEnable(const bool &enable)
{
	Motor_TorqueEnable = enable;
	is_Write_TorqueEnable = true;
}
void motor::SetMotor_Velocity_withTime(const short &ms)
{
	float Deg_Change_X = std::abs(Motor_Angle - Motor_Present_Angle);
	float P_goal = Deg_Change_X / 360;			 // Round to go
	float V_goal = P_goal / (ms / 1000) * 60000; // RPM = Round/ms * (ms/min)
	float Velocity = V_goal / Scale2RPM;

	SetMotor_Velocity(Velocity);
}