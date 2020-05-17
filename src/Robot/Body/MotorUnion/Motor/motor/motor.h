#pragma once
#include <cmath>

#define Rad2Angle (180.0 / M_PI)
#define Angle2Rad (M_PI / 180.0)

class motor
{
public:
	motor();
	~motor(){};
	//-----------------------------------------------------//
	const float &GetMotor_Scale2RPM() const;
	const short &GetMotor_CenterScale() const;
	const float &GetMotor_Angle() const;
	const int &GetMotor_Scale() const;
	const int &GetMotor_Velocity() const;
	const short &GetMotor_Torque() const;
	const bool &GetMotor_TorqueEnable() const;
	const float &GetMotor_PresentAngle() const;
	const float &GetMotor_PresentVelocity() const;
	const float &GetMotor_PresentTorque() const;
	const int &GetMotor_Max_Position_Limit() const;
	const int &GetMotor_Min_Position_Limit() const;
	const int &GetMotor_Max_Velocity_Limit() const;
	const int &GetMotor_Min_Velocity_Limit() const;
	const int &GetMotor_Max_Torque_Limit() const;
	const int &GetMotor_Min_Torque_Limit() const;
	const bool &GetMotor_Arrival() const;
	//-----------------------------------------------------//
	void SetMotor_CenterScale(const short &);
	void SetMotor_Angle(const float &);
	void SetMotor_Velocity(const int &);
	void SetMotor_Velocity_withTime(const short &);
	void SetMotor_Torque(const short &);
	void SetMotor_TorqueEnable(const bool &);
	//------------------------------------------------------------------------------------------------------------------//

protected:
	/* Conversion of units */
	float Angle2MotorScale;
	float MotorScale2Angle;
	float Scale2RPM;

	/* Motor basic attributes */
	short Motor_CenterScale;
	float Motor_Angle;			  // (degree)
	int Motor_Scale;			  // Goal Position (Motor Scale)
	int Motor_Velocity;			  // Goal Velocity
	short Motor_Torque;			  // Goal Torque
	float Motor_Present_Angle;	// Present Position (degree)
	float Motor_Present_Velocity; // Present Velocity (rpm)
	float Motor_Present_Torque;   // Percentage loading (%) (Present_Current / MaxCurrent)
	bool Motor_TorqueEnable;

	/* Max Min Limit*/
	int Max_Position_Limit;
	int Min_Position_Limit;
	int Max_Velocity_Limit;
	int Min_Velocity_Limit;
	int Max_Torque_Limit;
	int Min_Torque_Limit;

	/* Background flags */
	bool is_Arrival;
	bool is_Write_Scale;
	bool is_Write_Velocity;
	bool is_Write_Torque;
	bool is_Write_TorqueEnable;
};
