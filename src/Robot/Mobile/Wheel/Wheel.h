#pragma once
#include "../../MotorUnion/MotorUnion.h"

class Wheel : public MotorUnion
{
public:
	static Wheel *getWheel();
	~Wheel() { inst_ = nullptr; };

	/*
	Wheel_LF_Velocity, 
	Wheel_RF_Velocity, 
	Wheel_LB_Velocity, 
	Wheel_RB_Velocity,
	Distance
	*/
	void Move(const int &velocity_LF,
			  const int &velocity_RF,
			  const int &velocity_LB,
			  const int &velocity_RB,
			  const float &distance = 0);
	/*
	All velocity,
	Distance
	*/
	void Move(const int &velocity, const float &distance = 0);
	/*
	All velocity,
	Distance
	*/
	void MoveForward(const int &velocity, const float &distance = 0);
	/*
	All velocity,
	Distance
	*/
	void MoveBackward(const int &velocity, const float &distance = 0);

	void Stop();

	void MoveByConstant(const int &velocity);
	void SelfTurnByConstant(const int &velocity);
	void LockOn(void);
	void Wait(void);

private:
	Wheel();
	static Wheel *inst_;

	// Motor ID
	const unsigned char wheel_LF; // (LF = left front)
	const unsigned char wheel_RF; // (RF = right front)
	const unsigned char wheel_LB; // (LB = left back)
	const unsigned char wheel_RB; // (RB = right back)

	const float wheel_gear_ratio; // 驅動輪齒輪比
	const float wheel_radius;	  // 驅動半徑(m)
	const float VEL2METER_MS;
	const float ACCEL2METER_MS2;
};