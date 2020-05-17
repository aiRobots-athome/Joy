#pragma once
#include "../../MotorUnion/MotorUnion.h"

class Wheel : public MotorUnion
{
public:
	/* 
	@ ID, 
	@ MotorModel, 
	@ Port
	*/
	Wheel(const vector<unsigned char> &IDArray,
		  const vector<string> &MotorModelArray,
		  vector<unsigned char> &AllPortNumber);
	~Wheel(){};

	/*
	@ All velocity
	*/
	void Move(const int &velocity);
	/*
	@ All velocity
	*/
	void MoveForward(const int &velocity);
	/*
	@ All velocity
	*/
	void MoveBackward(const int &velocity);
	/*
	@ Wheel_LF_Velocity, 
	@ Wheel_RF_Velocity, 
	@ Wheel_LB_Velocity, 
	@ Wheel_RB_Velocity
	*/
	void Move(const int &velocity_LF, const int &velocity_RF, const int &velocity_LB, const int &velocity_RB);
	/*
	Stop wheel moving
	*/
	void Stop();
	/*
	# Return milliseconds by distance 
	@ Velocity(scale), 
	@ Time(millisecond)
	*/
	const int CalculateDistanceTime(const float &distance, const int &velocity);
	/*
	@ waiting time (ms)
	*/
	void Wait(const int waiting_time_ms) const;

private:
	// Motor ID
	const unsigned char wheel_LF; // (LF = left front)
	const unsigned char wheel_RF; // (RF = right front)
	const unsigned char wheel_LB; // (LB = left back)
	const unsigned char wheel_RB; // (RB = right back)

	const float wheel_gear_ratio; // 驅動輪齒輪比
	const float wheel_radius;	 // 驅動半徑(m)
	const float scale2meter_ms;

	const int phase;
	int delay;
	float softstart_distance;

	/*
	Turn off torque if velocity is 0, vice versa
	*/
	void SwitchTorqueEnable(const int &velocity_LF, const int &velocity_RF, const int &velocity_LB, const int &velocity_RB);
	/*
	Make a soft start
	*/
	void SoftStart(const int &velocity_LF, const int &velocity_RF, const int &velocity_LB, const int &velocity_RB);
};