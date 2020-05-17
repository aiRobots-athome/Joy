#pragma once
#include "../../MotorUnion/MotorUnion.h"

class Steering : public MotorUnion
{
public:
	/* 
	@ ID, 
	@ MotorModel, 
	@ Port
	*/
	Steering(const vector<unsigned char> &IDArray,
			 const vector<string> &MotorModelArray,
			 vector<unsigned char> &AllPortNumber);
	~Steering(){};

	/*
	! No argument (angle = 0)
	*/
	void TurnStraight();

	/*
	! No argument (angle = 90)
	*/
	void TurnHorizontal();

	/*
	@ All angle
	*/
	void TurnAll(const float &angle);

	/*
	@ Left-front angle
	@ Right-front angle
	*/
	void TurnFront(const float &angle1, const float &angle2);

	/*
	@ Left-front angle
	@ Right-front angle
	@ Left-back angle
	@ Right-back angle
	*/
	void TurnCircle(const float &angle1, const float &angle2, const float &angle3, const float &angle4);

	/*
	@ All angle (45 degree)
	*/
	void Self_Turn();

	void Wait();

private:
	// Motor ID
	const unsigned char steering_LF; //	(LF = left front)
	const unsigned char steering_RF; //	(RF = right front)
	const unsigned char steering_LB; //	(LB = left back)
	const unsigned char steering_RB; // (RB = right back)
	const float steering_gear_ratio; // 舵輪齒比
};