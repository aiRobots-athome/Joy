#pragma once
#include "./Steering/Steering.h"
#include "./Wheel/Wheel.h"

class Mobile
{
public:
	Steering *CSteering;
	Wheel *CWheel;
	static const int default_velocity;

	static Mobile *getMobile();
	~Mobile();

	/*
	@ Distance
	@ Velocity
	@ Direction (angle)
	*/
	void Move(const float &distance = 0.0f, const int &velocity = default_velocity, const float &direction = 0.0f);
	/*
	@ Distance
	@ Velocity
	*/
	void MoveForward(const float &distance = 0.0f, const int &velocity = default_velocity);
	/*
	@ Distance
	@ Velocity
	*/
	void MoveBackward(const float &distance = 0.0f, const int &velocity = default_velocity);
	/*
	@ Distance
	@ Velocity
	*/
	void MoveLeft(const float &distance = 0.0f, const int &velocity = default_velocity);
	/*
	@ Distance
	@ Velocity
	*/
	void MoveRight(const float &distance = 0.0f, const int &velocity = default_velocity);
	/*
	@ Direction (angle)
	@ Distance
	@ Velocity
	*/
	void Turn(const float &direction, const float &distance = 0.0f, const int &velocity = default_velocity);
	/*
	@ Radius
	@ Distance(circle angle)
	@ Velocity
	*/
	void TurnCircleByRadius(const float &radius, const float &distance = 0.0f, const int &velocity = default_velocity);
	/*
	@ Direction(angle)
	@ Velocity
	*/
	void SelfTurn(const float &distance = 0.0f, const int &velocity = default_velocity);

private:
	Mobile();
	static Mobile *inst_;

	const float kWheelBase;	  // 軸距
	const float kAxle;		  // 輪距
	const float kWheelBase_2; // 軸距/2 機器人中心到前(後)輪
	const float kAxle_2;	  // 輪距/2 機器人中心到左(右)輪
};