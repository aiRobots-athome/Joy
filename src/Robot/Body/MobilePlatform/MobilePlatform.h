#pragma once
#include "./Steering/Steering.h"
#include "./Wheel/Wheel.h"

class MobilePlatform
{
public:
	Steering *CSteering;
	Wheel *CWheel;
	static const int default_velocity;

	/*
	@ Steering ID List
	@ Steering Model List
	@ Wheel ID List
	@ Wheel Model List
	@ Port
	*/
	MobilePlatform(const vector<unsigned char> &steeringID,
				   const vector<string> &steeringModel,
				   const vector<unsigned char> &wheelID,
				   const vector<string> &wheelModel,
				   vector<unsigned char> &allPortNumber);
	~MobilePlatform();

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
	void Stop();

private:
	const float kWheelBase;   // 軸距
	const float kAxle;		  // 輪距
	const float kWheelBase_2; // 軸距/2 機器人中心到前(後)輪
	const float kAxle_2;	  // 輪距/2 機器人中心到左(右)輪
};