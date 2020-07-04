#include "Mobile.h"

const int Mobile::default_velocity = 400;
Mobile *Mobile::inst_ = nullptr;
Mobile *Mobile::getMobile()
{
	if (inst_ == nullptr)
		inst_ = new Mobile();
	return inst_;
}

Mobile::Mobile()
	: kWheelBase(0.293),
	  kAxle(0.293),
	  kWheelBase_2(kWheelBase / 2),
	  kAxle_2(kAxle / 2)
{
	CSteering = Steering::getSteering();
	CWheel = Wheel::getWheel();
	cout << "\tClass constructed: Mobile" << endl;
}

Mobile::~Mobile()
{
	delete CSteering;
	delete CWheel;
	inst_ = nullptr;
}

void Mobile::Move(const float &distance, const int &velocity, const float &direction)
{
	float tmp_angle = direction;
	int tmp_velocity = velocity;

	if (abs(direction) > 90)
	{
		tmp_angle = copysignf(180 - abs(direction), -direction);
		tmp_velocity = -abs(velocity);
	}
	else
	{
		tmp_angle = direction;
		tmp_velocity = velocity;
	}

	CSteering->TurnAll(tmp_angle);
	CSteering->Wait();
	CWheel->Move(tmp_velocity, distance);
}

void Mobile::MoveForward(const float &distance, const int &velocity)
{
	Move(distance, abs(velocity), 0);
}

void Mobile::MoveBackward(const float &distance, const int &velocity)
{
	Move(distance, -abs(velocity), 0);
}

void Mobile::MoveLeft(const float &distance, const int &velocity)
{
	Move(distance, abs(velocity), 90);
}

void Mobile::MoveRight(const float &distance, const int &velocity)
{
	Move(distance, -abs(velocity), 90);
}

void Mobile::Turn(const float &direction, const float &distance, const int &velocity)
{
	float tmp_angle = direction;
	int tmp_velocity = velocity;

	// direction boundary
	if (abs(direction) > 45 && abs(direction) <= 90)
	{
		tmp_angle = copysignf(45, direction);
		tmp_velocity = velocity;
	}
	else if (abs(direction) > 90 && abs(direction) <= 135)
	{
		tmp_angle = copysignf(45, -direction);
		tmp_velocity = -velocity;
	}
	else if (abs(direction) > 135 && abs(direction) <= 180)
	{
		tmp_angle = copysignf(180 - abs(direction), -direction);
		tmp_velocity = -velocity;
	}
	else
		;

	// Calculate Ackermann steering model
	const float radius = kWheelBase / tan(tmp_angle * Angle2Rad); // this radius is circle center to back wheel center

	const float inside_theta = atan(kWheelBase / copysignf(abs(radius) - kAxle_2, radius));
	const float outside_theta = atan(kWheelBase / copysignf(abs(radius) + kAxle_2, radius));

	const float inside_front_radius = kWheelBase / sin(inside_theta);
	const float outside_front_radius = kWheelBase / sin(outside_theta);
	const float inside_back_radius = abs(radius) - kAxle_2;
	const float outside_back_radius = abs(radius) + kAxle_2;

	const int inside_front_velocity = abs(inside_front_radius / radius) * tmp_velocity;
	const int outside_front_velocity = abs(outside_front_radius / radius) * tmp_velocity;
	const int inside_back_velocity = abs(inside_back_radius / radius) * tmp_velocity;
	const int outside_back_velocity = abs(outside_back_radius / radius) * tmp_velocity;

	const float inside_angle = inside_theta * Rad2Angle;
	const float outside_angle = outside_theta * Rad2Angle;
	// Move
	if (tmp_angle > 0)
	{
		CSteering->TurnFront(inside_angle, outside_angle);
		CSteering->Wait();
		CWheel->Move(-inside_front_velocity, outside_front_velocity, -inside_back_velocity, outside_back_velocity, distance);
	}
	else
	{
		CSteering->TurnFront(outside_angle, inside_angle);
		CSteering->Wait();
		CWheel->Move(-outside_front_velocity, inside_front_velocity, -outside_back_velocity, inside_back_velocity, distance);
	}
}

void Mobile::TurnCircleByRadius(const float &radius, const float &distance, const int &velocity)
{
	const float inside_theta = atan(kWheelBase_2 / copysignf(abs(radius) - kAxle_2, radius));
	const float outside_theta = atan(kWheelBase_2 / copysignf(abs(radius) + kAxle_2, radius));

	const float inside_radius = kWheelBase_2 / sin(inside_theta);
	const float outside_radius = kWheelBase_2 / sin(outside_theta);

	const int inside_velocity = abs(inside_radius / radius) * velocity;
	const int outside_velocity = abs(outside_radius / radius) * velocity;

	const float inside_angle = inside_theta * Rad2Angle;
	const float outside_angle = outside_theta * Rad2Angle;

	if (radius > 0)
	{
		CSteering->TurnCircle(inside_angle, outside_angle, -inside_angle, -outside_angle);
		CSteering->Wait();
		CWheel->Move(-inside_velocity, outside_velocity, -inside_velocity, outside_velocity, distance * Angle2Rad * radius);
	}
	else
	{
		CSteering->TurnCircle(outside_angle, inside_angle, -outside_angle, -inside_angle);
		CSteering->Wait();
		CWheel->Move(-outside_velocity, inside_velocity, -outside_velocity, inside_velocity, distance * Angle2Rad * radius);
	}
}

void Mobile::SelfTurn(const float &distance, const int &velocity)
{
	float radius = sqrt(pow(kAxle_2, 2) + pow(kWheelBase_2, 2));
	int tmp_velocity = velocity;
	if (distance < 0)
		tmp_velocity = -abs(velocity);

	CSteering->Self_Turn();
	CSteering->Wait();
	CWheel->Move(tmp_velocity, tmp_velocity, tmp_velocity, tmp_velocity, distance * Angle2Rad * radius);
}