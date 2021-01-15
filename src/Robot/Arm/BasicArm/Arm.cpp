#include "Arm.h"

Arm::Arm(const vector<unsigned char> &IDArray, const vector<string> &MotorModelArrayList)
	: MotorUnion(IDArray, MotorModelArrayList),
	  FIRST_SHOULDER_ID_(0),
	  FIRST_HAND_ID_(1),
	  FIRST_FINGER_ID_(FIRST_HAND_ID_ + 6),
	  pro_radpersec2scale_(60.0 / (2 * M_PI) / 0.01),
	  delay_time_(10)
{
	is_working_ = false;
	is_out_of_limit_ = false;
	Start();
}

void Arm::Start()
{
	SetAllMotorsOperatingMode(1);
	SetAllMotorsVelocity(0);
	SetAllMotorsAccel(500);
	SetMotor_Accel(6, 1000);
	SetAllMotorsTorqueEnable(true);
}

void Arm::Stop()
{
	SetAllMotorsTorqueEnable(false);
}

/**
 * Implementation that can be used whenever you need a transform matrix.
 *
 * @param const float &theta - yeah, it's theta in DH table
 * @param const float &alpha - alpha in DH table
 * @param const float &a - a in DH table
 * @param const float &d - d in DH table
 * 
 * @retval Eigen::Matrix<float, 4, 4> transform_matrix - full transform matrix from i-1 to i
 */
Eigen::Matrix<float, 4, 4> Arm::GetTransformMatrix(const float &theta, const float &alpha, const float &a, const float &d)
{
	Eigen::Matrix<float, 4, 4> transform_matrix;
	transform_matrix << cos(theta), -cos(alpha) * sin(theta), sin(alpha) * sin(theta), a * cos(theta),
		sin(theta), cos(alpha) * cos(theta), -sin(alpha) * cos(theta), a * sin(theta),
		0, sin(alpha), cos(alpha), d,
		0, 0, 0, 1;

	return transform_matrix;
}

Eigen::Matrix<float, 3, 3> Arm::GetRotationMatrix(const int &axis_index, const float &theta)
{
	Eigen::Matrix<float, 3, 3> rotation_matrix;
	if (axis_index == 0)
	{
		rotation_matrix << 1, 0, 0,
			0, cos(theta), -sin(theta),
			0, sin(theta), cos(theta);
	}
	else if (axis_index == 1)
	{
		rotation_matrix << cos(theta), 0, sin(theta),
			0, 1, 0,
			-sin(theta), 0, cos(theta);
	}
	else if (axis_index == 2)
	{
		rotation_matrix << cos(theta), -sin(theta), 0,
			sin(theta), cos(theta), 0,
			0, 0, 1;
	}
	return rotation_matrix;
}

float Arm::GetCurrentPosition(int index)
{
	if (index == 0)
		return current_position_(0, 0);
	else if (index == 1)
		return current_position_(1, 0);
	else if (index == 2)
		return current_position_(2, 0);
	else
		return -1;
}

float Arm::GetCurrentOrientation(int index)
{
	if (index == 0)
		return current_orientation_(0, 0) * 180 / M_PI;
	else if (index == 1)
		return current_orientation_(1, 0) * 180 / M_PI;
	else if (index == 2)
		return current_orientation_(2, 0) * 180 / M_PI;
	else
		return -1;
}

bool Arm::GetWorkingState() { return is_working_; }

Eigen::Matrix<float, 6, 6> Arm::GetJacobianMatrix(void) { return jacobian_matrix_; }

int Arm::Sign(float x)
{
	if (x > 0)
		return 1;
	else if (x < 0)
		return -1;
	else
		return 0;
}

Eigen::Matrix<float, 6, 1> Arm::CheckMotorVelocity(Eigen::Matrix<float, 6, 1> motor_velocity)
{
	float max_speed = 0;
	float max_index = 0;
	for (int i = 0; i < 6; i++)
	{
		if (abs(motor_velocity(i, 0)) >= max_speed)
		{
			max_speed = abs(motor_velocity(i, 0));
			max_index = i;
		}
	}
	if (max_speed >= M_PI) // dangerous zone
	{
		is_out_of_limit_ = true;
	}
	else // limited zone
	{
		if (max_speed >= (M_PI / 6))
			motor_velocity *= abs((M_PI / 6) / motor_velocity(max_index, 0));
	}

	return motor_velocity;
}

float Arm::RootMeanSquareError(Eigen::Matrix<float, 3, 1> target_data, Eigen::Matrix<float, 3, 1> current_data)
{
	Eigen::Matrix<float, 3, 1> square = (target_data - current_data).array().square();
	float result = (square(0) + square(1) + square(2)) / 3;

	return sqrt(result);
}