#include "ScaraArm.h"

#define USE_BIG

ScaraArm *ScaraArm::inst_ = nullptr;

/**
 * Alternatively new scara arm object
 * To ensure we only have one scara arm object alive at a time
 */
ScaraArm *ScaraArm::getScaraArm()
{
	if (inst_ == nullptr)
		inst_ = new ScaraArm();
	return inst_;
}

ScaraArm::ScaraArm()
#ifdef USE_BIG
	/* Big */
	: MotorUnion({0, 1, 2, 3}, {"Pro200", "Pro200", "Pro20", "Pro20"}),
	  LINK0_LENGTH_(390),
	  LINK1_LENGTH_(238),
	  LINK2_LENGTH_(242),
	  SCREW_MOTOR_ID_(0),
#else
	/* Small */
	: MotorUnion({0, 1, 2, 3}, {"Mx106", "Mx106", "Mx106", "Mx106"}),
	  LINK0_LENGTH_(92),
	  LINK1_LENGTH_(92),
	  LINK2_LENGTH_(69),
	  SCREW_MOTOR_ID_(0),

#endif
	  /* Common variable */
	  PRO200_RESOLUTION_(1003846),
	  MX106_RESOLUTION_(4096),
	  PRO_RADS2SCALE_(60.0 / (2 * M_PI) / 0.01),
	  MX_RADS2SCALE_(60.0),
	  REV_2_SCREW(226)
{
	std::cout << "\t\tClass constructed: ScaraArm";

#ifdef USE_BIG
	std::cout << " (BIG version)" << std::endl;
#else
	std::cout << " (SMALL version)" << std::endl;
#endif
	Start();
	ReadHeight();
}

/**
 * Enable all motors, and set up the speed
 */
void ScaraArm::Start()
{
	SetAllMotorsOperatingMode(1);
	SetMotor_Operating_Mode(SCREW_MOTOR_ID_, 4);
	SetAllMotorsVelocity(0);
	SetAllMotorsTorqueEnable(true);
	is_out_of_limit_ = false;
	is_working_ = false;

#ifdef USE_BIG
	/* Big */
	SetAllMotorsAccel(1500);
#else
	/* Small */
	SetAllMotorsAccel(200);
#endif
}

/**
 * Disable all motors
 */
void ScaraArm::Stop() { SetAllMotorsTorqueEnable(false); }

float ScaraArm::RootMeanSquareError(Eigen::Matrix<float, 3, 1> target_data, Eigen::Matrix<float, 3, 1> current_data)
{
	Eigen::Matrix<float, 3, 1> square = (target_data - current_data).array().square();
	float result = (square(0) + square(1) + square(2)) / 3;

	return sqrt(result);
}

Eigen::Matrix<float, 3, 1> ScaraArm::CheckMotorVelocity(Eigen::Matrix<float, 3, 1> motor_velocity)
{
	/* Determine the index of max velocity */
	float max_speed = 0;
	float max_index = 0;
	for (int i = 0; i < 3; i++)
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

Eigen::Matrix<float, 4, 4> ScaraArm::GetTransformMatrix(const float &theta, const float &alpha, const float &a, const float &d)
{
	Eigen::Matrix<float, 4, 4> transform_matrix;
	transform_matrix << cos(theta), -cos(alpha) * sin(theta), sin(alpha) * sin(theta), a * cos(theta),
		sin(theta), cos(alpha) * cos(theta), -sin(alpha) * cos(theta), a * sin(theta),
		0, sin(alpha), cos(alpha), d,
		0, 0, 0, 1;

	return transform_matrix;
}

float ScaraArm::GetCurrentPosition(int index)
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

float ScaraArm::GetCurrentOrientation(int index)
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

bool ScaraArm::GetWorkingState(void) { return is_working_; }

void ScaraArm::SetArmVelocity(float v0, float v1, float v2)
{
	SetMotor_Velocity(1, v0 * PRO_RADS2SCALE_);
	SetMotor_Velocity(2, v1 * PRO_RADS2SCALE_);
	SetMotor_Velocity(3, v2 * PRO_RADS2SCALE_);
}

void ScaraArm::CalculateJacobianMatrix()
{
	/* get motor delta angle*/
	float J0 = GetMotor_PresentAngle(1) * Angle2Rad;
	float J1 = GetMotor_PresentAngle(2) * Angle2Rad;
	float J2 = GetMotor_PresentAngle(3) * Angle2Rad;

	/* basic transform matrix from i-1 to i */
	/* the sign of delta angle is decided by the motor setting direction and the definition of coordination */
	Eigen::Matrix<float, 4, 4> T01 = GetTransformMatrix(J0, M_PI, LINK0_LENGTH_, 0);
	Eigen::Matrix<float, 4, 4> T12 = GetTransformMatrix(J1, M_PI, LINK1_LENGTH_, 0);
	Eigen::Matrix<float, 4, 4> T23 = GetTransformMatrix(J2, 0, LINK2_LENGTH_, 0);

	/* transform matrix based on the 1st coordination */
	Eigen::Matrix<float, 4, 4> T02 = T01 * T12;
	Eigen::Matrix<float, 4, 4> T03 = T02 * T23;

	/* end effector oreientation */
	float oz = atan2(T03(1, 0), T03(0, 0));
	float oy = atan2(-T03(2, 0), (T03(0, 0) * cos(oz) + T03(1, 0) * sin(oz)));
	float ox = atan2((T03(0, 2) * sin(oz) - T03(1, 2) * cos(oz)), (T03(1, 1) * cos(oz) - T03(0, 1) * sin(oz)));

	current_orientation_ << ox, oy, oz;

	/* positions based on the 1st coordination */
	Eigen::Matrix<float, 3, 1> P00;
	Eigen::Matrix<float, 3, 1> P01;
	Eigen::Matrix<float, 3, 1> P02;
	Eigen::Matrix<float, 3, 1> P03;

	P00 << 0, 0, 0;
	P01 << T01(0, 3), T01(1, 3), T01(2, 3);
	P02 << T02(0, 3), T02(1, 3), T02(2, 3);
	P03 << T03(0, 3), T03(1, 3), T03(2, 3);

	current_position_ = P03;

	/* rotation based on the 1st coordination (upper part of the jacobian matrix) */
	Eigen::Matrix<float, 3, 1> Z00;
	Eigen::Matrix<float, 3, 1> Z01;
	Eigen::Matrix<float, 3, 1> Z02;
	Eigen::Matrix<float, 3, 1> Z03;

	Z00 << 0, 0, 1;
	Z01 << T01(0, 2), T01(1, 2), T01(2, 2);
	Z02 << T02(0, 2), T02(1, 2), T02(2, 2);
	Z03 << T03(0, 2), T03(1, 2), T03(2, 2);

	/* linear transform based on the 1st coordination (lower part of the jacobian matrix) */
	Eigen::Matrix<float, 3, 1> Jv00 = Z00.cross(P03 - P00);
	Eigen::Matrix<float, 3, 1> Jv01 = Z01.cross(P03 - P01);
	Eigen::Matrix<float, 3, 1> Jv02 = Z02.cross(P03 - P02);

	/* full jacobian matrix (member variable which should be refreshed after arm moves) */
	jacobian_matrix_ << Z00(2, 0), Z01(2, 0), Z02(2, 0),
		Jv00(0, 0), Jv01(0, 0), Jv02(0, 0),
		Jv00(1, 0), Jv01(1, 0), Jv02(1, 0);
}

/** 
 * @param oi - RPY angle of the target orientation (in degree, so it need to be multiplied by Angle2Rad as following)
 * @param pi - target position (in minimeter)
 */
void ScaraArm::TrajectoryPlanning(const float &oz, const float &px, const float &py)
{
	/* used to control the scalar of motor velocity */
	const float velocity_factor = 1.0;
	/* used to control angular (RPY) precision */
	const float angular_threshold = M_PI / 180;
	/* used to control linear (XYZ) precision */
	const float linear_threshold = 0.1;

	/* Convert target position and orientation into Eigen form */
	Eigen::Matrix<float, 3, 1> target_position;
	Eigen::Matrix<float, 3, 1> target_orientation;
	target_position << px, py, 0;
	target_orientation << 0, 0, oz;
	target_orientation *= Angle2Rad;

	/* angular RMS error is to check whether RPY is close enough to target orientation */
	float angular_error = RootMeanSquareError(target_orientation, current_orientation_);
	/* linear RMS error is to check whether XYZ is close enough to target position */
	float linear_error = RootMeanSquareError(target_position, current_position_);

	/* End-effector velocity in Cartesian coordinate */
	Eigen::Matrix<float, 3, 1> linear_velocity;
	Eigen::Matrix<float, 3, 1> angular_velocity;
	linear_velocity.setZero(3, 1);
	angular_velocity.setZero(3, 1);

	int acceleration_counter = 0;
	float acceleration_factor = 0;

	int stop_counter = 0;

	is_working_ = true;

	/* only when all the values of error are under thresholds or when predicted velocity is out of range can the loop be broken */
	while (angular_error > angular_threshold || linear_error > linear_threshold)
	{
		/* first update the jacobian matrix, current orientation and position */
		CalculateJacobianMatrix();

		/* calculate new error based on oriention and position */
		angular_error = RootMeanSquareError(target_orientation, current_orientation_);
		linear_error = RootMeanSquareError(target_position, current_position_);

		/* linear deceleration control */
		angular_velocity = (target_orientation - current_orientation_) * velocity_factor * acceleration_factor;
		linear_velocity = (target_position - current_position_) * velocity_factor * acceleration_factor;

		if (acceleration_factor < 1.0 && acceleration_counter % 10 == 0)
			acceleration_factor += 0.05;

		acceleration_counter++;

		/* end-effector velocity includes angular part (pitch, raw, yaw) in the upper and linear part (spatial position) in the lower */
		Eigen::Matrix<float, 3, 1> end_effector_velocity;
		end_effector_velocity << angular_velocity(2), linear_velocity(0), linear_velocity(1);

		/* find the solution of Jw = V by the function in library <Eigen/LU> */
		Eigen::Matrix<float, 3, 1> motor_velocity;
		motor_velocity = jacobian_matrix_.lu().solve(end_effector_velocity);

		/* Upper bound check */
		motor_velocity = CheckMotorVelocity(motor_velocity);

		if (is_out_of_limit_)
		{
			std::cout << "\t[WARNING] Dangerous velocity! Fail to arrive." << std::endl;

			SetAllMotorsVelocity(0);
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			is_out_of_limit_ = false;
			break;
		}
		else
		{
			/* Lower bound adjustment */
			if (abs(motor_velocity(0, 0) * PRO_RADS2SCALE_) <= 3 &&
				abs(motor_velocity(1, 0) * PRO_RADS2SCALE_) <= 3 &&
				abs(motor_velocity(2, 0) * PRO_RADS2SCALE_) <= 3)
			{
				if (stop_counter < 20)
					stop_counter++;
				else
					break;
			}

			SetArmVelocity(motor_velocity(0, 0),
						   motor_velocity(1, 0),
						   motor_velocity(2, 0));
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	SetAllMotorsVelocity(0);
	std::cout << "\t[INFO] Move is over." << std::endl;
	std::cout << "\t\tFinal position (" << GetCurrentPosition(0) << ", "
			  << GetCurrentPosition(1) << ", "
			  << GetCurrentPosition(2) << ")" << std::endl;

	std::cout << "\t\tFinal orientation (" << GetCurrentOrientation(0) << ", "
			  << GetCurrentOrientation(1) << ", "
			  << GetCurrentOrientation(2) << ")" << std::endl;

	is_working_ = false;
}

float ScaraArm::GetPresentHeight()
{
	return current_screw_height_;
}

/**
 * Read height from file
 */
void ScaraArm::ReadHeight()
{
	// Read Height
	char height[200];
	fstream heightfile;
	string path = string(getenv("PWD")) + "/src/Scara/ScaraArm/Height.txt";
	heightfile.open(path, ios::in);
	if (heightfile.fail())
		std::cout << "[ScaraArm] Cannot open Height.txt" << std::endl;
	else
	{
		heightfile.read(height, sizeof(height));
		heightfile.close();
		current_screw_height_ = stof(height);
	}
}

/**
 * Write height into height.txt
 */
void ScaraArm::WriteHeight(const float &height) const
{
	fstream heightfile;
	string path = string(getenv("PWD")) + "/src/Scara/ScaraArm/Height.txt";
	heightfile.open(path, ios::out);
	if (heightfile.fail())
		std::cout << "[ScaraArm] Cannot open Height.txt" << std::endl;
	else
	{
		heightfile << height;
		heightfile.close();
	}
}

/**
 * Move arm to desire height
 * 
 * @param goal_height - Height we desired to achieve, in mm 
 * @retval - true if height is legal, false if illegal
 */
// Need to check
bool ScaraArm::GoScrewHeight(const float &goal_height)
{
	ReadHeight();
	if (goal_height > 400)
	{
		std::cout << "[ScaraArm] Too high" << std::endl;
		return false;
	}
	else if (goal_height < 20)
	{
		std::cout << "[ScaraArm] Too low" << std::endl;
		return false;
	}
	else if (goal_height == current_screw_height_)
	{
		std::cout << "[ScaraArm] Screw arrival !" << std::endl;
		return true;
	}
	else
	{
		float delta_height = goal_height - current_screw_height_;
		float delta_angle = delta_height / REV_2_SCREW * 360;

		// Motor angle is set to present angle + angle needed
		SetMotor_Angle(SCREW_MOTOR_ID_, delta_angle + GetMotor_PresentAngle(SCREW_MOTOR_ID_));

		// cout << "delta_height: " << delta_height << ", delta_angle: " << delta_angle << endl;
		// cout << "desire angle: " << GetMotor_Angle(FIRST_HAND_ID) << endl;
		this_thread::sleep_for(chrono::milliseconds(500));

		WaitMotorArrival(SCREW_MOTOR_ID_);
		this_thread::sleep_for(chrono::milliseconds(50));

		SetMotor_Velocity(SCREW_MOTOR_ID_, 500);

		WriteHeight(goal_height);
		current_screw_height_ = goal_height;

		// cout << "sp_angle: " << GetMotor_PresentAngle(FIRST_HAND_ID) << endl;
		std::cout << "[ScaraArm] Screw arrival !" << endl;
		return true;
	}
}

/**
 * Move scara arm to initial point
 */
void ScaraArm::Reset()
{
	GoScrewHeight(250.0f);
}