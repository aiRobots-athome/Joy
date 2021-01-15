#include "SaleArmRight.h"

SaleArmRight *SaleArmRight::inst_ = nullptr;
SaleArmRight *SaleArmRight::getSaleArmRight()
{
	if (inst_ == nullptr)
		inst_ = new SaleArmRight();
	return inst_;
}

SaleArmRight::SaleArmRight()
	: Arm({23, 24, 25, 26, 27, 28, 29}, {"Pro200", "Pro200", "Pro200", "Pro20", "Pro100", "Pro20", "Pro20"})
{
    SHOULDER_LINK_LENGTH_ = 120;
    UPPER_LINK_LENGTH_ = -258;
    LOWER_LINK_LENGTH_ = 260;
    END_EFFECTOR_LENGTH_ = 190;

    CalculateJacobianMatrix();
	cout << "\tClass constructed: SaleArmRight" << endl;
}

/**
 * This function is in charge of only the calculation process of jacobian matrix of 6 motors.
 * Variable jacobian_matrix_ is the member variable of class Arm, which will be used to solve the solution of angular velocity of 6 motors.
 * 
 * Because there will be infinite solutions under conditions of 7 dof, here we fix the 0th motor and build the jacobian for next 6 motors.
 * Thus the positions and orientations will be based on 1st coordination.
 *  
 * @retval void
 * 
 * @result Arm::Eigen::Matrix<float, 3, 1> current_orientation_
 * @result Arm::Eigen::Matrix<float, 3, 1> current_position_
 * @result Arm::Eigen::Matrix<float, 6, 6> jacobian_matrix_
 */
void SaleArmRight::CalculateJacobianMatrix()
{
	/* get motor delta angle*/
	float J0 = GetMotor_PresentAngle(FIRST_SHOULDER_ID_) * Angle2Rad;
	float J1 = GetMotor_PresentAngle(FIRST_HAND_ID_ + 0) * Angle2Rad;
	float J2 = GetMotor_PresentAngle(FIRST_HAND_ID_ + 1) * Angle2Rad;
	float J3 = GetMotor_PresentAngle(FIRST_HAND_ID_ + 2) * Angle2Rad;
	float J4 = GetMotor_PresentAngle(FIRST_HAND_ID_ + 3) * Angle2Rad;
	float J5 = GetMotor_PresentAngle(FIRST_HAND_ID_ + 4) * Angle2Rad;
	float J6 = GetMotor_PresentAngle(FIRST_HAND_ID_ + 5) * Angle2Rad;

	/* basic transform matrix from i-1 to i */
	/* the sign of delta angle is decided by the motor setting direction and the definition of coordination */
	Eigen::Matrix<float, 4, 4> T01 = GetTransformMatrix(-J0, M_PI_2, 0, 0);
	Eigen::Matrix<float, 4, 4> T12 = GetTransformMatrix(+J1 - M_PI_2, M_PI_2, 0, SHOULDER_LINK_LENGTH_);
	Eigen::Matrix<float, 4, 4> T23 = GetTransformMatrix(-J2 + M_PI_2, -M_PI_2, 0, 0);
	Eigen::Matrix<float, 4, 4> T34 = GetTransformMatrix(-J3 + M_PI_2, M_PI_2, 0, UPPER_LINK_LENGTH_);
	Eigen::Matrix<float, 4, 4> T45 = GetTransformMatrix(+J4, M_PI_2, 0, 0);
	Eigen::Matrix<float, 4, 4> T56 = GetTransformMatrix(+J5, M_PI_2, 0, LOWER_LINK_LENGTH_);
	Eigen::Matrix<float, 4, 4> T67 = GetTransformMatrix(+J6 + M_PI_2, M_PI_2, END_EFFECTOR_LENGTH_, 0);

	/* transform matrix based on the 1st coordination */
	Eigen::Matrix<float, 4, 4> T02 = T01 * T12;
	Eigen::Matrix<float, 4, 4> T03 = T02 * T23;
	Eigen::Matrix<float, 4, 4> T04 = T03 * T34;
	Eigen::Matrix<float, 4, 4> T05 = T04 * T45;
	Eigen::Matrix<float, 4, 4> T06 = T05 * T56;
	Eigen::Matrix<float, 4, 4> T07 = T06 * T67;

	/* end effector oreientation */
	float oz = atan2(T07(1, 0), T07(0, 0));
	float oy = atan2(-T07(2, 0), (T07(0, 0) * cos(oz) + T07(1, 0) * sin(oz)));
	float ox = atan2((T07(0, 2) * sin(oz) - T07(1, 2) * cos(oz)), (T07(1, 1) * cos(oz) - T07(0, 1) * sin(oz)));

	Arm::current_orientation_ << ox, oy, oz;

	/* positions based on the 1st coordination */
	Eigen::Matrix<float, 3, 1> P00;
	Eigen::Matrix<float, 3, 1> P01;
	Eigen::Matrix<float, 3, 1> P02;
	Eigen::Matrix<float, 3, 1> P03;
	Eigen::Matrix<float, 3, 1> P04;
	Eigen::Matrix<float, 3, 1> P05;
	Eigen::Matrix<float, 3, 1> P06;
	Eigen::Matrix<float, 3, 1> P07;

	P00 << 0, 0, 0;
	P01 << T01(0, 3), T01(1, 3), T01(2, 3);
	P02 << T02(0, 3), T02(1, 3), T02(2, 3);
	P03 << T03(0, 3), T03(1, 3), T03(2, 3);
	P04 << T04(0, 3), T04(1, 3), T04(2, 3);
	P05 << T05(0, 3), T05(1, 3), T05(2, 3);
	P06 << T06(0, 3), T06(1, 3), T06(2, 3);
	P07 << T07(0, 3), T07(1, 3), T07(2, 3);

	Arm::current_position_ = P07;

	/* rotation based on the 1st coordination (upper part of the jacobian matrix) */
	Eigen::Matrix<float, 3, 1> Z00;
	Eigen::Matrix<float, 3, 1> Z01;
	Eigen::Matrix<float, 3, 1> Z02;
	Eigen::Matrix<float, 3, 1> Z03;
	Eigen::Matrix<float, 3, 1> Z04;
	Eigen::Matrix<float, 3, 1> Z05;
	Eigen::Matrix<float, 3, 1> Z06;

	Z00 << 0, 0, 1;
	Z01 << T01(0, 2), T01(1, 2), T01(2, 2);
	Z02 << T02(0, 2), T02(1, 2), T02(2, 2);
	Z03 << T03(0, 2), T03(1, 2), T03(2, 2);
	Z04 << T04(0, 2), T04(1, 2), T04(2, 2);
	Z05 << T05(0, 2), T05(1, 2), T05(2, 2);
	Z06 << T06(0, 2), T06(1, 2), T06(2, 2);

	/* linear transform based on the 1st coordination (lower part of the jacobian matrix) */
	Eigen::Matrix<float, 3, 1> Jv01 = Z01.cross(P07 - P01);
	Eigen::Matrix<float, 3, 1> Jv02 = Z02.cross(P07 - P02);
	Eigen::Matrix<float, 3, 1> Jv03 = Z03.cross(P07 - P03);
	Eigen::Matrix<float, 3, 1> Jv04 = Z04.cross(P07 - P04);
	Eigen::Matrix<float, 3, 1> Jv05 = Z05.cross(P07 - P05);
	Eigen::Matrix<float, 3, 1> Jv06 = Z06.cross(P07 - P06);

	/* full jacobian matrix (member variable which should be refreshed after arm moves) */
	Arm::jacobian_matrix_ << Z01(0, 0), Z02(0, 0), Z03(0, 0), Z04(0, 0), Z05(0, 0), Z06(0, 0),
		Z01(1, 0), Z02(1, 0), Z03(1, 0), Z04(1, 0), Z05(1, 0), Z06(1, 0),
		Z01(2, 0), Z02(2, 0), Z03(2, 0), Z04(2, 0), Z05(2, 0), Z06(2, 0),
		Jv01(0, 0), Jv02(0, 0), Jv03(0, 0), Jv04(0, 0), Jv05(0, 0), Jv06(0, 0),
		Jv01(1, 0), Jv02(1, 0), Jv03(1, 0), Jv04(1, 0), Jv05(1, 0), Jv06(1, 0),
		Jv01(2, 0), Jv02(2, 0), Jv03(2, 0), Jv04(2, 0), Jv05(2, 0), Jv06(2, 0);
}

/**
 * This function is set to be PUBLIC in order to be used in strategy.
 * 
 * @param vi - target velocity of motors from 0th to 6th 
 */
void SaleArmRight::SetArmVelocity(float v0, float v1, float v2, float v3, float v4, float v5, float v6)
{
	SetMotor_Velocity(FIRST_SHOULDER_ID_, +v0 * pro_radpersec2scale_);
	SetMotor_Velocity(FIRST_HAND_ID_ + 0, +v1 * pro_radpersec2scale_);
	SetMotor_Velocity(FIRST_HAND_ID_ + 1, -v2 * pro_radpersec2scale_);
	SetMotor_Velocity(FIRST_HAND_ID_ + 2, -v3 * pro_radpersec2scale_);
	SetMotor_Velocity(FIRST_HAND_ID_ + 3, +v4 * pro_radpersec2scale_);
	SetMotor_Velocity(FIRST_HAND_ID_ + 4, +v5 * pro_radpersec2scale_);
	SetMotor_Velocity(FIRST_HAND_ID_ + 5, +v6 * pro_radpersec2scale_);
}

/**
 * This function basically implements the new version of 6 DOF (or called the fake 7 DOF) GotoPosition in velocity mode,
 * in which the end effector angular velocities (RPY) and linear velocities (XYZ) are transformed into angular velocities of 6 motors by jacobian matrix. 
 * 
 * Jacobian matrix can only actually solve the solutions of 6 DOF arm,
 * but with the dynamically end effector data, the movement of 0th motor can be combined with the 6 DOF and be done together.
 * 
 * @param J0 - target angle of the 0th motor (in degree)
 * @param oi - RPY angle of the target orientation (in degree, so it need to be multiplied by Angle2Rad as following)
 * @param pi - target position (in minimeter)
 */
void SaleArmRight::TrajectoryPlanning(const float &J0, const float &ox, const float &oy, const float &oz, const float &px, const float &py, const float &pz)
{
	/* used to control the scalar of motor velocity */
    const float velocity_factor = 1.0;
    /* used to control angular (RPY) precision */
    const float angular_threshold = M_PI / 180;
    /* used to control linear (XYZ) precision */
    const float linear_threshold = 0.1;
    /* used to control zeroth motor angle precision */
    const float zeroth_joint_threshold = M_PI / 180;

    Eigen::Matrix<float, 3, 1> target_position;
    Eigen::Matrix<float, 3, 1> target_orientation;
    target_position << px, py, pz;
    target_orientation << ox, oy, oz;
    target_orientation *= Angle2Rad;

    /* angular RMS error is to check whether RPY is close enough to target orientation */
    float angular_error = RootMeanSquareError(target_orientation, current_orientation_);

    /* linear RMS error is to check whether XYZ is close enough to target position */
    float linear_error = RootMeanSquareError(target_position, current_position_);

    /* 0th joint error */
    float zeroth_joint_error = (J0 - GetMotor_PresentAngle(FIRST_SHOULDER_ID_)) * Angle2Rad;

    Eigen::Matrix<float, 3, 1> angular_velocity;
    Eigen::Matrix<float, 3, 1> linear_velocity;
    angular_velocity.setZero(3, 1);
    linear_velocity.setZero(3, 1);

    float velocity_zeroth_joint = 0.0;

    /* simple acceleration control, this factor will increase to 1.0 as the counter increases */
    float acceleration_factor = 0.1;
    int counter = 0;

    is_working_ = true;

    /* only when all the values of error are under thresholds or when predicted velocity is out of range can the loop be broken */
    while (angular_error > angular_threshold || linear_error > linear_threshold || abs(zeroth_joint_error) > zeroth_joint_threshold)
    {
        /* first update the jacobian matrix, current orientation and position */
        CalculateJacobianMatrix();

        /* calculate new error based on oriention and position */
        angular_error = RootMeanSquareError(target_orientation, current_orientation_);
        linear_error = RootMeanSquareError(target_position, current_position_);
        zeroth_joint_error = (J0 - GetMotor_PresentAngle(FIRST_SHOULDER_ID_)) * Angle2Rad;

        /* linear acceleration control */
        if (acceleration_factor < 1 && counter % 20 == 0)
            acceleration_factor += 0.1;

        counter++;

        /* linear deceleration control */
        angular_velocity = (target_orientation - current_orientation_) * velocity_factor * acceleration_factor;
        linear_velocity = (target_position - current_position_) * velocity_factor * acceleration_factor;
        velocity_zeroth_joint = zeroth_joint_error * acceleration_factor;

        /* end-effector velocity includes angular part (pitch, raw, yaw) in the upper and linear part (spatial position) in the lower */
        Eigen::Matrix<float, 6, 1> end_effector_velocity;
        end_effector_velocity << angular_velocity, linear_velocity;

        /* find the solution of Jw = V by the function in library <Eigen/LU> */
        Eigen::Matrix<float, 6, 1> motor_velocity;
        motor_velocity = jacobian_matrix_.lu().solve(end_effector_velocity);        
        motor_velocity = CheckMotorVelocity(motor_velocity);

        if (abs(velocity_zeroth_joint) >= (M_PI / 12))
            velocity_zeroth_joint = Sign(velocity_zeroth_joint) * (M_PI / 12);

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
            /* after upper limit control, we found that motors may stop moving as they are very close to ther target point. */
            /* so here are some lower limit solution to handle the problem */
            if (abs(motor_velocity(0, 0) * pro_radpersec2scale_) <= 1 &&
                abs(motor_velocity(1, 0) * pro_radpersec2scale_) <= 1 &&
                abs(motor_velocity(2, 0) * pro_radpersec2scale_) <= 1 &&
                abs(motor_velocity(3, 0) * pro_radpersec2scale_) <= 1 &&
                abs(motor_velocity(4, 0) * pro_radpersec2scale_) <= 1 &&
                abs(motor_velocity(5, 0) * pro_radpersec2scale_) <= 1)
            {
                motor_velocity *= 10;
            }

            /* set motor speed accroding to the solution */
            /* again, the sign of velocity is accroding to the installation of motor and the definition of coordination */
            /* for the right arm in current version, it should be (+, +, -, -, +, +, +) */
            SetArmVelocity(velocity_zeroth_joint,
                        motor_velocity(0, 0),
                        motor_velocity(1, 0),
                        motor_velocity(2, 0),
                        motor_velocity(3, 0),
                        motor_velocity(4, 0),
                        motor_velocity(5, 0));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(int(delay_time_)));
    }
    SetAllMotorsVelocity(0);
    is_working_ = false;
    std::cout << "\t[INFO] Move is over." << std::endl;
    std::cout << "\t\tFinal position (" << GetCurrentPosition(0) << ", "
              << GetCurrentPosition(1) << ", "
              << GetCurrentPosition(2) << ")" << std::endl;

    std::cout << "\t\tFinal orientation (" << GetCurrentOrientation(0) << ", "
              << GetCurrentOrientation(1) << ", "
              << GetCurrentOrientation(2) << ")" << std::endl;
}

void SaleArmRight::PneumaticOn()
{
	const char *port_name = "/dev/ttyACM0";
    Arm::port_file_ = fopen(port_name, "w");
    fprintf(port_file_, "%d", 3);
    fclose(port_file_);
	std::cout << "\t[INFO] Right Pneumatic ON." << std::endl;
}
void SaleArmRight::PneumaticOff()
{
	const char *port_name = "/dev/ttyACM0";
    Arm::port_file_ = fopen(port_name, "w");
    fprintf(port_file_, "%d", 2);
    fclose(port_file_);
	std::cout << "\t[INFO] Right Pneumatic OFF." << std::endl;
}