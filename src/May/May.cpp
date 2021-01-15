#include "May.h"

May *May::inst_ = nullptr;
May *May::getMay()
{
    if (inst_ == nullptr)
        inst_ = new May();
    return inst_;
}

May::May()
    : MotorUnion({40, 41, 42, 43, 44, 45}, {"Pro200", "Pro200", "Pro200", "Pro100", "Pro20", "Pro20+"}),
      pro_radpersec2scale_(60.0 / (2 * M_PI) / 0.01)
{
    END_EFFECTOR_LENGTH_ = 200;
    is_out_of_limit_ = false;
    is_working_ = false;

    Start();

    std::cout << "Class constructed: May" << std::endl;
    std::cout << "================================================================================" << std::endl;
}

void May::Start()
{
    SetAllMotorsOperatingMode(1);
    SetAllMotorsVelocity(0);
    SetAllMotorsAccel(1000);
    SetAllMotorsTorqueEnable(true);
}

void May::Stop()
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
Eigen::Matrix<float, 4, 4> May::GetTransformMatrix(const float &theta, const float &alpha, const float &a, const float &d)
{
    Eigen::Matrix<float, 4, 4> transform_matrix;
    transform_matrix << cos(theta), -cos(alpha) * sin(theta), sin(alpha) * sin(theta), a * cos(theta),
        sin(theta), cos(alpha) * cos(theta), -sin(alpha) * cos(theta), a * sin(theta),
        0, sin(alpha), cos(alpha), d,
        0, 0, 0, 1;

    return transform_matrix;
}

Eigen::Matrix<float, 3, 3> May::GetRotationMatrix(const int &axis_index, const float &theta)
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

float May::GetCurrentPosition(int index)
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

float May::GetCurrentOrientation(int index)
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

bool May::GetWorkingState() { return is_working_; }

Eigen::Matrix<float, 6, 6> May::GetJacobianMatrix(void) { return jacobian_matrix_; }

int May::Sign(float x) { return (x >= 0 ? 1 : -1); }

float May::RootMeanSquareError(Eigen::Matrix<float, 3, 1> target_data, Eigen::Matrix<float, 3, 1> current_data)
{
    Eigen::Matrix<float, 3, 1> square = (target_data - current_data).array().square();
    float result = (square(0) + square(1) + square(2)) / 3;

    return sqrt(result);
}

void May::CalculateJacobianMatrix()
{
    /* get motor delta angle*/
    float J0 = GetMotor_PresentAngle(0) * Angle2Rad;
    float J1 = GetMotor_PresentAngle(1) * Angle2Rad;
    float J2 = GetMotor_PresentAngle(2) * Angle2Rad;
    float J3 = GetMotor_PresentAngle(3) * Angle2Rad;
    float J4 = GetMotor_PresentAngle(4) * Angle2Rad;
    float J5 = GetMotor_PresentAngle(5) * Angle2Rad;

    /* basic transform matrix from i-1 to i */
    /* the sign of delta angle is decided by the motor setting direction and the definition of coordination */
    Eigen::Matrix<float, 4, 4> T01 = GetTransformMatrix(J0 + (18 * Angle2Rad), -M_PI_2, 0, 159);
    Eigen::Matrix<float, 4, 4> T12 = GetTransformMatrix(J1 - M_PI_2, 0, 531, 0);
    Eigen::Matrix<float, 4, 4> T23 = GetTransformMatrix(J2 + M_PI_2, -M_PI_2, 87, 0);
    Eigen::Matrix<float, 4, 4> T34 = GetTransformMatrix(J3, M_PI_2, 0, 376);
    Eigen::Matrix<float, 4, 4> T45 = GetTransformMatrix(J4, -M_PI_2, 0, 0);
    Eigen::Matrix<float, 4, 4> T56 = GetTransformMatrix(J5, M_PI, 0, END_EFFECTOR_LENGTH_);

    /* transform matrix based on the 1st coordination */
    Eigen::Matrix<float, 4, 4> T02 = T01 * T12;
    Eigen::Matrix<float, 4, 4> T03 = T02 * T23;
    Eigen::Matrix<float, 4, 4> T04 = T03 * T34;
    Eigen::Matrix<float, 4, 4> T05 = T04 * T45;
    Eigen::Matrix<float, 4, 4> T06 = T05 * T56;

    /* end effector oreientation */
    float oz = atan2(T06(1, 0), T06(0, 0));
    float oy = atan2(-T06(2, 0), (T06(0, 0) * cos(oz) + T06(1, 0) * sin(oz)));
    float ox = atan2((T06(0, 2) * sin(oz) - T06(1, 2) * cos(oz)), (T06(1, 1) * cos(oz) - T06(0, 1) * sin(oz)));

    current_orientation_ << ox, oy, oz;

    /* positions based on the 1st coordination */
    Eigen::Matrix<float, 3, 1> P00;
    Eigen::Matrix<float, 3, 1> P01;
    Eigen::Matrix<float, 3, 1> P02;
    Eigen::Matrix<float, 3, 1> P03;
    Eigen::Matrix<float, 3, 1> P04;
    Eigen::Matrix<float, 3, 1> P05;
    Eigen::Matrix<float, 3, 1> P06;

    P00 << 0, 0, 0;
    P01 << T01(0, 3), T01(1, 3), T01(2, 3);
    P02 << T02(0, 3), T02(1, 3), T02(2, 3);
    P03 << T03(0, 3), T03(1, 3), T03(2, 3);
    P04 << T04(0, 3), T04(1, 3), T04(2, 3);
    P05 << T05(0, 3), T05(1, 3), T05(2, 3);
    P06 << T06(0, 3), T06(1, 3), T06(2, 3);

    current_position_ = P06;

    /* rotation based on the 1st coordination (upper part of the jacobian matrix) */
    Eigen::Matrix<float, 3, 1> Z00;
    Eigen::Matrix<float, 3, 1> Z01;
    Eigen::Matrix<float, 3, 1> Z02;
    Eigen::Matrix<float, 3, 1> Z03;
    Eigen::Matrix<float, 3, 1> Z04;
    Eigen::Matrix<float, 3, 1> Z05;

    Z00 << 0, 0, 1;
    Z01 << T01(0, 2), T01(1, 2), T01(2, 2);
    Z02 << T02(0, 2), T02(1, 2), T02(2, 2);
    Z03 << T03(0, 2), T03(1, 2), T03(2, 2);
    Z04 << T04(0, 2), T04(1, 2), T04(2, 2);
    Z05 << T05(0, 2), T05(1, 2), T05(2, 2);

    /* linear transform based on the 1st coordination (lower part of the jacobian matrix) */
    Eigen::Matrix<float, 3, 1> Jv00 = Z00.cross(P06 - P00);
    Eigen::Matrix<float, 3, 1> Jv01 = Z01.cross(P06 - P01);
    Eigen::Matrix<float, 3, 1> Jv02 = Z02.cross(P06 - P02);
    Eigen::Matrix<float, 3, 1> Jv03 = Z03.cross(P06 - P03);
    Eigen::Matrix<float, 3, 1> Jv04 = Z04.cross(P06 - P04);
    Eigen::Matrix<float, 3, 1> Jv05 = Z05.cross(P06 - P05);

    /* full jacobian matrix (member variable which should be refreshed after arm moves) */
    jacobian_matrix_ << Z00(0, 0), Z01(0, 0), Z02(0, 0), Z03(0, 0), Z04(0, 0), Z05(0, 0),
        Z00(1, 0), Z01(1, 0), Z02(1, 0), Z03(1, 0), Z04(1, 0), Z05(1, 0),
        Z00(2, 0), Z01(2, 0), Z02(2, 0), Z03(2, 0), Z04(2, 0), Z05(2, 0),
        Jv00(0, 0), Jv01(0, 0), Jv02(0, 0), Jv03(0, 0), Jv04(0, 0), Jv05(0, 0),
        Jv00(1, 0), Jv01(1, 0), Jv02(1, 0), Jv03(1, 0), Jv04(1, 0), Jv05(1, 0),
        Jv00(2, 0), Jv01(2, 0), Jv02(2, 0), Jv03(2, 0), Jv04(2, 0), Jv05(2, 0);
}

/**
 * This function is set to be PUBLIC in order to be used in strategy.
 * 
 * @param vi - target velocity of motors from 0th to 6th 
 */
void May::SetArmVelocity(float v0, float v1, float v2, float v3, float v4, float v5)
{
    SetMotor_Velocity(0, v0 * pro_radpersec2scale_);
    SetMotor_Velocity(1, v1 * pro_radpersec2scale_);
    SetMotor_Velocity(2, v2 * pro_radpersec2scale_);
    SetMotor_Velocity(3, v3 * pro_radpersec2scale_);
    SetMotor_Velocity(4, v4 * pro_radpersec2scale_);
    SetMotor_Velocity(5, v5 * pro_radpersec2scale_);
}

Eigen::Matrix<float, 6, 1> May::CheckMotorVelocity(Eigen::Matrix<float, 6, 1> motor_velocity)
{
    /* Determine the index of max velocity */
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
        if (max_speed >= (M_PI / 3))
            motor_velocity *= abs((M_PI / 3) / motor_velocity(max_index, 0));
    }
    return motor_velocity;
}

/** 
 * @param oi - RPY angle of the target orientation (in degree, so it need to be multiplied by Angle2Rad as following)
 * @param pi - target position (in minimeter)
 */
void May::TrajectoryPlanning(const float &ox, const float &oy, const float &oz, const float &px, const float &py, const float &pz)
{
    /* used to control the scalar of motor velocity */
    const float velocity_factor = 1.6;
    /* used to control angular (RPY) precision */
    const float angular_threshold = M_PI / 180;
    /* used to control linear (XYZ) precision */
    const float linear_threshold = 0.1;

    /* Convert target position and orientation into Eigen form */
    Eigen::Matrix<float, 3, 1> target_position;
    Eigen::Matrix<float, 3, 1> target_orientation;
    target_position << px, py, pz;
    target_orientation << ox, oy, oz;
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

        if (acceleration_factor < 1.0 && acceleration_counter % 20 == 0)
            acceleration_factor += 0.1;

        acceleration_counter++;

        /* end-effector velocity includes angular part (pitch, raw, yaw) in the upper and linear part (spatial position) in the lower */
        Eigen::Matrix<float, 6, 1> end_effector_velocity;
        end_effector_velocity << angular_velocity, linear_velocity;

        /* find the solution of Jw = V by the function in library <Eigen/LU> */
        Eigen::Matrix<float, 6, 1> motor_velocity;
        motor_velocity = jacobian_matrix_.lu().solve(end_effector_velocity);

        /* Upper bound check */
        motor_velocity = CheckMotorVelocity(motor_velocity);

        if (is_out_of_limit_)
        {
            std::cout << "\t[WARNING] Dangerous velocity! Fail to arrive." << std::endl;
            std::cout << GetCurrentPosition(0) << "\t"
                      << GetCurrentPosition(1) << "\t"
                      << GetCurrentPosition(2) << "\t"
                      << GetCurrentOrientation(0) << "\t"
                      << GetCurrentOrientation(1) << "\t"
                      << GetCurrentOrientation(2) << std::endl;

            SetAllMotorsVelocity(0);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            is_out_of_limit_ = false;
            break;
        }
        else
        {
            /* Lower bound adjustment */
            if (abs(motor_velocity(0, 0) * pro_radpersec2scale_) <= 3 &&
                abs(motor_velocity(1, 0) * pro_radpersec2scale_) <= 3 &&
                abs(motor_velocity(2, 0) * pro_radpersec2scale_) <= 3 &&
                abs(motor_velocity(3, 0) * pro_radpersec2scale_) <= 3 &&
                abs(motor_velocity(4, 0) * pro_radpersec2scale_) <= 3 &&
                abs(motor_velocity(5, 0) * pro_radpersec2scale_) <= 3)
            {
                if (stop_counter < 20)
                    stop_counter++;
                else
                    break;                
            }

            SetArmVelocity(motor_velocity(0, 0),
                           motor_velocity(1, 0),
                           motor_velocity(2, 0),
                           motor_velocity(3, 0),
                           motor_velocity(4, 0),
                           motor_velocity(5, 0));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
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

void May::ImpedanceControl(const float &om, const float &ob, const float &ok, const float &pm, const float &pb, const float &pk,
                           const float &ox, const float &oy, const float &oz, const float &px, const float &py, const float &pz)
{
    /* Define the coefficients of Motion(M), Damping(B) and Stiffness(K) on different axis xyz */
    /* LINEAR */
    Eigen::Matrix<float, 3, 3> M_linear = Eigen::Matrix3f::Identity(3, 3);
    M_linear *= pm;

    Eigen::Matrix<float, 3, 3> B_linear = Eigen::Matrix3f::Identity(3, 3);
    B_linear *= pb;

    Eigen::Matrix<float, 3, 3> K_linear = Eigen::Matrix3f::Identity(3, 3);
    K_linear *= pk;

    /* ANGULAR */
    Eigen::Matrix<float, 3, 3> M_angular = Eigen::Matrix3f::Identity(3, 3);
    M_angular *= om;

    Eigen::Matrix<float, 3, 3> B_angular = Eigen::Matrix3f::Identity(3, 3);
    B_angular *= ob;

    Eigen::Matrix<float, 3, 3> K_angular = Eigen::Matrix3f::Identity(3, 3);
    K_angular *= ok;

    float time_step = 0.005;

    Eigen::Matrix<float, 3, 1> target_position;
    Eigen::Matrix<float, 3, 1> target_orientation;
    target_position << px, py, pz;
    target_orientation << ox, oy, oz;
    target_orientation = target_orientation * Angle2Rad;

    /* 
     * For following two velocity matrix, col defines time step and row defines the axis
     * It should look like:
     * 
     * Vx_t1, Vx_t2, Vx_t3
     * Vy_t1, Vy_t2, Vy_t3
     * Vz_t1, Vz_t2, Vz_t3 
     */
    Eigen::Matrix3f linear_velocity = Eigen::Matrix3f::Zero(3, 3);
    Eigen::Matrix3f angular_velocity = Eigen::Matrix3f::Zero(3, 3);

    /* used to control angular (RPY) precision */
    const float angular_threshold = M_PI / 360;
    /* used to control linear (XYZ) precision */
    const float linear_threshold = 0.1;

    /* angular RMS error is to check whether RPY is close enough to target orientation */
    float angular_error = RootMeanSquareError(target_orientation, current_orientation_);
    /* linear RMS error is to check whether XYZ is close enough to target position */
    float linear_error = RootMeanSquareError(target_position, current_position_);

    is_working_ = true;

    while (angular_error > angular_threshold || linear_error > linear_threshold)
    {
        CalculateJacobianMatrix();

        angular_error = RootMeanSquareError(target_orientation, current_orientation_);
        linear_error = RootMeanSquareError(target_position, current_position_);

        /* Calculate linear velocity in t1, t2 and t3 */
        linear_velocity.col(2) = B_linear.lu().solve(-M_linear * (linear_velocity.col(1) - linear_velocity.col(0)) / time_step -
                                                     K_linear * (current_position_ - target_position));
        linear_velocity.col(0) = linear_velocity.col(1);
        linear_velocity.col(1) = linear_velocity.col(2);

        /* Calculate angular velocity in t1, t2 and t3 */
        angular_velocity.col(2) = B_angular.lu().solve(-M_angular * (angular_velocity.col(1) - angular_velocity.col(0)) / time_step -
                                                       K_angular * (current_orientation_ - target_orientation));
        angular_velocity.col(0) = angular_velocity.col(1);
        angular_velocity.col(1) = angular_velocity.col(2);

        Eigen::Matrix<float, 6, 1> end_effector_velocity;

        end_effector_velocity << angular_velocity(0, 2),
            angular_velocity(1, 2),
            angular_velocity(2, 2),
            linear_velocity(0, 2),
            linear_velocity(1, 2),
            linear_velocity(2, 2);

        Eigen::Matrix<float, 6, 1> motor_velocity;
        motor_velocity = jacobian_matrix_.lu().solve(end_effector_velocity);

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
            if (abs(motor_velocity(0, 0) * pro_radpersec2scale_) <= 1 &&
                abs(motor_velocity(1, 0) * pro_radpersec2scale_) <= 1 &&
                abs(motor_velocity(2, 0) * pro_radpersec2scale_) <= 1 &&
                abs(motor_velocity(3, 0) * pro_radpersec2scale_) <= 1 &&
                abs(motor_velocity(4, 0) * pro_radpersec2scale_) <= 1 &&
                abs(motor_velocity(5, 0) * pro_radpersec2scale_) <= 1)
            {
                motor_velocity *= 10;
            }

            SetArmVelocity(motor_velocity(0, 0),
                           motor_velocity(1, 0),
                           motor_velocity(2, 0),
                           motor_velocity(3, 0),
                           motor_velocity(4, 0),
                           motor_velocity(5, 0));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(int(time_step * 1000)));
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

void May::PneumaticOn()
{
    const char *port_name = "/dev/ttyACM0";
    port_file_ = fopen(port_name, "w");
    fprintf(port_file_, "%d", 0);
    fclose(port_file_);
    std::cout << "\t[INFO] Pneumatic ON." << std::endl;
}
void May::PneumaticOff()
{
    const char *port_name = "/dev/ttyACM0";
    port_file_ = fopen(port_name, "w");
    fprintf(port_file_, "%d", 1);
    fclose(port_file_);
    std::cout << "\t[INFO] Pneumatic OFF." << std::endl;
}