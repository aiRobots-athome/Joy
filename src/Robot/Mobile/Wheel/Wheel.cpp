#include "Wheel.h"

Wheel *Wheel::inst_ = nullptr;
Wheel *Wheel::getWheel()
{
    if (inst_ == nullptr)
        inst_ = new Wheel();
    return inst_;
}

Wheel::Wheel()
    : MotorUnion({4, 5, 6, 7}, {"Pro20+", "Pro20+", "Pro20+", "Pro20+"}),
      wheel_LF(0),
      wheel_RF(1),
      wheel_LB(2),
      wheel_RB(3),
      wheel_gear_ratio(1.8),
      wheel_radius(0.075),
      VEL2METER_MS(GetMotor_Scale2RPM(wheel_LF) * wheel_gear_ratio * (1.667 * 1e-5) * 2 * M_PI * wheel_radius),
      ACCEL2METER_MS2(GetMotor_Scale2RPMM(wheel_LF) * wheel_gear_ratio * pow(1.667 * 1e-5, 2) * 2 * M_PI * wheel_radius)
{
    SetAllMotorsAccel(200);
    SetAllMotorsOperatingMode(1);
    SetAllMotorsTorqueEnable(false);
}

void Wheel::Move(const int &velocity_LF,
                 const int &velocity_RF,
                 const int &velocity_LB,
                 const int &velocity_RB,
                 const float &distance)
{
    const int accel = GetMotor_Accel(wheel_LF);

    const char compare_sign = velocity_LF * velocity_RF < 0 ? -1 : 1;
    const int velocity = (compare_sign * velocity_LF + velocity_RF + compare_sign * velocity_LB + velocity_RB) / 4;

    const int present_velocity_LF = GetMotor_PresentVelocity(wheel_LF);
    const int present_velocity_RF = GetMotor_PresentVelocity(wheel_RF);
    const int present_velocity_LB = GetMotor_PresentVelocity(wheel_LB);
    const int present_velocity_RB = GetMotor_PresentVelocity(wheel_RB);
    const int present_velocity = (compare_sign * present_velocity_LF + present_velocity_RF +
                                  compare_sign * present_velocity_LB + present_velocity_RB) /
                                 4;

    const int rising_time = abs(velocity - present_velocity) * VEL2METER_MS / (accel * ACCEL2METER_MS2);
    const int steady_time =
        (distance - 2 * rising_time * (present_velocity * VEL2METER_MS + accel * ACCEL2METER_MS2 * rising_time / 2)) /
        (abs(velocity) * VEL2METER_MS);

    SetAllMotorsTorqueEnable(true);
    SetMotor_Velocity(wheel_LF, velocity_LF);
    SetMotor_Velocity(wheel_RF, velocity_RF);
    SetMotor_Velocity(wheel_LB, velocity_LB);
    SetMotor_Velocity(wheel_RB, velocity_RB);

    if (distance != 0)
    {
        WaitAllMotorsArrival(steady_time + rising_time);
        SetAllMotorsVelocity(0);
        WaitAllMotorsArrival(rising_time);
        SetAllMotorsTorqueEnable(false);
    }
}

void Wheel::Move(const int &velocity, const float &distance)
{
    Move(-velocity, velocity, -velocity, velocity, distance);
}

void Wheel::MoveForward(const int &velocity, const float &distance)
{
    Move(abs(velocity), distance);
}

void Wheel::MoveBackward(const int &velocity, const float &distance)
{
    Move(-abs(velocity), distance);
}

void Wheel::Stop()
{
    SetAllMotorsTorqueEnable(false);
}

void Wheel::MoveByConstant(const int &velocity)
{
    SetMotor_Velocity(wheel_LF, -velocity);
    SetMotor_Velocity(wheel_RF, +velocity);
    SetMotor_Velocity(wheel_LB, -velocity);
    SetMotor_Velocity(wheel_RB, +velocity);
}

void Wheel::SelfTurnByConstant(const int &velocity)
{
    SetMotor_Velocity(wheel_LF, +velocity);
    SetMotor_Velocity(wheel_RF, +velocity);
    SetMotor_Velocity(wheel_LB, +velocity);
    SetMotor_Velocity(wheel_RB, +velocity);
}

void Wheel::LockOn(void)
{
    // SetMotor_TorqueEnable(wheel_LB, true);
    // SetMotor_TorqueEnable(wheel_RB, true);
    SetAllMotorsTorqueEnable(true);
}

void Wheel::Wait(void)
{
    while(true)
    {
        std::cout << GetMotor_PresentVelocity(wheel_LF) << std::endl;
        if (abs(GetMotor_PresentVelocity(wheel_LF)) < 5)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}