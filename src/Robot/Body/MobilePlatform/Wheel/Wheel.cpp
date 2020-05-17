#include "Wheel.h"
Wheel::Wheel(const vector<unsigned char> &IDArray,
             const vector<string> &MotorModelArray,
             vector<unsigned char> &AllPortNumber)
    : MotorUnion(IDArray, MotorModelArray, AllPortNumber),
      wheel_LF(0),
      wheel_RF(1),
      wheel_LB(2),
      wheel_RB(3),
      wheel_gear_ratio(1.8),
      wheel_radius(0.075),
      scale2meter_ms(GetMotor_Scale2RPM(wheel_LF) * wheel_gear_ratio * (1.667 * 1e-5) * 2 * M_PI * wheel_radius),
      phase(50)
{
    SetAllMotorsTorqueEnable(false);
    delay = 0;
    softstart_distance = 0.0f;
}

void Wheel::Move(const int &velocity)
{
    SoftStart(-velocity, velocity, -velocity, velocity);
}

void Wheel::MoveForward(const int &velocity)
{
    Move(abs(velocity));
}

void Wheel::MoveBackward(const int &velocity)
{
    Move(-abs(velocity));
}

void Wheel::Move(const int &velocity_LF, const int &velocity_RF, const int &velocity_LB, const int &velocity_RB)
{
    SoftStart(velocity_LF, velocity_RF, velocity_LB, velocity_RB);
}

void Wheel::Stop()
{
    Move(0);
    SetAllMotorsTorqueEnable(false);
}

const int Wheel::CalculateDistanceTime(const float &distance, const int &velocity)
{
    return (abs(distance) - 2 * softstart_distance) / (abs(velocity) * scale2meter_ms);
}

void Wheel::Wait(const int waiting_time_ms) const
{
    WaitAllMotorsArrival(waiting_time_ms);
}

void Wheel::SwitchTorqueEnable(const int &velocity_LF, const int &velocity_RF, const int &velocity_LB, const int &velocity_RB)
{
    if (velocity_LF != 0)
        SetMotor_TorqueEnable(wheel_LF, true);

    if (velocity_RF != 0)
        SetMotor_TorqueEnable(wheel_RF, true);

    if (velocity_LB != 0)
        SetMotor_TorqueEnable(wheel_LB, true);

    if (velocity_RB != 0)
        SetMotor_TorqueEnable(wheel_RB, true);
}

void Wheel::SoftStart(const int &velocity_LF, const int &velocity_RF, const int &velocity_LB, const int &velocity_RB)
{
    // once set velocity, turn on torque enable
    SwitchTorqueEnable(velocity_LF, velocity_RF, velocity_LB, velocity_RB);

    int present_velocity_LF = GetMotor_PresentVelocity(wheel_LF);
    int present_velocity_RF = GetMotor_PresentVelocity(wheel_RF);
    int present_velocity_LB = GetMotor_PresentVelocity(wheel_LB);
    int present_velocity_RB = GetMotor_PresentVelocity(wheel_RB);
    const char compare_sign = velocity_LF * velocity_RF < 0 ? -1 : 1;
    const int average_present_velocity = (compare_sign * present_velocity_LF + present_velocity_RF + compare_sign * present_velocity_LB + present_velocity_RB) / 4;

    const int velocity_delta_LF = (velocity_LF - present_velocity_LF) / phase;
    const int velocity_delta_RF = (velocity_RF - present_velocity_RF) / phase;
    const int velocity_delta_LB = (velocity_LB - present_velocity_LB) / phase;
    const int velocity_delta_RB = (velocity_RB - present_velocity_RB) / phase;
    const int average_velocity_delta = (compare_sign * velocity_delta_LF + velocity_delta_RF + compare_sign * velocity_delta_LB + velocity_delta_RB) / 4;

    for (int i = 0; i < phase; i++)
    {
        present_velocity_LF += velocity_delta_LF;
        present_velocity_RF += velocity_delta_RF;
        present_velocity_LB += velocity_delta_LB;
        present_velocity_RB += velocity_delta_RB;

        SetMotor_Velocity(wheel_LF, present_velocity_LF);
        SetMotor_Velocity(wheel_RF, present_velocity_RF);
        SetMotor_Velocity(wheel_LB, present_velocity_LB);
        SetMotor_Velocity(wheel_RB, present_velocity_RB);

        delay = abs(average_velocity_delta);
        this_thread::sleep_for(chrono::milliseconds(delay));
    }

    SetMotor_Velocity(wheel_LF, velocity_LF);
    SetMotor_Velocity(wheel_RF, velocity_RF);
    SetMotor_Velocity(wheel_LB, velocity_LB);
    SetMotor_Velocity(wheel_RB, velocity_RB);
    const int average_velocity = (compare_sign * velocity_LF + velocity_RF + compare_sign * velocity_LB + velocity_RB) / 4;

    // calculate softstart moving distance
    softstart_distance = abs(delay * phase * (average_present_velocity + phase * average_velocity_delta / 2)) * scale2meter_ms;
}
