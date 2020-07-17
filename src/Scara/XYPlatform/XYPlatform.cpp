#include "XYPlatform.h"

XYPlatform *XYPlatform::inst_ = nullptr;
XYPlatform *XYPlatform::getXYPlatform()
{
    if (inst_ == nullptr)
        inst_ = new XYPlatform();
    return inst_;
}

XYPlatform::XYPlatform()
    : MotorUnion({10, 11}, {"Mx106", "Mx106"}),
      X_Motor(0),
      Y_Motor(1),
      RPM2mm_ms(2 * M_PI * (1.667 * 1e-5) / (M_PI * 0.5) * 10), //90 degree to 10 mm
      MAX_X(154),
      MAX_Y(227)
{
    SetAllMotorsOperatingMode(1);
    SetAllMotorsTorqueEnable(true);
    present_x = 0;
    present_y = 0;

    cout << "\t\tClass constructed: XY_Platform" << endl;
}

void XYPlatform::Start()
{
    SetAllMotorsTorqueEnable(true);
}

void XYPlatform::Stop()
{
    SetAllMotorsTorqueEnable(false);
}

void XYPlatform::GotoPosition(const unsigned char &MotorID, const int &target_pos, const int &speed)
{
    int displacement = 0;
    int tmp_target_pos = 0;

    if (MotorID == X_Motor)
    {
        tmp_target_pos = clip(target_pos, -MAX_X, MAX_X);
        displacement = tmp_target_pos - present_x;
        present_x = tmp_target_pos;
    }
    else
    {
        tmp_target_pos = clip(target_pos, -MAX_Y, MAX_Y);
        displacement = tmp_target_pos - present_y;
        present_y = tmp_target_pos;
    }
    int velocity = copysign(speed, displacement);
    int time = displacement / (velocity * GetMotor_Scale2RPM(MotorID) * RPM2mm_ms);

    SetMotor_Velocity(MotorID, velocity);
    WaitAllMotorsArrival(time);
    SetMotor_Velocity(MotorID, 0);
}

void XYPlatform::GotoPosition(const int &target_x, const int &target_y, const int &speed)
{
    void (XYPlatform::*callfunc)(const unsigned char &, const int &, const int &) = &XYPlatform::GotoPosition;
    thread thread_x = thread(callfunc, this, X_Motor, target_x, speed);
    thread thread_y = thread(callfunc, this, Y_Motor, target_y, speed);
    thread_x.join();
    thread_y.join();
}

const int &XYPlatform::GetPresentX() const
{
    return present_x;
}

const int &XYPlatform::GetPresentY() const
{
    return present_y;
}

const int &XYPlatform::GetMaxX() const
{
    return MAX_X;
}

const int &XYPlatform::GetMaxY() const
{
    return MAX_Y;
}

template <class T>
constexpr const T XYPlatform::clip(const T &v, const T &lo, const T &hi)
{
    return v > hi ? hi : (v < lo ? lo : v);
}