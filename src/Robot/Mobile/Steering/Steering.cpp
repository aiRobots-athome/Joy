#include "Steering.h"

Steering *Steering::inst_ = nullptr;
Steering *Steering::getSteering()
{
    if (inst_ == nullptr)
        inst_ = new Steering();
    return inst_;
}

Steering::Steering()
    : MotorUnion({0, 1, 2, 3}, {"Pro20", "Pro20", "Pro20", "Pro20"}),
      steering_LF(0),
      steering_RF(1),
      steering_LB(2),
      steering_RB(3),
      steering_gear_ratio(1.75)
{
    SetMotor_CenterScale(steering_LF, 284); //Motor center scale shift
    SetMotor_CenterScale(steering_RF, 5505);
    SetMotor_CenterScale(steering_LB, 5459);
    SetMotor_CenterScale(steering_RB, 602);
    SetAllMotorsAccel(2000);
    Start();
}

void Steering::TurnStraight()
{
    /*
        |  |
        |  |
    */
    SetAllMotorsAngle(0.0);
}

void Steering::TurnHorizontal()
{
    /*
        -  -
        -  -
    */
    float fangle = 90.0 * steering_gear_ratio;
    SetMotor_Angle(steering_LF, -fangle);
    SetMotor_Angle(steering_RF, fangle);
    SetMotor_Angle(steering_LB, -fangle);
    SetMotor_Angle(steering_RB, fangle);
}

void Steering::TurnAll(const float &angle)
{
    /*
        /  /     \  \ 
        /  / or  \  \ 轉角都相同
    */
    float fangle = angle * steering_gear_ratio;
    SetMotor_Angle(steering_LF, fangle);
    SetMotor_Angle(steering_RF, fangle);
    SetMotor_Angle(steering_LB, fangle);
    SetMotor_Angle(steering_RB, fangle);
}

void Steering::TurnFront(const float &angle1, const float &angle2)
{
    /*
        /  /     \  \
        |  | or  |  |  內外輪轉角不同
    */
    SetMotor_Angle(steering_LF, angle1 * steering_gear_ratio);
    SetMotor_Angle(steering_RF, angle2 * steering_gear_ratio);
    SetMotor_Angle(steering_LB, 0);
    SetMotor_Angle(steering_RB, 0);
}

void Steering::TurnCircle(const float &angle1, const float &angle2, const float &angle3, const float &angle4)
{
    /*
        /  /     \  \
        \  \ or  /  /  內外輪轉角不同
    */
    SetMotor_Angle(steering_LF, angle1 * steering_gear_ratio);
    SetMotor_Angle(steering_RF, angle2 * steering_gear_ratio);
    SetMotor_Angle(steering_LB, angle3 * steering_gear_ratio);
    SetMotor_Angle(steering_RB, angle4 * steering_gear_ratio);
}

void Steering::Self_Turn()
{
    /*
        /  \
        \  /
    */

    float fangle = 45.0 * steering_gear_ratio;
    SetMotor_Angle(steering_LF, -fangle);
    SetMotor_Angle(steering_RF, fangle);
    SetMotor_Angle(steering_LB, fangle);
    SetMotor_Angle(steering_RB, -fangle);
}

void Steering::Wait()
{
    WaitAllMotorsArrival();
}

void Steering::Start()
{
    SetAllMotorsVelocity(2000);
    SetAllMotorsTorqueEnable(true);
}