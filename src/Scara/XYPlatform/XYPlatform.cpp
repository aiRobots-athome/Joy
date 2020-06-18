#include "XYPlatform.h"

XYPlatform *XYPlatform::inst_ = nullptr;
XYPlatform *XYPlatform::getXYPlatform()
{
    if (inst_ == nullptr)
        inst_ = new XYPlatform();
    return inst_;
}

XYPlatform::XYPlatform()
    : MotorUnion({0, 1}, {"Mx106", "Mx106"}),
      X_Motor(0),
      Y_Motor(1),
      scale2mm_ms(GetMotor_Scale2RPM(X_Motor) * 2 * M_PI * (1.667 * 1e-5) / (M_PI * 0.5) * 10), //90 degree to 10 mm
      MAX_X(275),
      MAX_Y(275)
{
    SetAllMotorsOperatingMode(1);

    record_file.open(string(getenv("PWD")) + "/src/Scara/XYPlatform/XYRecord.txt");
    if (record_file.is_open())
    {
        char str[20];
        record_file.getline(str, sizeof(str), '\n');
        x = stof(str);
        record_file.getline(str, sizeof(str), '\n');
        y = stof(str);
        record_file.close();
    }
    else
        cout << "[XYPlatform] cannot open XYRecord.txt!" << endl;
}

void XYPlatform::GotoPosition(const int &target_x, const int &target_y)
{
    int tmp_target_x = clip(target_x, 0, MAX_X);
    int tmp_target_y = clip(target_y, 0, MAX_Y);
    int time_x = abs(tmp_target_x - x) / (GetMotor_Velocity(X_Motor) * scale2mm_ms);
    int time_y = abs(tmp_target_y - y) / (GetMotor_Velocity(Y_Motor) * scale2mm_ms);
    x = tmp_target_x;
    y = tmp_target_y;
    WaitAllMotorsArrival(max(time_x, time_y));
    WriteRecord();
}

const int &XYPlatform::GetPresentX() const
{
    return x;
}

const int &XYPlatform::GetPresentY() const
{
    return y;
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

void XYPlatform::WriteRecord()
{
    record_file.open(string(getenv("PWD")) + "/src/Scara/XYPlatform/XYRecord.txt", ios::out);
    if (record_file.is_open())
        record_file << x << '\n'
                    << y << '\n';
    else
        cout << "[XYPlatform] cannot open XYRecord.txt!" << endl;
    record_file.close();
}