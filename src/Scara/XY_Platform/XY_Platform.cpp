#include "XY_PLatform.h"

XY_Platform::XY_Platform(const vector<unsigned char> &IDArray,
                         const vector<string> &MotorModelArray,
                         vector<unsigned char> &AllPortNumber)
    : MotorUnion(IDArray, MotorModelArray, AllPortNumber),
      X_Motor(0),
      Y_Motor(1),
      Rad2MM(10 / (M_PI / 4)), //90 degree to 10 mm
      MAX_X(0.275),
      MAX_Y(0.275)
{
    SetAllMotorsOperatingMode(1);

    record_file.open(std::string(getenv("PWD")) + "/../XY_Platform/record.txt");

    if (record_file.is_open())
    {
        string str;
        getline(record_file, str);
        x = stof(str);
        getline(record_file, str);
        y = stof(str);
        record_file.close();
    }
    else
        cout << "XY_Platform record file cannot open!" << endl;
}

void XY_Platform::GotoPosition(const float &target_x, const float &target_y)
{
    float tmp_target_x = clip(target_x, 0.0f, MAX_X);
    float tmp_target_y = clip(target_y, 0.0f, MAX_Y);
    float t_x = (tmp_target_x - x) / (GetMotor_Velocity(X_Motor) * GetMotor_Scale2RPM(X_Motor) * 2 * M_PI / 60 * Rad2MM) * 1000;
    float t_y = (tmp_target_y - y) / (GetMotor_Velocity(Y_Motor) * GetMotor_Scale2RPM(Y_Motor) * 2 * M_PI / 60 * Rad2MM) * 1000;
    x = tmp_target_x;
    y = tmp_target_y;
    WaitAllMotorsArrival(t_x);
    ReadWriteRecord();
}

template <class T>
constexpr const T XY_Platform::clip(const T &v, const T &lo, const T &hi)
{
    T tmp;
    tmp = v < lo ? lo : v;
    tmp = v > hi ? hi : v;
    return tmp;
}

void XY_Platform::ReadWriteRecord()
{
    record_file.open(std::string(getenv("PWD")) + "/../XY_Platform/record.txt", ios::out | ios::trunc);
    record_file << x << "\n" << y;
}