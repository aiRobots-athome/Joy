#pragma once
#include "../../Robot/MotorUnion/MotorUnion.h"
#include <fstream>
#include <thread>

class XYPlatform : public MotorUnion
{
public:
    static XYPlatform *getXYPlatform();
    ~XYPlatform() { inst_ = nullptr; };

    void Start();
    void Stop();

    void GotoPosition(const unsigned char &MotorID, const int &target_pos, const int &speed = 100);
    void GotoPosition(const int &target_x, const int &target_y, const int &speed = 100);
    const int &GetPresentX() const;
    const int &GetPresentY() const;
    const int &GetMaxX() const;
    const int &GetMaxY() const;

private:
    XYPlatform();
    static XYPlatform *inst_;
    std::fstream record_file;
    const unsigned char X_Motor;
    const unsigned char Y_Motor;
    const float RPM2mm_ms;
    const int MAX_X;
    const int MAX_Y;
    int present_x;
    int present_y;

    template <class T>
    constexpr const T clip(const T &v, const T &lo, const T &hi);
};