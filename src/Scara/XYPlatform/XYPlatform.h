#pragma once
#include "../../Robot/MotorUnion/MotorUnion.h"
#include <fstream>

class XYPlatform : public MotorUnion
{
public:
    static XYPlatform *getXYPlatform();
    ~XYPlatform() { inst_ = nullptr; };

    void GotoPosition(const int &target_x, const int &target_y);
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
    const float scale2mm_ms;
    const int MAX_X;
    const int MAX_Y;
    int x;
    int y;

    template <class T>
    constexpr const T clip(const T &v, const T &lo, const T &hi);
    void WriteRecord();
};