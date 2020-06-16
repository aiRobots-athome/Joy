#pragma once
#include "../../Robot/Body/MotorUnion/MotorUnion.h"
#include <fstream>

class XY_Platform: public MotorUnion
{
public:
    XY_Platform(const vector<unsigned char> &IDArray,
	            const vector<string> &MotorModelArray,
			    vector<unsigned char> &AllPortNumber);
    ~XY_Platform(){};

    void GotoPosition(const float &target_x, const float &target_y);

private:
    std::fstream record_file;
    const unsigned char X_Motor; //	(LF = left front)
	const unsigned char Y_Motor;
    const float Rad2MM;
    const float MAX_X;
    const float MAX_Y;
    float x;
    float y;

    template <class T>
    constexpr const T clip(const T &v, const T &lo, const T &hi);
    void ReadWriteRecord();
};