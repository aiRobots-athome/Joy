#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;

class MouseEvent
{
    public:
    MouseEvent();
    ~MouseEvent();

    void OnMouse(int event, int x, int y);
    static void OnMouse(int event, int x, int y, int, void* userdata);

    //mouse
    cv::Point mouse_point;
    cv::Point rect_left_top;
    cv::Point rect_right_bot;
    cv::Point rect_center;

    bool rect_flag=false;

};