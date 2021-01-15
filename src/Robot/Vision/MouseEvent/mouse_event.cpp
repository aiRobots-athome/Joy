#include"mouse_event.h"

MouseEvent::MouseEvent()
{
    this->mouse_point=cv::Point(-1,-1);

}
MouseEvent::~MouseEvent()
{

}
void MouseEvent::OnMouse(int event, int x, int y, int, void* userdata)
{
    
    MouseEvent* me = reinterpret_cast<MouseEvent*>(userdata);
    me->OnMouse(event, x, y);

}
void MouseEvent::OnMouse(int event, int x, int y)
{
    if(event==cv::EVENT_MOUSEMOVE)
    {
        cout<<"mouse"<<endl;
        this->mouse_point.x = x;
        this->mouse_point.y = y;
    }
    if(event==cv::EVENT_LBUTTONDOWN)
    {
        this->rect_left_top.x = x;
        this->rect_left_top.y = y;
    }
    if(event==cv::EVENT_LBUTTONUP)
    {
        this->rect_right_bot.x = x;
        this->rect_right_bot.y = y;
        cv::Point tmp;
        tmp.x=(this->rect_left_top.x+this->rect_right_bot.x)/2;
        tmp.y=(this->rect_left_top.y+this->rect_right_bot.y)/2;
        this->rect_center=tmp;
        rect_flag=true;
    }

}