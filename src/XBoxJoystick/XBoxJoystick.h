/*
How to use?
1. GetState();
2. At that moment, pressed button and stick angle will be stored in the class: ButtonsAndThumbstick
2. In class ButtonsAndThumbstick, button is enum type and angle is float type
*/
#ifndef _XBOX_JOYSTICK_H_
#define _XBOX_JOYSTICK_H_

#include <SDL.h>
#include <iostream>
#include "../Robot/Mobile/Mobile.h"
#include <thread>

class XBoxJoystick
{
public:
    XBoxJoystick(){};
    ~XBoxJoystick(){};
    void OpenXboxJoystick();
    void CloseXboxJoystick();

private:
    SDL_GameController *controller;
    SDL_Event *event;
    std::thread *thread_getState;
    bool is_deleted_thread;

    void GetState();
    void LeftStickMotion();
    void DPadMotion();
    void ShoulderMotion();

    Mobile *CMobile;
};
#endif