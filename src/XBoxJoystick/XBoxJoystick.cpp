#include "XBoxJoystick.h"

void XBoxJoystick::OpenXboxJoystick()
{
    SDL_Init(SDL_INIT_GAMECONTROLLER);

    if (SDL_NumJoysticks() > 0)
    {
        if (SDL_GameControllerOpen(0))
        {
            controller = SDL_GameControllerOpen(0);
            CMobile = Mobile::getMobile();
            is_deleted_thread = false;
            thread_getState = new thread(&XBoxJoystick::GetState, this);
            printf("Connect to controller successfully !\n");
        }
    }
    else
        printf("Could not open gamecontroller !\n");
}

void XBoxJoystick::CloseXboxJoystick()
{
    if (SDL_NumJoysticks() > 0)
    {
        if (is_deleted_thread == false)
        {
            is_deleted_thread = true;
            thread_getState->join();
            delete thread_getState;
            SDL_GameControllerClose(controller);
        }
    }
}

void XBoxJoystick::GetState()
{
    while (!is_deleted_thread)
    {
        while (SDL_PollEvent(event))
        {
            LeftStickMotion();
            DPadMotion();
            ShoulderMotion();
            if (event->cbutton.type == SDL_CONTROLLERBUTTONUP)
                CMobile->CWheel->Stop();
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }
}

void XBoxJoystick::LeftStickMotion()
{
    if (event->cbutton.type == SDL_CONTROLLERAXISMOTION)
    {
        int x = -SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX);
        int y = -SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTY);
        short threshold = 20000;
        x = copysign(abs(x) - threshold < 0 ? 0 : abs(x) - threshold, x);
        y = copysign(abs(y) - threshold < 0 ? 0 : abs(y) - threshold, y);
        const int angle = atan2f(x, y) * Rad2Angle;
        if (x > 100 && y > 100)
            CMobile->Turn(angle);
    }
}

void XBoxJoystick::DPadMotion()
{
    if (event->cbutton.type == SDL_CONTROLLERBUTTONDOWN)
    {
        switch (event->cbutton.button)
        {
        case SDL_CONTROLLER_BUTTON_DPAD_UP:
            CMobile->MoveForward();
            break;
        case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
            CMobile->MoveBackward();
            break;
        case SDL_CONTROLLER_BUTTON_DPAD_LEFT:
            CMobile->MoveLeft();
            break;
        case SDL_CONTROLLER_BUTTON_DPAD_RIGHT:
            CMobile->MoveRight();
            break;
        default:
            break;
        }
    }
}

void XBoxJoystick::ShoulderMotion()
{
    if (event->cbutton.type == SDL_CONTROLLERBUTTONDOWN)
    {
        switch (event->cbutton.button)
        {
        case SDL_CONTROLLER_BUTTON_LEFTSHOULDER:
            CMobile->SelfTurn();
            break;
        case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER:
            CMobile->SelfTurn(0, -Mobile::default_velocity);
            break;

        default:
            break;
        }
    }
}