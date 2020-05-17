#include "MotorUnion.h"

int main()
{
    vector <unsigned char> port = {0};
    vector <unsigned char> id = {0, 1, 2, 3, 4, 5};
    vector <string> model = {"Mx106", "Mx106", "Mx106", "Mx106","Mx106", "Mx106"};
    MotorUnion a = MotorUnion(id, model, port);

    int idx = 0;
    cin >> idx;
    cout 
    << a.GetMotor_Connected(idx) << endl
    << a.GetMotor_ID(idx) << endl 
    << a.GetMotor_PresentAngle(idx) << endl
    << a.GetMotor_Velocity(idx) << endl
    << a.GetMotor_PresentTorque(idx) << endl;
    this_thread::sleep_for(chrono::seconds(1));
}