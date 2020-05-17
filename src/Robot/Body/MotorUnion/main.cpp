#include "MotorUnion.h"

int main()
{
    vector <unsigned char> port = {0, 1};
    vector <unsigned char> id = {0, 1, 2, 3, 4, 5};
    vector <string> model = {"Mx106", "Mx106", "Mx106", "Mx106", "Mx106", "Mx106"};
    MotorUnion a = MotorUnion(id, model, port);

    while(true)
    {
        float degree;
        int idx = 4;
        cin >> degree;
        a.SetMotor_Angle(4, degree);
        cout << a.GetMotor_PresentAngle(idx) << endl;
    }
}