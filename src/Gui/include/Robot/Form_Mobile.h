#ifndef FORM_MOBILE_H
#define FORM_MOBILE_H
#include <QtCore/QObject>
#include "Gui/ui_Form_Robot.h"
#include "Robot/Mobile/Mobile.h"

class Form_Mobile : public QObject
{
    Q_OBJECT

public:
    Form_Mobile(Ui::Form_Robot *_ui, QObject *parent = nullptr) : QObject(parent), torque_threshold(50) { ui = _ui; };
    ~Form_Mobile(){};
    void Display();
    void SetMobile(Mobile *_Mobile)
    {
        CMobile = _Mobile;
        CSteering = CMobile->CSteering;
        CWheel = CMobile->CWheel;
    }

public slots:
    void MoveForward();
    void MoveBackward();
    void MoveLeft();
    void MoveRight();

    void Oblique_Forward();
    void Oblique_Backward();
    void Turn_Forward();
    void Turn_Backward();
    void TurnCircleByRadius_Forward();
    void TurnCircleByRadius_Backward();
    void SelfTurn_Left();
    void SelfTurn_Right();

private:
    Ui::Form_Robot *ui;
    void isOK(bool checked_thing, QLabel *label);
    void Red(QLabel *label);
    void Green(QLabel *label);

    Mobile *CMobile;
    Steering *CSteering;
    Wheel *CWheel;
    const int torque_threshold;
};

#endif