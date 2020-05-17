#ifndef FORM_ARM_H
#define FORM_ARM_H
#include "Gui/ui_Form_Body.h"
#include "Robot/Robot.h"

class Form_Arm : public QObject
{
    Q_OBJECT

public:
    Form_Arm(Ui::Form_Body *_ui, QObject *parent = nullptr) : QObject(nullptr), torque_threshold(50) { ui = _ui; };
    ~Form_Arm(){};
    void Display();

public slots:
    void LeftArm_PosGo();
    void LeftArm_Initial();
    void LeftGripper_Hold();
    void LeftGripper_Release();
    void RightArm_PosGo();
    void RightArm_Initial();
    void RightGripper_Hold();
    void RightGripper_Release();

private:
    Ui::Form_Body *ui;
    void isOK(bool checked_thing, QLabel *label);
    void Red(QLabel *label);
    void Green(QLabel *label);

    const int torque_threshold;
};
#endif