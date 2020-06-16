#ifndef FORM_ARM_H
#define FORM_ARM_H
#include "Gui/ui_Form_Robot.h"
#include "Robot/Arm/SaleArmLeft.h"
#include "Robot/Arm/SaleArmRight.h"

class Form_Arm : public QObject
{
    Q_OBJECT

public:
    Form_Arm(Ui::Form_Robot *_ui, QObject *parent = nullptr) : QObject(parent), torque_threshold(50) { ui = _ui; };
    ~Form_Arm(){};
    void Display();
    void SetArm(SaleArmLeft *_LeftArm, SaleArmRight *_RightArm)
    {
        CLeftArm = _LeftArm;
        CRightArm = _RightArm;
    };

public slots:
    void LeftArm_PosGo();
    void LeftGripper_Hold();
    void LeftGripper_Release();
    void RightArm_PosGo();
    void RightGripper_Hold();
    void RightGripper_Release();

private:
    Ui::Form_Robot *ui;
    void isOK(bool checked_thing, QLabel *label);
    void Red(QLabel *label);
    void Green(QLabel *label);

    SaleArmLeft *CLeftArm;
    SaleArmRight *CRightArm;
    const int torque_threshold;
};
#endif