#ifndef FORM_MAYARM_H
#define FORM_MAYARM_H
#include "Gui/ui_Form_May.h"
#include "May/May.h"

class Form_MayArm : public QObject
{
    Q_OBJECT

public:
    Form_MayArm(Ui::Form_May *_ui, QObject *parent = nullptr) : QObject(parent), torque_threshold(50) { ui = _ui; };
    ~Form_MayArm(){};
    void Display();
    void SetArm(May *_May)
    {
        CMay = _May;
    };

public slots:
    void PositionGo();
    void ImpedanceGo();
    void GripperHold();
    void GripperRelease();

private:
    Ui::Form_May *ui;
    void isOK(bool checked_thing, QLabel *label);
    void Red(QLabel *label);
    void Green(QLabel *label);

    static int time_tick_;

    May *CMay;
    const int torque_threshold;
};
#endif