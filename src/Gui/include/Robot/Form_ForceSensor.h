#ifndef FORM_FORCESENSOR_H
#define FORM_FORCESENSOR_H
#include "Gui/ui_Form_Robot.h"
#include "Robot/ForceSensor/ForceSensor.h"

class Form_ForceSensor : public QObject
{
    Q_OBJECT

public:
    Form_ForceSensor(Ui::Form_Robot *_ui, QObject *parent = nullptr) : QObject(parent){ ui = _ui; };
    ~Form_ForceSensor(){};
    void Display();
    void SetForceSensor(ForceSensor *_Left, ForceSensor *_Right)
    {
        CLeftForceSensor = _Left;
        CRightForceSensor = _Right;
    };

public slots:
    void LeftForceSensorNormalize();
    void RightForceSensorNormalize();

private:
    Ui::Form_Robot *ui;

    ForceSensor *CLeftForceSensor;
    ForceSensor *CRightForceSensor;
};
#endif