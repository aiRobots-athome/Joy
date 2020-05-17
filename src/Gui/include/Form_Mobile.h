#ifndef FORM_MOBILE_H
#define FORM_MOBILE_H
#include <QtCore/QObject>
#include "Gui/ui_Form_Body.h"
#include "Robot/Robot.h"

class Form_Mobile : public QObject
{
    Q_OBJECT

public:
    Form_Mobile(Ui::Form_Body *_ui, QObject *parent = nullptr) : QObject(nullptr), torque_threshold(50) { ui = _ui; };
    ~Form_Mobile(){};
    void Display();

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
    Ui::Form_Body *ui;
    void isOK(bool checked_thing, QLabel *label);
    void Red(QLabel *label);
    void Green(QLabel *label);

    const int torque_threshold;
};

#endif