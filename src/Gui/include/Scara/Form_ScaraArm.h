#ifndef FORM_SCARAARM_H
#define FORM_SCARAARM_H
#include "Gui/ui_Form_Scara.h"
#include "Scara/Scara.h"

class Form_ScaraArm : public QObject
{
    Q_OBJECT

public:
    Form_ScaraArm(Ui::Form_Scara *_ui, QObject *parent = nullptr) : QObject(parent), torque_threshold(50) { ui = _ui; };
    ~Form_ScaraArm(){};
    void Display();
    void SetScaraArm(ScaraArm *_ScaraArm)
    {
        CScaraArm = _ScaraArm;
    }

public slots:
    void on_ScaraArm_btn_PosGo_clicked();
    void on_Scara_btn_Reset_clicked();
    void on_Screw_btn_Up_clicked();
    void on_Screw_btn_Down_clicked();
    void on_Goal_Height_btn_clicked();

private:
    Ui::Form_Scara *ui;
    void isOK(bool checked_thing, QLabel *label);
    void Red(QLabel *label);
    void Green(QLabel *label);
    void Get_Now_Position();

    const int torque_threshold;

    ScaraArm *CScaraArm;
};
#endif // FORM_SCARAARM_H