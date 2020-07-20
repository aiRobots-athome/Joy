#ifndef FORM_VisionCar_H
#define FORM_VisionCar_H
#include "Gui/ui_Form_Scara.h"
#include "Scara/Scara.h"

class Form_VisionCar : public QObject
{
    Q_OBJECT

public:
    Form_VisionCar(Ui::Form_Scara *_ui, QObject *parent = nullptr) : QObject(parent) { ui = _ui; };
    ~Form_VisionCar(){};
    void Display();
    void SetVisionCar(VisionCar *_VisionCar)
    {
        CVisionCar = _VisionCar;
    }

public slots:
    void on_VisionCar_btn_PosGo_clicked();
    void on_VisionCar_btn_Reset_clicked();
    void on_VisionCar_Cam_CTL_clicked();
    void on_VisionCar_Screw_CTL_clicked();

private:
    Ui::Form_Scara *ui;
    void isOK(bool checked_thing, QLabel *label);

    VisionCar *CVisionCar;
};
#endif // FORM_VisionCar_H