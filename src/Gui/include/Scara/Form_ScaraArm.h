#ifndef FORM_SCARA_H
#define FORM_SCARA_H
#include "Gui/ui_Form_Scara.h"
#include "../../Scara/Scara.h"

class Form_ScaraArm : public QObject
{
	Q_OBJECT

public:
	Form_ScaraArm(Ui::Form_Scara *_ui, QObject *parent = nullptr) : QObject(nullptr), torque_threshold(50) { ui = _ui; };
	~Form_ScaraArm(){};
    void Display();

protected:
    void showEvent(QShowEvent *event);
    void closeEvent(QCloseEvent *event);

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

};
#endif // FORM_Scara_H