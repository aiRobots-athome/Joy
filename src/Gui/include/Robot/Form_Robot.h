#ifndef FORM_ROBOT_H
#define FORM_ROBOT_H
#include "Gui/ui_Form_Robot.h"
#include "Form_Head.h"
#include "Form_Arm.h"
#include "Form_Mobile.h"
#include "Form_ForceSensor.h"
#include "Robot/Robot.h"
#include "XBoxJoystick/XBoxJoystick.h"
#include <QtWidgets/QDialog>
#include <QtCore/QThread>

class Form_Robot : public QDialog
{
	Q_OBJECT

public:
	explicit Form_Robot(QWidget *parent = nullptr);
	virtual ~Form_Robot();

protected:
	void showEvent(QShowEvent *event);
	void closeEvent(QCloseEvent *event);

private slots:
	void on_Robot_btn_Reconnect_clicked();
	void on_LeftHand_btn_Start_clicked();
	void on_LeftHand_btn_Stop_clicked();
	void on_RightHand_btn_Start_clicked();
	void on_RightHand_btn_Stop_clicked();
	void on_Move_btn_Start_clicked();
	void on_Move_btn_Stop_clicked();
	void XBoxJoystick_state(int state);

private:
	Ui::Form_Robot *ui;
	Form_Head *form_head;
	Form_Arm *form_arm;
	Form_Mobile *form_mobile;
	Form_ForceSensor *form_forcesensor;
	QThread *thread_head;
	QThread *thread_arm;
	QThread *thread_mobile;
	QThread *thread_forcesensor;

	// Display
	std::thread *thread_display;
	bool _is_deleted_thread_display;
	void Display();
	void SetupImages();

	Robot *CRobot;
	XBoxJoystick *CXBoxJoystick;
};
#endif
