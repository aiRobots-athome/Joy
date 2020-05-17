#ifndef FORM_BODY_H
#define FORM_BODY_H
#include "Gui/ui_Form_Body.h"
#include "Form_Head.h"
#include "Form_Arm.h"
#include "Form_Mobile.h"
#include "XBoxJoystick/XBoxJoystick.h"
#include <QtWidgets/QDialog>
#include <QtCore/QThread>

class Form_Body : public QDialog
{
	Q_OBJECT

public:
	explicit Form_Body(QWidget *parent = nullptr);
	virtual ~Form_Body();

protected:
	void showEvent(QShowEvent *event);
	void closeEvent(QCloseEvent *event);

private slots:
	void on_LeftHand_btn_Stop_clicked();
	void on_RightHand_btn_Stop_clicked();
	void on_Move_btn_Stop_clicked();
	void XBoxJoystick_state(int state);

private:
	Ui::Form_Body *ui;
	Form_Head *form_head;
	Form_Arm *form_arm;
	Form_Mobile *form_mobile;
	QThread *thread_head;
	QThread *thread_arm;
	QThread *thread_mobile;
	XBoxJoystick *CXBoxJoystick;
	
	// Display
	std::thread *thread_display;
	bool _is_deleted_thread_display;
	void Display();
	void SetupImages();
};
#endif // FORM_BODY_H
