#ifndef FORM_SCARA_H
#define FORM_SCARA_H
#include "Gui/ui_Form_Scara.h"
#include "Form_ScaraArm.h"
#include "Form_XYPlatform.h"
#include "Form_VisionCar.h"
#include <QtWidgets/QDialog>
#include <QtCore/QThread>

class Form_Scara : public QDialog
{
	Q_OBJECT

public:
	explicit Form_Scara(QWidget *parent = nullptr);
	virtual ~Form_Scara();

protected:
	void showEvent(QShowEvent *event);
	void closeEvent(QCloseEvent *event);

private slots:
	void on_Scara_btn_Reconnect_clicked();

	void on_ScaraArm_btn_Start_clicked();
	void on_ScaraArm_btn_Stop_clicked();

	void on_XYPlatform_btn_Start_clicked();
	void on_XYPlatform_btn_Stop_clicked();

	void on_VisionCar_btn_Start_clicked();
	void on_VisionCar_btn_Stop_clicked();

private:
	Ui::Form_Scara *ui;

	Form_ScaraArm *form_scara_arm;
	Form_XYPlatform *form_xy_platform;
	Form_VisionCar *form_visioncar;

	QThread *thread_scara_arm;
	QThread *thread_xy_platform;
	QThread *thread_visioncar;
	
	Scara *CScara;


	//Display
	std::thread *thread_display;
	bool _is_deleted_thread_display;
	void Display();
};

#endif // FORM_SCARA_H