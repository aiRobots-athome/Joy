#include "Scara/Form_Scara.h"

Form_Scara::Form_Scara(QWidget *parent) : QDialog(parent),
										  ui(new Ui::Form_Scara),
										  thread_display(nullptr)
{
	ui->setupUi(this);
	
	form_scara_arm = new Form_ScaraArm(ui);
	thread_scara_arm = new QThread();
	QObject::connect(ui->ScaraArm_btn_PosGo, SIGNAL(clicked()), form_scara_arm, SLOT(on_ScaraArm_btn_PosGo_clicked()));
	QObject::connect(ui->ScaraArm_btn_Reset, SIGNAL(clicked()), form_scara_arm, SLOT(on_Scara_btn_Reset_clicked()));
	QObject::connect(ui->Screw_btn_Up, SIGNAL(clicked()), form_scara_arm, SLOT(on_Screw_btn_Up_clicked()));
	QObject::connect(ui->Screw_btn_Down, SIGNAL(clicked()), form_scara_arm, SLOT(on_Screw_btn_Down_clicked()));
	QObject::connect(ui->Screw_btn_Go, SIGNAL(clicked()), form_scara_arm, SLOT(on_Goal_Height_btn_clicked()));
	form_scara_arm->moveToThread(thread_scara_arm);
	thread_scara_arm->start();

	form_xy_platform = new Form_XYPlatform(ui);
	thread_xy_platform = new QThread();
	QObject::connect(ui->XYPlatform_spinBox_X, SIGNAL(valueChanged(int)), form_xy_platform, SLOT(XYPlatformPosGo()));
	QObject::connect(ui->XYPlatform_spinBox_Y, SIGNAL(valueChanged(int)), form_xy_platform, SLOT(XYPlatformPosGo()));
	QObject::connect(ui->XYPlatform_btn_Reset, SIGNAL(clicked()), form_xy_platform, SLOT(XYPlatformReset()));
	form_xy_platform->moveToThread(thread_xy_platform);
	thread_xy_platform->start();

	form_visioncar = new Form_VisionCar(ui);
	thread_visioncar= new QThread();
	QObject::connect(ui->VisionCar_btn_PosGo, SIGNAL(clicked()), form_visioncar, SLOT(on_VisionCar_btn_PosGo_clicked()));
	QObject::connect(ui->VisionCar_Cam_CTL, SIGNAL(clicked()), form_visioncar, SLOT(on_VisionCar_Cam_CTL_clicked()));
	QObject::connect(ui->VisionCar_Screw_CTL, SIGNAL(clicked()), form_visioncar, SLOT(on_VisionCar_Screw_CTL_clicked()));
	QObject::connect(ui->VisionCar_btn_Reset, SIGNAL(clicked()), form_visioncar, SLOT(on_VisionCar_btn_Reset_clicked()));
	form_visioncar->moveToThread(thread_visioncar);
	thread_visioncar->start();
}

Form_Scara::~Form_Scara()
{
	form_scara_arm->deleteLater();
	form_xy_platform->deleteLater();
	form_visioncar->deleteLater();

	thread_scara_arm->deleteLater();
	thread_xy_platform->deleteLater();
	thread_visioncar->deleteLater();
	delete ui;
}

void Form_Scara::on_Scara_btn_Reconnect_clicked()
{
	_is_deleted_thread_display = true;
	if (thread_display != nullptr)
	{
		thread_display->join();
		delete thread_display;
	}

	CScara->Reconnect();
	form_scara_arm->SetScaraArm(CScara->CScaraArm);
	form_xy_platform->SetXYPlatform(CScara->CXYPlatform);
	form_visioncar->SetVisionCar(CScara->CVisionCar);

	_is_deleted_thread_display = false;
	thread_display = new std::thread(&Form_Scara::Display, this);
}

void Form_Scara::on_ScaraArm_btn_Start_clicked()
{
	CScara->CScaraArm->Start();
}
void Form_Scara::on_ScaraArm_btn_Stop_clicked()
{
	CScara->CScaraArm->Stop();
}

void Form_Scara::on_XYPlatform_btn_Start_clicked()
{
	CScara->CXYPlatform->Start();
}
void Form_Scara::on_XYPlatform_btn_Stop_clicked()
{
	CScara->CXYPlatform->Stop();
}

void Form_Scara::on_VisionCar_btn_Start_clicked()
{
	CScara->CVisionCar->Start();
}
void Form_Scara::on_VisionCar_btn_Stop_clicked()
{
	CScara->CVisionCar->Stop();
}

////////////////////////////////////////////////////////////////////////////////
///  Display   /////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void Form_Scara::showEvent(QShowEvent *event)
{
	CScara = Scara::getScara();
	form_scara_arm->SetScaraArm(CScara->CScaraArm);
	form_xy_platform->SetXYPlatform(CScara->CXYPlatform);
	form_visioncar->SetVisionCar(CScara->CVisionCar);
	form_xy_platform->Initial();
	
	_is_deleted_thread_display = false;
	thread_display = new std::thread(&Form_Scara::Display, this);
}

void Form_Scara::closeEvent(QCloseEvent *event)
{
	_is_deleted_thread_display = true;
	if (thread_display != nullptr)
	{
		thread_display->join();
		delete thread_display;
		thread_display = nullptr;
	}
}

void Form_Scara::Display()
{
	while (!_is_deleted_thread_display)
	{
		if (ui->Correction->currentIndex() == 0)
			form_scara_arm->Display();
		else if (ui->Correction->currentIndex() == 1)
			form_xy_platform->Display();
		else if (ui->Correction->currentIndex() == 2)
			form_visioncar->Display();
		else
			;
		this_thread::sleep_for(chrono::milliseconds(50));
	}
}