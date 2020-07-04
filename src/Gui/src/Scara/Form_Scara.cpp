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
	QObject::connect(ui->XYPlatform_btn_PosGo, SIGNAL(clicked()), form_xy_platform, SLOT(XYPlatformPosGo()));
	QObject::connect(ui->XYPlatform_btn_Reset, SIGNAL(clicked()), form_xy_platform, SLOT(XYPlatformReset()));
	form_xy_platform->moveToThread(thread_xy_platform);
	thread_xy_platform->start();
}

Form_Scara::~Form_Scara()
{
	form_scara_arm->deleteLater();
	form_xy_platform->deleteLater();
	thread_scara_arm->deleteLater();
	thread_xy_platform->deleteLater();
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

////////////////////////////////////////////////////////////////////////////////
///  Display   /////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void Form_Scara::showEvent(QShowEvent *event)
{
	CScara = Scara::getScara();
	form_scara_arm->SetScaraArm(CScara->CScaraArm);
	form_xy_platform->SetXYPlatform(CScara->CXYPlatform);
	
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
			;
		else
			;
		this_thread::sleep_for(chrono::milliseconds(50));
	}
}