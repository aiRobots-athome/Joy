#include "../include/Form_Scara.h"

Form_Scara::Form_Scara(QWidget *parent) : QDialog(parent), ui(new Ui::Form_Scara), thread_display(nullptr)
{
    ui->setupUi(this);
	
    form_scara = new Form_ScaraArm(ui);
    thread_Scara = new QThread;
    QObject::connect(ui->ScaraArm_btn_PosGo, SIGNAL(clicked()), form_scara, SLOT(on_ScaraArm_btn_PosGo_clicked()));
    QObject::connect(ui->Scara_btn_Reset, SIGNAL(clicked()), form_scara, SLOT(on_Scara_btn_Reset_clicked()));
    QObject::connect(ui->Screw_btn_Up, SIGNAL(clicked()), form_scara, SLOT(on_Screw_btn_Up_clicked()));
    QObject::connect(ui->Screw_btn_Down, SIGNAL(clicked()), form_scara, SLOT(on_Screw_btn_Down_clicked()));
    QObject::connect(ui->Goal_Height_btn, SIGNAL(clicked()), form_scara, SLOT(on_Goal_Height_btn_clicked()));
    form_scara->moveToThread(thread_Scara);
    thread_Scara->start();

}

Form_Scara::~Form_Scara()
{
    form_scara->deleteLater();
    thread_Scara->deleteLater();
	delete ui;
}
void Form_Scara::showEvent(QShowEvent *event)
{
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
	}
}

void Form_Scara::Display()
{
	while (!_is_deleted_thread_display)
	{
		if (ui->Correction->currentIndex() == 0)
			form_scara->Display();
		else if (ui->Correction->currentIndex() == 1)
			;
		else if (ui->Correction->currentIndex() == 2)
			;
		else
			;
		this_thread::sleep_for(chrono::milliseconds(50));
	}
}

void Form_Scara::on_Screw_btn_Stop_clicked()
{
    CScaraArm->StopScrew();
}

void Form_Scara::on_ScaraArm_btn_Torque_Off_clicked()
{
	CScaraArm->AllmoterTorque(false);
}
void Form_Scara::on_ScaraArm_btn_Torque_On_clicked()
{
	CScaraArm->AllmoterTorque(true);
}