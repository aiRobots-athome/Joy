#include "../include/May/Form_May.h"

Form_May::Form_May(QWidget *parent) : QDialog(parent),
                                      ui(new Ui::Form_May),
									  thread_display(nullptr)
{
    ui->setupUi(this);

	form_mayarm = new Form_MayArm(ui);
	thread_mayarm = new QThread();
	QObject::connect(ui->May_btn_PosGo, SIGNAL(clicked()), form_mayarm, SLOT(PositionGo()));
	QObject::connect(ui->May_btn_ImpedancePosGo, SIGNAL(clicked()), form_mayarm, SLOT(ImpedanceGo()));
	QObject::connect(ui->May_btn_GripperOn, SIGNAL(clicked()), form_mayarm, SLOT(GripperHold()));
	QObject::connect(ui->May_btn_GripperOff, SIGNAL(clicked()), form_mayarm, SLOT(GripperRelease()));
	form_mayarm->moveToThread(thread_mayarm);
	thread_mayarm->start();
}

Form_May::~Form_May()
{
	form_mayarm->deleteLater();
	thread_mayarm->deleteLater();
    delete ui;
}

void Form_May::on_May_btn_Start_clicked()
{	
	CMay->Start();
}

void Form_May::on_May_btn_Stop_clicked()
{
	CMay->Stop();
}

////////////////////////////////////////////////////////////////////////////////
///  Display   /////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void Form_May::showEvent(QShowEvent *event)
{
	// Initialize instace and then turn on display
	CMay = May::getMay();
	form_mayarm->SetArm(CMay);

	_is_deleted_thread_display = false;
	thread_display = new std::thread(&Form_May::Display, this);
}

void Form_May::closeEvent(QCloseEvent *event)
{
	_is_deleted_thread_display = true;
	if (thread_display != nullptr)
	{
		thread_display->join();
		delete thread_display;
		thread_display = nullptr;
	}
}

void Form_May::Display()
{
	while (!_is_deleted_thread_display)
	{
		if (ui->Container_May->currentIndex() == 0)
		{
			CMay->CalculateJacobianMatrix();
			form_mayarm->Display();
		}			
		else
			;
		this_thread::sleep_for(chrono::milliseconds(50));
	}
}