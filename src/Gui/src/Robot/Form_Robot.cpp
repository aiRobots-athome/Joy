#include "Robot/Form_Robot.h"

Form_Robot::Form_Robot(QWidget *parent) : QDialog(parent),
										  ui(new Ui::Form_Robot),
										  thread_display(nullptr)
{
	ui->setupUi(this);
	SetupImages();

	/* Head */
	form_head = new Form_Head(ui);
	thread_head = new QThread();
	QObject::connect(ui->Head_btn_Reset, SIGNAL(clicked()), form_head, SLOT(HeadReset()));
	QObject::connect(ui->HeadHorizontal_Slider, SIGNAL(valueChanged(int)), form_head, SLOT(HeadHorizontalMove()));
	QObject::connect(ui->HeadVertical_Slider, SIGNAL(valueChanged(int)), form_head, SLOT(HeadVerticalMove()));
	form_head->moveToThread(thread_head);
	thread_head->start();

	/* Arm */
	form_arm = new Form_Arm(ui);
	thread_arm = new QThread();
	QObject::connect(ui->LeftHand_btn_PosGo, SIGNAL(clicked()), form_arm, SLOT(LeftArm_PosGo()));
	QObject::connect(ui->LeftGripper_btn_Hold, SIGNAL(clicked()), form_arm, SLOT(LeftGripper_Hold()));
	QObject::connect(ui->LeftGripper_btn_Release, SIGNAL(clicked()), form_arm, SLOT(LeftGripper_Release()));
	QObject::connect(ui->RightHand_btn_PosGo, SIGNAL(clicked()), form_arm, SLOT(RightArm_PosGo()));
	QObject::connect(ui->RightGripper_btn_Hold, SIGNAL(clicked()), form_arm, SLOT(RightGripper_Hold()));
	QObject::connect(ui->RightGripper_btn_Release, SIGNAL(clicked()), form_arm, SLOT(RightGripper_Release()));
	form_arm->moveToThread(thread_arm);
	thread_arm->start();

	/* Mobile */
	form_mobile = new Form_Mobile(ui);
	thread_mobile = new QThread();
	QObject::connect(ui->Move_btn_Forward, SIGNAL(clicked()), form_mobile, SLOT(MoveForward()));
	QObject::connect(ui->Move_btn_Backward, SIGNAL(clicked()), form_mobile, SLOT(MoveBackward()));
	QObject::connect(ui->Move_btn_Left, SIGNAL(clicked()), form_mobile, SLOT(MoveLeft()));
	QObject::connect(ui->Move_btn_Right, SIGNAL(clicked()), form_mobile, SLOT(MoveRight()));
	QObject::connect(ui->SelfTurn_btn_Left, SIGNAL(clicked()), form_mobile, SLOT(SelfTurn_Left()));
	QObject::connect(ui->SelfTurn_btn_Right, SIGNAL(clicked()), form_mobile, SLOT(SelfTurn_Right()));
	QObject::connect(ui->Oblique_btn_Forward, SIGNAL(clicked()), form_mobile, SLOT(Oblique_Forward()));
	QObject::connect(ui->Oblique_btn_Backward, SIGNAL(clicked()), form_mobile, SLOT(Oblique_Backward()));
	QObject::connect(ui->Turn_btn_Forward, SIGNAL(clicked()), form_mobile, SLOT(Turn_Forward()));
	QObject::connect(ui->Turn_btn_Backward, SIGNAL(clicked()), form_mobile, SLOT(Turn_Backward()));
	QObject::connect(ui->Circle_btn_Forward, SIGNAL(clicked()), form_mobile, SLOT(TurnCircleByRadius_Forward()));
	QObject::connect(ui->Circle_btn_Backward, SIGNAL(clicked()), form_mobile, SLOT(TurnCircleByRadius_Backward()));
	form_mobile->moveToThread(thread_mobile);
	thread_mobile->start();

	/* ForceSensor */
	form_forcesensor = new Form_ForceSensor(ui);
	thread_forcesensor = new QThread();
	QObject::connect(ui->ForceSensor_btn_LeftDataNormalize, SIGNAL(clicked()), form_forcesensor, SLOT(LeftForceSensorNormalize()));
	QObject::connect(ui->ForceSensor_btn_RightDataNormalize, SIGNAL(clicked()), form_forcesensor, SLOT(RightForceSensorNormalize()));
	form_forcesensor->moveToThread(thread_forcesensor);
	thread_forcesensor->start();

	QObject::connect(ui->Xbox_Enable, SIGNAL(stateChanged(int)), this, SLOT(XBoxJoystick_state(int)));
}

Form_Robot::~Form_Robot()
{
	form_head->deleteLater();
	form_arm->deleteLater();
	form_mobile->deleteLater();
	form_forcesensor->deleteLater();
	thread_head->deleteLater();
	thread_arm->deleteLater();
	thread_mobile->deleteLater();
	thread_forcesensor->deleteLater();
	delete ui;
}

void Form_Robot::on_Robot_btn_Reconnect_clicked()
{
	_is_deleted_thread_display = true;
	if (thread_display != nullptr)
	{
		thread_display->join();
		delete thread_display;
	}

	CRobot->Reconnect();
	form_head->SetHeadandLifting(CRobot->CHeadandLifting);
	form_arm->SetArm(CRobot->CLeftArm, CRobot->CRightArm);
	form_mobile->SetMobile(CRobot->CMobile);
	form_forcesensor->SetForceSensor(CRobot->CLeftForceSensor, CRobot->CRightForceSensor);

	_is_deleted_thread_display = false;
	thread_display = new std::thread(&Form_Robot::Display, this);
}

void Form_Robot::on_LeftHand_btn_Start_clicked()
{
	CRobot->CLeftArm->Start();
}

void Form_Robot::on_LeftHand_btn_Stop_clicked()
{
	CRobot->CLeftArm->Stop();
}

void Form_Robot::on_RightHand_btn_Start_clicked()
{
	CRobot->CRightArm->Start();
}

void Form_Robot::on_RightHand_btn_Stop_clicked()
{
	CRobot->CRightArm->Stop();
}

void Form_Robot::on_Move_btn_Start_clicked()
{
	CRobot->CMobile->CSteering->Start();
}

void Form_Robot::on_Move_btn_Stop_clicked()
{
	CRobot->CMobile->CWheel->Stop();
}

void Form_Robot::XBoxJoystick_state(int state)
{
	if (state == 0)
		CXBoxJoystick->CloseXboxJoystick();
	else
		CXBoxJoystick->OpenXboxJoystick();
}

////////////////////////////////////////////////////////////////////////////////
///  Display   /////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void Form_Robot::showEvent(QShowEvent *event)
{
	// Initialize instace and then turn on display
	CRobot = Robot::getRobot();
	form_head->SetHeadandLifting(CRobot->CHeadandLifting);
	form_arm->SetArm(CRobot->CLeftArm, CRobot->CRightArm);
	form_mobile->SetMobile(CRobot->CMobile);
	form_forcesensor->SetForceSensor(CRobot->CLeftForceSensor, CRobot->CRightForceSensor);

	_is_deleted_thread_display = false;
	thread_display = new std::thread(&Form_Robot::Display, this);
}

void Form_Robot::closeEvent(QCloseEvent *event)
{
	_is_deleted_thread_display = true;
	if (thread_display != nullptr)
	{
		thread_display->join();
		delete thread_display;
		thread_display = nullptr;
	}
}

void Form_Robot::Display()
{
	while (!_is_deleted_thread_display)
	{
		if (ui->Correction->currentIndex() == 0)
		{
			form_arm->Display();
			CRobot->CLeftArm->CalculateJacobianMatrix();
			CRobot->CRightArm->CalculateJacobianMatrix();
		}			
		else if (ui->Correction->currentIndex() == 1)
			form_mobile->Display();
		else if (ui->Correction->currentIndex() == 2)
			form_head->Display();
		else if (ui->Correction->currentIndex() == 3)
			form_forcesensor->Display();
		else
			;
		this_thread::sleep_for(chrono::milliseconds(50));
	}
}

void Form_Robot::SetupImages()
{
	string path = string(getenv("PWD")) + "/src/Gui/images/";
	if (std::fstream(path + "up.png").good())
	{
		QIcon up = QIcon(string(path + "up.png").c_str());
		QIcon down = QIcon(string(path + "down.png").c_str());
		QIcon left = QIcon(string(path + "left.png").c_str());
		QIcon right = QIcon(string(path + "right.png").c_str());
		QIcon stop = QIcon(string(path + "stop.png").c_str());
		QIcon start = QIcon(string(path + "start.png").c_str());
		QIcon selfturn_left = QIcon(string(path + "selfturn_left.png").c_str());
		QIcon selfturn_right = QIcon(string(path + "selfturn_right.png").c_str());
		QPixmap oblique = QPixmap(string(path + "oblique.png").c_str());
		QPixmap turn = QPixmap(string(path + "turn.png").c_str());
		QPixmap turncircle = QPixmap(string(path + "turncircle.png").c_str());

		ui->Move_btn_Forward->setIcon(up);
		ui->Move_btn_Backward->setIcon(down);
		ui->Move_btn_Left->setIcon(left);
		ui->Move_btn_Right->setIcon(right);
		ui->Move_btn_Stop->setIcon(stop);
		ui->Move_btn_Start->setIcon(start);
		ui->SelfTurn_btn_Left->setIcon(selfturn_left);
		ui->SelfTurn_btn_Right->setIcon(selfturn_right);
		ui->Oblique_btn_Forward->setIcon(up);
		ui->Oblique_btn_Backward->setIcon(down);
		ui->Oblique_image->setPixmap(oblique);
		ui->Circle_btn_Forward->setIcon(up);
		ui->Circle_btn_Backward->setIcon(down);
		ui->Circle_image->setPixmap(turncircle);
		ui->Turn_btn_Forward->setIcon(up);
		ui->Turn_btn_Backward->setIcon(down);
		ui->Turn_image->setPixmap(turn);
	}
}