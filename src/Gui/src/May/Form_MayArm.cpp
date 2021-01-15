#include "May/Form_MayArm.h"

int Form_MayArm::time_tick_ = 0;

void Form_MayArm::PositionGo()
{
	const float ox = ui->May_lineEdit_Ox->text().toFloat();
	const float oy = ui->May_lineEdit_Oy->text().toFloat();
	const float oz = ui->May_lineEdit_Oz->text().toFloat();

	const float px = ui->May_lineEdit_X->text().toFloat();
	const float py = ui->May_lineEdit_Y->text().toFloat();
	const float pz = ui->May_lineEdit_Z->text().toFloat();

	CMay->TrajectoryPlanning(ox, oy, oz, px, py, pz);
}

void Form_MayArm::ImpedanceGo()
{
	const float om = ui->May_lineEdit_angularM->text().toFloat();
	const float ob = ui->May_lineEdit_angularB->text().toFloat();
	const float ok = ui->May_lineEdit_angularK->text().toFloat();

	const float pm = ui->May_lineEdit_linearM->text().toFloat();
	const float pb = ui->May_lineEdit_linearB->text().toFloat();
	const float pk = ui->May_lineEdit_linearK->text().toFloat();

	const float ox = ui->May_lineEdit_Ox->text().toFloat();
	const float oy = ui->May_lineEdit_Oy->text().toFloat();
	const float oz = ui->May_lineEdit_Oz->text().toFloat();

	const float px = ui->May_lineEdit_X->text().toFloat();
	const float py = ui->May_lineEdit_Y->text().toFloat();
	const float pz = ui->May_lineEdit_Z->text().toFloat();

	CMay->ImpedanceControl(om, ob, ok, pm, pb, pk, ox, oy, oz, px, py, pz);
}

void Form_MayArm::GripperHold()
{
	CMay->PneumaticOn();
}

void Form_MayArm::GripperRelease()
{
	CMay->PneumaticOff();
}

void Form_MayArm::Display()
{
	// time tick used to display animation
	time_tick_++;
	time_tick_ %= 20;

	// ID
	ui->May_label_ID_0->setText(QString::number(CMay->GetMotor_ID(0)));
	ui->May_label_ID_1->setText(QString::number(CMay->GetMotor_ID(1)));
	ui->May_label_ID_2->setText(QString::number(CMay->GetMotor_ID(2)));
	ui->May_label_ID_3->setText(QString::number(CMay->GetMotor_ID(3)));
	ui->May_label_ID_4->setText(QString::number(CMay->GetMotor_ID(4)));
	ui->May_label_ID_5->setText(QString::number(CMay->GetMotor_ID(5)));

	isOK(CMay->GetMotor_Connected(0), ui->May_label_ID_0);
	isOK(CMay->GetMotor_Connected(1), ui->May_label_ID_1);
	isOK(CMay->GetMotor_Connected(2), ui->May_label_ID_2);
	isOK(CMay->GetMotor_Connected(3), ui->May_label_ID_3);
	isOK(CMay->GetMotor_Connected(4), ui->May_label_ID_4);
	isOK(CMay->GetMotor_Connected(5), ui->May_label_ID_5);

	// Angle
	ui->May_label_angle_0->setText(QString::number((int)CMay->GetMotor_PresentAngle(0)));
	ui->May_label_angle_1->setText(QString::number((int)CMay->GetMotor_PresentAngle(1)));
	ui->May_label_angle_2->setText(QString::number((int)CMay->GetMotor_PresentAngle(2)));
	ui->May_label_angle_3->setText(QString::number((int)CMay->GetMotor_PresentAngle(3)));
	ui->May_label_angle_4->setText(QString::number((int)CMay->GetMotor_PresentAngle(4)));
	ui->May_label_angle_5->setText(QString::number((int)CMay->GetMotor_PresentAngle(5)));

	//Velocity
	ui->May_label_velocity_0->setText(QString::number((int)CMay->GetMotor_Velocity(0)));
	ui->May_label_velocity_1->setText(QString::number((int)CMay->GetMotor_Velocity(1)));
	ui->May_label_velocity_2->setText(QString::number((int)CMay->GetMotor_Velocity(2)));
	ui->May_label_velocity_3->setText(QString::number((int)CMay->GetMotor_Velocity(3)));
	ui->May_label_velocity_4->setText(QString::number((int)CMay->GetMotor_Velocity(4)));
	ui->May_label_velocity_5->setText(QString::number((int)CMay->GetMotor_Velocity(5)));

	// Torque
	ui->May_label_torque_0->setText(QString::number((int)CMay->GetMotor_PresentTorque(0)));
	ui->May_label_torque_1->setText(QString::number((int)CMay->GetMotor_PresentTorque(1)));
	ui->May_label_torque_2->setText(QString::number((int)CMay->GetMotor_PresentTorque(2)));
	ui->May_label_torque_3->setText(QString::number((int)CMay->GetMotor_PresentTorque(3)));
	ui->May_label_torque_4->setText(QString::number((int)CMay->GetMotor_PresentTorque(4)));
	ui->May_label_torque_5->setText(QString::number((int)CMay->GetMotor_PresentTorque(5)));

	isOK((abs(CMay->GetMotor_PresentTorque(0)) < torque_threshold) ? true : false, ui->May_label_torque_0);
	isOK((abs(CMay->GetMotor_PresentTorque(1)) < torque_threshold) ? true : false, ui->May_label_torque_1);
	isOK((abs(CMay->GetMotor_PresentTorque(2)) < torque_threshold) ? true : false, ui->May_label_torque_2);
	isOK((abs(CMay->GetMotor_PresentTorque(3)) < torque_threshold) ? true : false, ui->May_label_torque_3);
	isOK((abs(CMay->GetMotor_PresentTorque(4)) < torque_threshold) ? true : false, ui->May_label_torque_4);
	isOK((abs(CMay->GetMotor_PresentTorque(5)) < torque_threshold) ? true : false, ui->May_label_torque_5);

	// forward kinematics
	ui->May_label_currentPosX->setText(QString::number((int)CMay->GetCurrentPosition(0)));
	ui->May_label_currentPosY->setText(QString::number((int)CMay->GetCurrentPosition(1)));
	ui->May_label_currentPosZ->setText(QString::number((int)CMay->GetCurrentPosition(2)));
	ui->May_label_currentRotX->setText(QString::number((int)CMay->GetCurrentOrientation(0)));
	ui->May_label_currentRotY->setText(QString::number((int)CMay->GetCurrentOrientation(1)));
	ui->May_label_currentRotZ->setText(QString::number((int)CMay->GetCurrentOrientation(2)));

	if (CMay->GetWorkingState())
	{
		/* Full description: "QWidget { background-color: rgb(255, 240, 240)}" */
		int rgb_value = 150 + time_tick_ * 5;
		std::string background_name = "QWidget { background-color: rgb(255, " + to_string(rgb_value) + ", " + to_string(rgb_value) + ")}";

		QMetaObject::invokeMethod(ui->Tab_May, "setStyleSheet", Qt::QueuedConnection, Q_ARG(QString, QString::fromStdString(background_name)));
	}
		
	else
		QMetaObject::invokeMethod(ui->Tab_May, "setStyleSheet", Qt::QueuedConnection, Q_ARG(QString, "QWidget { background-color: rgb(255, 255, 255)}"));
}

void Form_MayArm::isOK(bool checked_thing, QLabel *label)
{
	if (checked_thing == true)
		Green(label);
	else
		Red(label);
}

void Form_MayArm::Red(QLabel *label)
{
	QMetaObject::invokeMethod(label, "setStyleSheet", Qt::QueuedConnection, Q_ARG(QString, "QLabel { background-color: rgb(255, 60, 63)}"));
}

void Form_MayArm::Green(QLabel *label)
{
	QMetaObject::invokeMethod(label, "setStyleSheet", Qt::QueuedConnection, Q_ARG(QString, "QLabel { background-color: rgb(170, 255, 127)}"));
}