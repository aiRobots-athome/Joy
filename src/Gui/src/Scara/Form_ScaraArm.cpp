#include "Scara/Form_ScaraArm.h"

void Form_ScaraArm::on_ScaraArm_btn_PosGo_clicked()
{
	const float oz = ui->ScaraArm_lineEdit_Oz->text().toFloat();

	const float px = ui->ScaraArm_lineEdit_X->text().toFloat();
	const float py = ui->ScaraArm_lineEdit_Y->text().toFloat();

	CScaraArm->TrajectoryPlanning(oz, px, py, 1);
}

void Form_ScaraArm::on_Scara_btn_Reset_clicked()
{
	CScaraArm->Reset();
}

void Form_ScaraArm::on_Screw_btn_Up_clicked()
{
	CScaraArm->GoScrewHeight(CScaraArm->GetPresentHeight() + 1);
}

void Form_ScaraArm::on_Screw_btn_Down_clicked()
{
	CScaraArm->GoScrewHeight(CScaraArm->GetPresentHeight() - 1);
}

void Form_ScaraArm::on_Goal_Height_btn_clicked()
{
	const float height = ui->Screw_lineEdit_Height->text().toFloat();
	CScaraArm->GoScrewHeight(height);
}

void Form_ScaraArm::Display()
{
	CScaraArm->CalculateJacobianMatrix();

	// ID
	ui->ScaraArm_label_ID->setText(QString::number(CScaraArm->GetMotor_ID(0)));
	ui->ScaraArm_label_ID_2->setText(QString::number(CScaraArm->GetMotor_ID(1)));
	ui->ScaraArm_label_ID_3->setText(QString::number(CScaraArm->GetMotor_ID(2)));
	ui->ScaraArm_label_ID_4->setText(QString::number(CScaraArm->GetMotor_ID(3)));
	isOK(CScaraArm->GetMotor_Connected(0), ui->ScaraArm_label_ID);
	isOK(CScaraArm->GetMotor_Connected(1), ui->ScaraArm_label_ID_2);
	isOK(CScaraArm->GetMotor_Connected(2), ui->ScaraArm_label_ID_3);
	isOK(CScaraArm->GetMotor_Connected(3), ui->ScaraArm_label_ID_4);

	// Angle
	ui->ScaraArm_label_PresentAngle->setText(QString::number((int)CScaraArm->GetMotor_PresentAngle(0)));
	ui->ScaraArm_label_PresentAngle_2->setText(QString::number((int)CScaraArm->GetMotor_PresentAngle(1)));
	ui->ScaraArm_label_PresentAngle_3->setText(QString::number((int)CScaraArm->GetMotor_PresentAngle(2)));
	ui->ScaraArm_label_PresentAngle_4->setText(QString::number((int)CScaraArm->GetMotor_PresentAngle(3)));

	//Velocity
	ui->ScaraArm_label_PresentVelocity->setText(QString::number((int)CScaraArm->GetMotor_Velocity(0)));
	ui->ScaraArm_label_PresentVelocity_2->setText(QString::number((int)CScaraArm->GetMotor_Velocity(1)));
	ui->ScaraArm_label_PresentVelocity_3->setText(QString::number((int)CScaraArm->GetMotor_Velocity(2)));
	ui->ScaraArm_label_PresentVelocity_4->setText(QString::number((int)CScaraArm->GetMotor_Velocity(3)));

	// Torque
	ui->ScaraArm_label_PresentTorque->setText(QString::number((int)CScaraArm->GetMotor_PresentTorque(0)));
	ui->ScaraArm_label_PresentTorque_2->setText(QString::number((int)CScaraArm->GetMotor_PresentTorque(1)));
	ui->ScaraArm_label_PresentTorque_3->setText(QString::number((int)CScaraArm->GetMotor_PresentTorque(2)));
	ui->ScaraArm_label_PresentTorque_4->setText(QString::number((int)CScaraArm->GetMotor_PresentTorque(3)));

	isOK((abs(CScaraArm->GetMotor_PresentTorque(0)) < torque_threshold) ? true : false, ui->ScaraArm_label_PresentTorque);
	isOK((abs(CScaraArm->GetMotor_PresentTorque(1)) < torque_threshold) ? true : false, ui->ScaraArm_label_PresentTorque_2);
	isOK((abs(CScaraArm->GetMotor_PresentTorque(2)) < torque_threshold) ? true : false, ui->ScaraArm_label_PresentTorque_3);
	isOK((abs(CScaraArm->GetMotor_PresentTorque(3)) < torque_threshold) ? true : false, ui->ScaraArm_label_PresentTorque_4);

	// forward kinematics
	ui->ScaraArm_label_PresentX->setText(QString::number((float)CScaraArm->GetCurrentPosition(0)));
	ui->ScaraArm_label_PresentY->setText(QString::number((float)CScaraArm->GetCurrentPosition(1)));
	ui->ScaraArm_label_PresentZ->setText(QString::number((float)CScaraArm->GetPresentHeight()));
	ui->ScaraArm_label_PresentOz->setText(QString::number((float)CScaraArm->GetCurrentOrientation(2)));

	if (CScaraArm->GetWorkingState())
		QMetaObject::invokeMethod(ui->Tab_ScaraArm, "setStyleSheet", Qt::QueuedConnection, Q_ARG(QString, "QWidget { background-color: rgb(255, 200, 200)}"));
	else
		QMetaObject::invokeMethod(ui->Tab_ScaraArm, "setStyleSheet", Qt::QueuedConnection, Q_ARG(QString, "QWidget { background-color: rgb(255, 255, 255)}"));	
}

void Form_ScaraArm::isOK(bool checked_thing, QLabel *label)
{
	if (checked_thing == true)
		Green(label);
	else
		Red(label);
}

void Form_ScaraArm::Red(QLabel *label)
{
	QMetaObject::invokeMethod(label, "setStyleSheet", Qt::QueuedConnection, Q_ARG(QString, "QLabel { background-color: rgb(255, 60, 63)}"));
}

void Form_ScaraArm::Green(QLabel *label)
{
	QMetaObject::invokeMethod(label, "setStyleSheet", Qt::QueuedConnection, Q_ARG(QString, "QLabel { background-color: rgb(170, 255, 127)}"));
}