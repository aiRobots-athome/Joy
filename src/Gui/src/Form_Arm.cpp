#include "Form_Arm.h"
// Left Arm
void Form_Arm::LeftArm_PosGo()
{
	const float Axis0 = ui->LeftHand_lineEdit_Axis0->text().toFloat();

	const float ox = ui->LeftHand_lineEdit_RX->text().toFloat();
	const float oy = ui->LeftHand_lineEdit_RY->text().toFloat();
	const float oz = ui->LeftHand_lineEdit_RZ->text().toFloat();

	const float px = ui->LeftHand_lineEdit_X->text().toFloat();
	const float py = ui->LeftHand_lineEdit_Y->text().toFloat();
	const float pz = ui->LeftHand_lineEdit_Z->text().toFloat();

	CLeftArm->GotoPosition(Axis0, ox, oy, oz, px, py, pz);
}

void Form_Arm::LeftArm_Initial()
{
	CLeftArm->ResetAllMotorAngle();
}

void Form_Arm::LeftGripper_Hold()
{
	const float Angle = ui->LeftGripper_lineEdit_Angle->text().toFloat();
	CLeftArm->GraspObj(Angle);
}

void Form_Arm::LeftGripper_Release()
{
	CLeftArm->ReleaseObj();
}

// Right Arm
void Form_Arm::RightArm_PosGo()
{
	const float Axis0 = ui->RightHand_lineEdit_Axis0->text().toFloat();

	const float ox = ui->RightHand_lineEdit_RX->text().toFloat();
	const float oy = ui->RightHand_lineEdit_RY->text().toFloat();
	const float oz = ui->RightHand_lineEdit_RZ->text().toFloat();

	const float px = ui->RightHand_lineEdit_X->text().toFloat();
	const float py = ui->RightHand_lineEdit_Y->text().toFloat();
	const float pz = ui->RightHand_lineEdit_Z->text().toFloat();

	CRightArm->GotoPosition(Axis0, ox, oy, oz, px, py, pz);
}

void Form_Arm::RightArm_Initial()
{
	CRightArm->ResetAllMotorAngle();
}

void Form_Arm::RightGripper_Hold()
{
	const float Angle = ui->RightGripper_lineEdit_Angle->text().toFloat();
	CRightArm->GraspObj(Angle);
}

void Form_Arm::RightGripper_Release()
{
	CRightArm->ReleaseObj();
}

void Form_Arm::Display()
{
	/* LeftArm */
	// ID
	ui->Leftarm_label_ID->setText(QString::number(CLeftArm->GetMotor_ID(0)));
	ui->Leftarm_label_ID_2->setText(QString::number(CLeftArm->GetMotor_ID(1)));
	ui->Leftarm_label_ID_3->setText(QString::number(CLeftArm->GetMotor_ID(2)));
	ui->Leftarm_label_ID_4->setText(QString::number(CLeftArm->GetMotor_ID(3)));
	ui->Leftarm_label_ID_5->setText(QString::number(CLeftArm->GetMotor_ID(4)));
	ui->Leftarm_label_ID_6->setText(QString::number(CLeftArm->GetMotor_ID(5)));
	ui->Leftarm_label_ID_7->setText(QString::number(CLeftArm->GetMotor_ID(6)));
	ui->Leftarm_label_ID_8->setText(QString::number(CLeftArm->GetMotor_ID(7)));
	isOK(CLeftArm->GetMotor_Connected(0), ui->Leftarm_label_ID);
	isOK(CLeftArm->GetMotor_Connected(1), ui->Leftarm_label_ID_2);
	isOK(CLeftArm->GetMotor_Connected(2), ui->Leftarm_label_ID_3);
	isOK(CLeftArm->GetMotor_Connected(3), ui->Leftarm_label_ID_4);
	isOK(CLeftArm->GetMotor_Connected(4), ui->Leftarm_label_ID_5);
	isOK(CLeftArm->GetMotor_Connected(5), ui->Leftarm_label_ID_6);
	isOK(CLeftArm->GetMotor_Connected(6), ui->Leftarm_label_ID_7);
	isOK(CLeftArm->GetMotor_Connected(7), ui->Leftarm_label_ID_8);
	// Angle
	ui->Leftarm_label_PresentAngle->setText(QString::number((int)CLeftArm->GetMotor_PresentAngle(0)));
	ui->Leftarm_label_PresentAngle_2->setText(QString::number((int)CLeftArm->GetMotor_PresentAngle(1)));
	ui->Leftarm_label_PresentAngle_3->setText(QString::number((int)CLeftArm->GetMotor_PresentAngle(2)));
	ui->Leftarm_label_PresentAngle_4->setText(QString::number((int)CLeftArm->GetMotor_PresentAngle(3)));
	ui->Leftarm_label_PresentAngle_5->setText(QString::number((int)CLeftArm->GetMotor_PresentAngle(4)));
	ui->Leftarm_label_PresentAngle_6->setText(QString::number((int)CLeftArm->GetMotor_PresentAngle(5)));
	ui->Leftarm_label_PresentAngle_7->setText(QString::number((int)CLeftArm->GetMotor_PresentAngle(6)));
	ui->Leftarm_label_PresentAngle_8->setText(QString::number((int)CLeftArm->GetMotor_PresentAngle(7)));
	//Velocity
	ui->Leftarm_label_PresentVelocity->setText(QString::number((int)CLeftArm->GetMotor_Velocity(0)));
	ui->Leftarm_label_PresentVelocity_2->setText(QString::number((int)CLeftArm->GetMotor_Velocity(1)));
	ui->Leftarm_label_PresentVelocity_3->setText(QString::number((int)CLeftArm->GetMotor_Velocity(2)));
	ui->Leftarm_label_PresentVelocity_4->setText(QString::number((int)CLeftArm->GetMotor_Velocity(3)));
	ui->Leftarm_label_PresentVelocity_5->setText(QString::number((int)CLeftArm->GetMotor_Velocity(4)));
	ui->Leftarm_label_PresentVelocity_6->setText(QString::number((int)CLeftArm->GetMotor_Velocity(5)));
	ui->Leftarm_label_PresentVelocity_7->setText(QString::number((int)CLeftArm->GetMotor_Velocity(6)));
	ui->Leftarm_label_PresentVelocity_8->setText(QString::number((int)CLeftArm->GetMotor_Velocity(7)));
	// Torque
	ui->Leftarm_label_PresentTorque->setText(QString::number((int)CLeftArm->GetMotor_PresentTorque(0)));
	ui->Leftarm_label_PresentTorque_2->setText(QString::number((int)CLeftArm->GetMotor_PresentTorque(1)));
	ui->Leftarm_label_PresentTorque_3->setText(QString::number((int)CLeftArm->GetMotor_PresentTorque(2)));
	ui->Leftarm_label_PresentTorque_4->setText(QString::number((int)CLeftArm->GetMotor_PresentTorque(3)));
	ui->Leftarm_label_PresentTorque_5->setText(QString::number((int)CLeftArm->GetMotor_PresentTorque(4)));
	ui->Leftarm_label_PresentTorque_6->setText(QString::number((int)CLeftArm->GetMotor_PresentTorque(5)));
	ui->Leftarm_label_PresentTorque_7->setText(QString::number((int)CLeftArm->GetMotor_PresentTorque(6)));
	ui->Leftarm_label_PresentTorque_8->setText(QString::number((int)CLeftArm->GetMotor_PresentTorque(7)));
	isOK((abs(CLeftArm->GetMotor_PresentTorque(0)) < torque_threshold) ? true : false, ui->Leftarm_label_PresentTorque);
	isOK((abs(CLeftArm->GetMotor_PresentTorque(1)) < torque_threshold) ? true : false, ui->Leftarm_label_PresentTorque_2);
	isOK((abs(CLeftArm->GetMotor_PresentTorque(2)) < torque_threshold) ? true : false, ui->Leftarm_label_PresentTorque_3);
	isOK((abs(CLeftArm->GetMotor_PresentTorque(3)) < torque_threshold) ? true : false, ui->Leftarm_label_PresentTorque_4);
	isOK((abs(CLeftArm->GetMotor_PresentTorque(4)) < torque_threshold) ? true : false, ui->Leftarm_label_PresentTorque_5);
	isOK((abs(CLeftArm->GetMotor_PresentTorque(5)) < torque_threshold) ? true : false, ui->Leftarm_label_PresentTorque_6);
	isOK((abs(CLeftArm->GetMotor_PresentTorque(6)) < torque_threshold) ? true : false, ui->Leftarm_label_PresentTorque_7);
	isOK((abs(CLeftArm->GetMotor_PresentTorque(7)) < torque_threshold) ? true : false, ui->Leftarm_label_PresentTorque_8);

	/* RightArm */
	// ID
	ui->Rightarm_label_ID->setText(QString::number(CRightArm->GetMotor_ID(0)));
	ui->Rightarm_label_ID_2->setText(QString::number(CRightArm->GetMotor_ID(1)));
	ui->Rightarm_label_ID_3->setText(QString::number(CRightArm->GetMotor_ID(2)));
	ui->Rightarm_label_ID_4->setText(QString::number(CRightArm->GetMotor_ID(3)));
	ui->Rightarm_label_ID_5->setText(QString::number(CRightArm->GetMotor_ID(4)));
	ui->Rightarm_label_ID_6->setText(QString::number(CRightArm->GetMotor_ID(5)));
	ui->Rightarm_label_ID_7->setText(QString::number(CRightArm->GetMotor_ID(6)));
	ui->Rightarm_label_ID_8->setText(QString::number(CRightArm->GetMotor_ID(7)));
	isOK(CRightArm->GetMotor_Connected(0), ui->Rightarm_label_ID);
	isOK(CRightArm->GetMotor_Connected(1), ui->Rightarm_label_ID_2);
	isOK(CRightArm->GetMotor_Connected(2), ui->Rightarm_label_ID_3);
	isOK(CRightArm->GetMotor_Connected(3), ui->Rightarm_label_ID_4);
	isOK(CRightArm->GetMotor_Connected(4), ui->Rightarm_label_ID_5);
	isOK(CRightArm->GetMotor_Connected(5), ui->Rightarm_label_ID_6);
	isOK(CRightArm->GetMotor_Connected(6), ui->Rightarm_label_ID_7);
	isOK(CRightArm->GetMotor_Connected(7), ui->Rightarm_label_ID_8);
	// Angle
	ui->Rightarm_label_PresentAngle->setText(QString::number((int)CRightArm->GetMotor_PresentAngle(0)));
	ui->Rightarm_label_PresentAngle_2->setText(QString::number((int)CRightArm->GetMotor_PresentAngle(1)));
	ui->Rightarm_label_PresentAngle_3->setText(QString::number((int)CRightArm->GetMotor_PresentAngle(2)));
	ui->Rightarm_label_PresentAngle_4->setText(QString::number((int)CRightArm->GetMotor_PresentAngle(3)));
	ui->Rightarm_label_PresentAngle_5->setText(QString::number((int)CRightArm->GetMotor_PresentAngle(4)));
	ui->Rightarm_label_PresentAngle_6->setText(QString::number((int)CRightArm->GetMotor_PresentAngle(5)));
	ui->Rightarm_label_PresentAngle_7->setText(QString::number((int)CRightArm->GetMotor_PresentAngle(6)));
	ui->Rightarm_label_PresentAngle_8->setText(QString::number((int)CRightArm->GetMotor_PresentAngle(7)));
	// Velocity
	ui->Rightarm_label_PresentVelocity->setText(QString::number((int)CRightArm->GetMotor_Velocity(0)));
	ui->Rightarm_label_PresentVelocity_2->setText(QString::number((int)CRightArm->GetMotor_Velocity(1)));
	ui->Rightarm_label_PresentVelocity_3->setText(QString::number((int)CRightArm->GetMotor_Velocity(2)));
	ui->Rightarm_label_PresentVelocity_4->setText(QString::number((int)CRightArm->GetMotor_Velocity(3)));
	ui->Rightarm_label_PresentVelocity_5->setText(QString::number((int)CRightArm->GetMotor_Velocity(4)));
	ui->Rightarm_label_PresentVelocity_6->setText(QString::number((int)CRightArm->GetMotor_Velocity(5)));
	ui->Rightarm_label_PresentVelocity_7->setText(QString::number((int)CRightArm->GetMotor_Velocity(6)));
	ui->Rightarm_label_PresentVelocity_8->setText(QString::number((int)CRightArm->GetMotor_Velocity(7)));
	// Torque
	ui->Rightarm_label_PresentTorque->setText(QString::number((int)CRightArm->GetMotor_PresentTorque(0)));
	ui->Rightarm_label_PresentTorque_2->setText(QString::number((int)CRightArm->GetMotor_PresentTorque(1)));
	ui->Rightarm_label_PresentTorque_3->setText(QString::number((int)CRightArm->GetMotor_PresentTorque(2)));
	ui->Rightarm_label_PresentTorque_4->setText(QString::number((int)CRightArm->GetMotor_PresentTorque(3)));
	ui->Rightarm_label_PresentTorque_5->setText(QString::number((int)CRightArm->GetMotor_PresentTorque(4)));
	ui->Rightarm_label_PresentTorque_6->setText(QString::number((int)CRightArm->GetMotor_PresentTorque(5)));
	ui->Rightarm_label_PresentTorque_7->setText(QString::number((int)CRightArm->GetMotor_PresentTorque(6)));
	ui->Rightarm_label_PresentTorque_8->setText(QString::number((int)CRightArm->GetMotor_PresentTorque(7)));
	isOK((abs(CRightArm->GetMotor_PresentTorque(0)) < torque_threshold) ? true : false, ui->Rightarm_label_PresentTorque);
	isOK((abs(CRightArm->GetMotor_PresentTorque(1)) < torque_threshold) ? true : false, ui->Rightarm_label_PresentTorque_2);
	isOK((abs(CRightArm->GetMotor_PresentTorque(2)) < torque_threshold) ? true : false, ui->Rightarm_label_PresentTorque_3);
	isOK((abs(CRightArm->GetMotor_PresentTorque(3)) < torque_threshold) ? true : false, ui->Rightarm_label_PresentTorque_4);
	isOK((abs(CRightArm->GetMotor_PresentTorque(4)) < torque_threshold) ? true : false, ui->Rightarm_label_PresentTorque_5);
	isOK((abs(CRightArm->GetMotor_PresentTorque(5)) < torque_threshold) ? true : false, ui->Rightarm_label_PresentTorque_6);
	isOK((abs(CRightArm->GetMotor_PresentTorque(6)) < torque_threshold) ? true : false, ui->Rightarm_label_PresentTorque_7);
	isOK((abs(CRightArm->GetMotor_PresentTorque(7)) < torque_threshold) ? true : false, ui->Rightarm_label_PresentTorque_8);
}

void Form_Arm::isOK(bool checked_thing, QLabel *label)
{
	if (checked_thing == true)
		Green(label);
	else
		Red(label);
}

void Form_Arm::Red(QLabel *label)
{
	QMetaObject::invokeMethod(label, "setStyleSheet", Qt::QueuedConnection, Q_ARG(QString, "QLabel { background-color: rgb(255, 60, 63)}"));
}

void Form_Arm::Green(QLabel *label)
{
	QMetaObject::invokeMethod(label, "setStyleSheet", Qt::QueuedConnection, Q_ARG(QString, "QLabel { background-color: rgb(170, 255, 127)}"));
}