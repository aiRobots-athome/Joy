#include "Robot/Form_Mobile.h"
void Form_Mobile::MoveForward()
{
	const float distance = ui->Move_lineEdit_Distance->text().toFloat();
	const float velocity = ui->MobileControl_lineEdit_Velocity->text().toFloat();
	CMobile->MoveForward(distance, velocity);
}

void Form_Mobile::MoveBackward()
{
	const float distance = ui->Move_lineEdit_Distance->text().toFloat();
	const float velocity = ui->MobileControl_lineEdit_Velocity->text().toFloat();
	CMobile->MoveBackward(distance, velocity);
}

void Form_Mobile::MoveLeft()
{
	const float distance = ui->Move_lineEdit_Distance->text().toFloat();
	const float velocity = ui->MobileControl_lineEdit_Velocity->text().toFloat();
	CMobile->MoveLeft(distance, velocity);
}

void Form_Mobile::MoveRight()
{
	const float distance = ui->Move_lineEdit_Distance->text().toFloat();
	const float velocity = ui->MobileControl_lineEdit_Velocity->text().toFloat();
	CMobile->MoveRight(distance, velocity);
}

void Form_Mobile::SelfTurn_Left()
{
	const float direction = ui->SelfTurn_lineEdit_Direction->text().toFloat();
	const float velocity = ui->MobileControl_lineEdit_Velocity->text().toFloat();
	CMobile->SelfTurn(abs(direction), abs(velocity));
}

void Form_Mobile::SelfTurn_Right()
{
	const float direction = ui->SelfTurn_lineEdit_Direction->text().toFloat();
	const float velocity = ui->MobileControl_lineEdit_Velocity->text().toFloat();
	CMobile->SelfTurn(abs(direction), -abs(velocity));
}

void Form_Mobile::Oblique_Forward()
{
	const float distance = ui->Oblique_lineEdit_Distance->text().toFloat();
	const float velocity = ui->MobileControl_lineEdit_Velocity->text().toFloat();
	const float direction = ui->Oblique_lineEdit_Direction->text().toFloat();
	CMobile->Move(distance, abs(velocity), direction);
}

void Form_Mobile::Oblique_Backward()
{
	const float distance = ui->Oblique_lineEdit_Distance->text().toFloat();
	const float velocity = ui->MobileControl_lineEdit_Velocity->text().toFloat();
	const float direction = ui->Oblique_lineEdit_Direction->text().toFloat();
	CMobile->Move(distance, -abs(velocity), direction);
}

void Form_Mobile::Turn_Forward()
{
	const float direction = ui->Turn_lineEdit_Direction->text().toFloat();
	const float velocity = ui->MobileControl_lineEdit_Velocity->text().toFloat();
	const float distance = ui->Turn_lineEdit_Distance->text().toFloat();
	CMobile->Turn(direction, distance, abs(velocity));
}

void Form_Mobile::Turn_Backward()
{
	const float direction = ui->Turn_lineEdit_Direction->text().toFloat();
	const float velocity = ui->MobileControl_lineEdit_Velocity->text().toFloat();
	const float distance = ui->Turn_lineEdit_Distance->text().toFloat();
	CMobile->Turn(direction, distance, -abs(velocity));
}

void Form_Mobile::TurnCircleByRadius_Forward()
{
	const float radius = ui->Circle_lineEdit_Radius->text().toFloat();
	const float velocity = ui->MobileControl_lineEdit_Velocity->text().toFloat();
	const float distance = ui->Circle_lineEdit_Distance->text().toFloat();
	CMobile->TurnCircleByRadius(radius, distance, abs(velocity));
}

void Form_Mobile::TurnCircleByRadius_Backward()
{
	const float radius = ui->Circle_lineEdit_Radius->text().toFloat();
	const float velocity = ui->MobileControl_lineEdit_Velocity->text().toFloat();
	const float distance = ui->Circle_lineEdit_Distance->text().toFloat();
	CMobile->TurnCircleByRadius(radius, distance, -abs(velocity));
}

void Form_Mobile::Display()
{
	/* Wheel */
	// ID
	ui->Wheel_label_ID->setText(QString::number(CWheel->GetMotor_ID(0)));
	ui->Wheel_label_ID_2->setText(QString::number(CWheel->GetMotor_ID(1)));
	ui->Wheel_label_ID_3->setText(QString::number(CWheel->GetMotor_ID(2)));
	ui->Wheel_label_ID_4->setText(QString::number(CWheel->GetMotor_ID(3)));
	isOK(CWheel->GetMotor_Connected(0), ui->Wheel_label_ID);
	isOK(CWheel->GetMotor_Connected(1), ui->Wheel_label_ID_2);
	isOK(CWheel->GetMotor_Connected(2), ui->Wheel_label_ID_3);
	isOK(CWheel->GetMotor_Connected(3), ui->Wheel_label_ID_4);
	// Angle
	ui->Wheel_label_PresentAngle->setText(QString::number((int)CWheel->GetMotor_PresentAngle(0)));
	ui->Wheel_label_PresentAngle_2->setText(QString::number((int)CWheel->GetMotor_PresentAngle(1)));
	ui->Wheel_label_PresentAngle_3->setText(QString::number((int)CWheel->GetMotor_PresentAngle(2)));
	ui->Wheel_label_PresentAngle_4->setText(QString::number((int)CWheel->GetMotor_PresentAngle(3)));
	//Velocity
	ui->Wheel_label_PresentVelocity->setText(QString::number((int)CWheel->GetMotor_Velocity(0)));
	ui->Wheel_label_PresentVelocity_2->setText(QString::number((int)CWheel->GetMotor_Velocity(1)));
	ui->Wheel_label_PresentVelocity_3->setText(QString::number((int)CWheel->GetMotor_Velocity(2)));
	ui->Wheel_label_PresentVelocity_4->setText(QString::number((int)CWheel->GetMotor_Velocity(3)));
	// Torque
	ui->Wheel_label_PresentTorque->setText(QString::number((int)CWheel->GetMotor_PresentTorque(0)));
	ui->Wheel_label_PresentTorque_2->setText(QString::number((int)CWheel->GetMotor_PresentTorque(1)));
	ui->Wheel_label_PresentTorque_3->setText(QString::number((int)CWheel->GetMotor_PresentTorque(2)));
	ui->Wheel_label_PresentTorque_4->setText(QString::number((int)CWheel->GetMotor_PresentTorque(3)));
	isOK((abs(CWheel->GetMotor_PresentTorque(0)) < torque_threshold) ? true : false, ui->Wheel_label_PresentTorque);
	isOK((abs(CWheel->GetMotor_PresentTorque(1)) < torque_threshold) ? true : false, ui->Wheel_label_PresentTorque_2);
	isOK((abs(CWheel->GetMotor_PresentTorque(2)) < torque_threshold) ? true : false, ui->Wheel_label_PresentTorque_3);
	isOK((abs(CWheel->GetMotor_PresentTorque(3)) < torque_threshold) ? true : false, ui->Wheel_label_PresentTorque_4);

	/* Steering */
	// ID
	ui->Steering_label_ID->setText(QString::number(CSteering->GetMotor_ID(0)));
	ui->Steering_label_ID_2->setText(QString::number(CSteering->GetMotor_ID(1)));
	ui->Steering_label_ID_3->setText(QString::number(CSteering->GetMotor_ID(2)));
	ui->Steering_label_ID_4->setText(QString::number(CSteering->GetMotor_ID(3)));
	isOK(CSteering->GetMotor_Connected(0), ui->Steering_label_ID);
	isOK(CSteering->GetMotor_Connected(1), ui->Steering_label_ID_2);
	isOK(CSteering->GetMotor_Connected(2), ui->Steering_label_ID_3);
	isOK(CSteering->GetMotor_Connected(3), ui->Steering_label_ID_4);
	// Angle
	ui->Steering_label_PresentAngle->setText(QString::number((int)CSteering->GetMotor_PresentAngle(0)));
	ui->Steering_label_PresentAngle_2->setText(QString::number((int)CSteering->GetMotor_PresentAngle(1)));
	ui->Steering_label_PresentAngle_3->setText(QString::number((int)CSteering->GetMotor_PresentAngle(2)));
	ui->Steering_label_PresentAngle_4->setText(QString::number((int)CSteering->GetMotor_PresentAngle(3)));
	// Velocity
	ui->Steering_label_PresentVelocity->setText(QString::number((int)CSteering->GetMotor_Velocity(0)));
	ui->Steering_label_PresentVelocity_2->setText(QString::number((int)CSteering->GetMotor_Velocity(1)));
	ui->Steering_label_PresentVelocity_3->setText(QString::number((int)CSteering->GetMotor_Velocity(2)));
	ui->Steering_label_PresentVelocity_4->setText(QString::number((int)CSteering->GetMotor_Velocity(3)));
	// Torque
	ui->Steering_label_PresentTorque->setText(QString::number((int)CSteering->GetMotor_PresentTorque(0)));
	ui->Steering_label_PresentTorque_2->setText(QString::number((int)CSteering->GetMotor_PresentTorque(1)));
	ui->Steering_label_PresentTorque_3->setText(QString::number((int)CSteering->GetMotor_PresentTorque(2)));
	ui->Steering_label_PresentTorque_4->setText(QString::number((int)CSteering->GetMotor_PresentTorque(3)));
	isOK((abs(CSteering->GetMotor_PresentTorque(0)) < torque_threshold) ? true : false, ui->Steering_label_PresentTorque);
	isOK((abs(CSteering->GetMotor_PresentTorque(1)) < torque_threshold) ? true : false, ui->Steering_label_PresentTorque_2);
	isOK((abs(CSteering->GetMotor_PresentTorque(2)) < torque_threshold) ? true : false, ui->Steering_label_PresentTorque_3);
	isOK((abs(CSteering->GetMotor_PresentTorque(3)) < torque_threshold) ? true : false, ui->Steering_label_PresentTorque_4);
}

void Form_Mobile::isOK(bool checked_thing, QLabel *label)
{
	if (checked_thing == true)
		Green(label);
	else
		Red(label);
}

void Form_Mobile::Red(QLabel *label)
{
	QMetaObject::invokeMethod(label, "setStyleSheet", Qt::QueuedConnection, Q_ARG(QString, "QLabel { background-color: rgb(255, 60, 63)}"));
}

void Form_Mobile::Green(QLabel *label)
{
	QMetaObject::invokeMethod(label, "setStyleSheet", Qt::QueuedConnection, Q_ARG(QString, "QLabel { background-color: rgb(170, 255, 127)}"));
}