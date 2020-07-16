#include "Scara/Form_VisionCar.h"

/**
 * Set vision car angle, screw and camera manually
 * @brief - Get value from user interface and pass to VisionCar
 */
void Form_VisionCar::on_VisionCar_btn_PosGo_clicked()
{
    /* Vision car angle */
	const float oz = ui->VisionCar_lineEdit_Oz->text().toFloat();

    /* Vision car screw up/down */
	const float pz = ui->VisionCar_lineEdit_Z->text().toFloat();

    /* Vision car camera position */
    const float cam = ui->VisionCar_lineEdit_cam->text().toFloat();

	CVisionCar->GotoPosition(oz, pz, cam);
}

/**
 * Reset vision car to default position
 */
void Form_VisionCar::on_Scara_btn_Reset_clicked()
{
	CVisionCar->Reset();
}

/**
 * Set the vision car screw up
 */
void Form_VisionCar::on_Screw_btn_Up_clicked()
{
	CVisionCar->GoScrewHeight(CVisionCar->GetPresentHeight() + 1);
}

/**
 * Set the vision car screw fown
 */
void Form_VisionCar::on_Screw_btn_Down_clicked()
{
	CVisionCar->GoScrewHeight(CVisionCar->GetPresentHeight() - 1);
}

void Form_VisionCar::on_Goal_Height_btn_clicked()
{
	const float height = ui->Screw_lineEdit_Height->text().toFloat();
	CVisionCar->GoScrewHeight(height);
}

void Form_VisionCar::Get_Now_Position()
{
	cv::Mat tempT = CVisionCar->GetKinematics();
	int x = tempT.at<float>(0, 3);
	int y = tempT.at<float>(1, 3);
	int z = CVisionCar->GetPresentHeight();
	float nx = tempT.at<float>(0, 0);
	float ny = tempT.at<float>(1, 0);
	int oz = atan2(ny, nx) * Rad2Angle;
	if (oz < -180)
		oz = oz + 360;
	else if (oz > 180)
		oz = oz - 360;

	ui->VisionCar_label_PresentX->setText(QString::number(x));
	ui->VisionCar_label_PresentY->setText(QString::number(y));
	ui->VisionCar_label_PresentZ->setText(QString::number(z));
	ui->VisionCar_label_PresentOz->setText(QString::number(oz));
}

void Form_VisionCar::Display()
{
	// ID
	ui->VisionCar_label_ID->setText(QString::number(CVisionCar->GetMotor_ID(0)));
	ui->VisionCar_label_ID_2->setText(QString::number(CVisionCar->GetMotor_ID(1)));
	ui->VisionCar_label_ID_3->setText(QString::number(CVisionCar->GetMotor_ID(2)));
	ui->VisionCar_label_ID_4->setText(QString::number(CVisionCar->GetMotor_ID(3)));
	isOK(CVisionCar->GetMotor_Connected(0), ui->VisionCar_label_ID);
	isOK(CVisionCar->GetMotor_Connected(1), ui->VisionCar_label_ID_2);
	isOK(CVisionCar->GetMotor_Connected(2), ui->VisionCar_label_ID_3);
	isOK(CVisionCar->GetMotor_Connected(3), ui->VisionCar_label_ID_4);

	// Angle
	ui->VisionCar_label_PresentAngle->setText(QString::number((int)CVisionCar->GetMotor_PresentAngle(0)));
	ui->VisionCar_label_PresentAngle_2->setText(QString::number((int)CVisionCar->GetMotor_PresentAngle(1)));
	ui->VisionCar_label_PresentAngle_3->setText(QString::number((int)CVisionCar->GetMotor_PresentAngle(2)));
	ui->VisionCar_label_PresentAngle_4->setText(QString::number((int)CVisionCar->GetMotor_PresentAngle(3)));

	//Velocity
	ui->VisionCar_label_PresentVelocity->setText(QString::number((int)CVisionCar->GetMotor_Velocity(0)));
	ui->VisionCar_label_PresentVelocity_2->setText(QString::number((int)CVisionCar->GetMotor_Velocity(1)));
	ui->VisionCar_label_PresentVelocity_3->setText(QString::number((int)CVisionCar->GetMotor_Velocity(2)));
	ui->VisionCar_label_PresentVelocity_4->setText(QString::number((int)CVisionCar->GetMotor_Velocity(3)));

	// Torque
	ui->VisionCar_label_PresentTorque->setText(QString::number((int)CVisionCar->GetMotor_PresentTorque(0)));
	ui->VisionCar_label_PresentTorque_2->setText(QString::number((int)CVisionCar->GetMotor_PresentTorque(1)));
	ui->VisionCar_label_PresentTorque_3->setText(QString::number((int)CVisionCar->GetMotor_PresentTorque(2)));
	ui->VisionCar_label_PresentTorque_4->setText(QString::number((int)CVisionCar->GetMotor_PresentTorque(3)));

	isOK((abs(CVisionCar->GetMotor_PresentTorque(0)) < torque_threshold) ? true : false, ui->VisionCar_label_PresentTorque);
	isOK((abs(CVisionCar->GetMotor_PresentTorque(1)) < torque_threshold) ? true : false, ui->VisionCar_label_PresentTorque_2);
	isOK((abs(CVisionCar->GetMotor_PresentTorque(2)) < torque_threshold) ? true : false, ui->VisionCar_label_PresentTorque_3);
	isOK((abs(CVisionCar->GetMotor_PresentTorque(3)) < torque_threshold) ? true : false, ui->VisionCar_label_PresentTorque_4);

	Get_Now_Position();
}

void Form_VisionCar::isOK(bool checked_thing, QLabel *label)
{
	if (checked_thing == true)
		Green(label);
	else
		Red(label);
}

void Form_VisionCar::Red(QLabel *label)
{
	QMetaObject::invokeMethod(label, "setStyleSheet", Qt::QueuedConnection, Q_ARG(QString, "QLabel { background-color: rgb(255, 60, 63)}"));
}

void Form_VisionCar::Green(QLabel *label)
{
	QMetaObject::invokeMethod(label, "setStyleSheet", Qt::QueuedConnection, Q_ARG(QString, "QLabel { background-color: rgb(170, 255, 127)}"));
}