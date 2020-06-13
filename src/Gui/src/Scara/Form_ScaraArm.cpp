#include "Form_ScaraArm.h"

void Form_ScaraArm::Display()
{
	// ID
	ui->Scara_label_ID->setText(QString::number(CScaraArm->GetMotor_ID(0)));
	ui->Scara_label_ID_2->setText(QString::number(CScaraArm->GetMotor_ID(1)));
	ui->Scara_label_ID_3->setText(QString::number(CScaraArm->GetMotor_ID(2)));
	ui->Scara_label_ID_4->setText(QString::number(CScaraArm->GetMotor_ID(3)));
	isOK(CScaraArm->GetMotor_Connected(0), ui->Scara_label_ID);
	isOK(CScaraArm->GetMotor_Connected(1), ui->Scara_label_ID_2);
	isOK(CScaraArm->GetMotor_Connected(2), ui->Scara_label_ID_3);
	isOK(CScaraArm->GetMotor_Connected(3), ui->Scara_label_ID_4);

	// Angle
	ui->Scara_label_PresentAngle->setText(QString::number((int)CScaraArm->GetMotor_PresentAngle(0)));
	ui->Scara_label_PresentAngle_2->setText(QString::number((int)CScaraArm->GetMotor_PresentAngle(1)));
	ui->Scara_label_PresentAngle_3->setText(QString::number((int)CScaraArm->GetMotor_PresentAngle(2)));
	ui->Scara_label_PresentAngle_4->setText(QString::number((int)CScaraArm->GetMotor_PresentAngle(3)));

	//Velocity
	ui->Scara_label_PresentVelocity->setText(QString::number((int)CScaraArm->GetMotor_Velocity(0)));
	ui->Scara_label_PresentVelocity_2->setText(QString::number((int)CScaraArm->GetMotor_Velocity(1)));
	ui->Scara_label_PresentVelocity_3->setText(QString::number((int)CScaraArm->GetMotor_Velocity(2)));
	ui->Scara_label_PresentVelocity_4->setText(QString::number((int)CScaraArm->GetMotor_Velocity(3)));

	// Torque
	ui->Scara_label_PresentTorque->setText(QString::number((int)CScaraArm->GetMotor_PresentTorque(0)));
	ui->Scara_label_PresentTorque_2->setText(QString::number((int)CScaraArm->GetMotor_PresentTorque(1)));
	ui->Scara_label_PresentTorque_3->setText(QString::number((int)CScaraArm->GetMotor_PresentTorque(2)));
	ui->Scara_label_PresentTorque_4->setText(QString::number((int)CScaraArm->GetMotor_PresentTorque(3)));
	
	isOK((abs(CScaraArm->GetMotor_PresentTorque(0)) < torque_threshold) ? true : false, ui->Scara_label_PresentTorque);
	isOK((abs(CScaraArm->GetMotor_PresentTorque(1)) < torque_threshold) ? true : false, ui->Scara_label_PresentTorque_2);
	isOK((abs(CScaraArm->GetMotor_PresentTorque(2)) < torque_threshold) ? true : false, ui->Scara_label_PresentTorque_3);
	isOK((abs(CScaraArm->GetMotor_PresentTorque(3)) < torque_threshold) ? true : false, ui->Scara_label_PresentTorque_4);

	Get_Now_Position();
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

void Form_ScaraArm::on_ScaraArm_btn_PosGo_clicked()
{
    const float ox = ui->Scara_lineEdit_RX->text().toFloat();
	const float oy = ui->Scara_lineEdit_RY->text().toFloat();
	const float oz = ui->Scara_lineEdit_RZ->text().toFloat();

	const float px = ui->Scara_lineEdit_X->text().toFloat();
	const float py = ui->Scara_lineEdit_Y->text().toFloat();
	const float pz = ui->Scara_lineEdit_Z->text().toFloat();

	CScaraArm->ScaraGO(ox, oy, oz, px, py, pz);
}

void Form_ScaraArm::on_Scara_btn_Reset_clicked()
{
    CScaraArm->Reset();
}

void Form_ScaraArm::on_Screw_btn_Up_clicked()
{
    float tmp = CScaraArm->ReadSaveHeight();
    CScaraArm->GOScrewHeight(tmp+1);
}

void Form_ScaraArm::on_Screw_btn_Down_clicked()
{
    float tmp = CScaraArm->ReadSaveHeight();
    CScaraArm->GOScrewHeight(tmp-1);
}

void Form_ScaraArm::on_Goal_Height_btn_clicked()
{
    const float height = ui->Goal_Height_lineEdit->text().toFloat();
    CScaraArm->GOScrewHeight(height);
}

void Form_ScaraArm::Get_Now_Position()
{
	cv::Mat* tempT;
    tempT = CScaraArm->GetKinematics();
    float x = tempT->at<float>( 0, 3 );
    float y = tempT->at<float>( 1, 3 );
	float nx = tempT->at<float>( 0, 0 );
	float ny = tempT->at<float>( 1, 0 );
	float oz = atan2(ny, nx);
	if (oz < -M_PI)
		oz = oz +2*M_PI;
	else if (oz > M_PI)
		oz = oz -2*M_PI;
	oz = oz * Rad2Angle;
    float z = CScaraArm->ReadSaveHeight();
    ui->Now_Posotion_X_Show->setText(QString::number(x));
    ui->Now_Posotion_Y_Show->setText(QString::number(y));
    ui->Now_Posotion_Z_Show->setText(QString::number(z));
	ui->Now_Posotion_OZ_Show->setText(QString::number(oz));
}