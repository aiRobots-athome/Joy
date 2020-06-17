#include "Robot/Form_Head.h"
void Form_Head::HeadReset()
{
	ui->HeadHorizontal_label_value->setText(QString::number(0));
	ui->HeadVertical_label_value->setText(QString::number(0));
	CHeadandLifting->ResetAllMotorAngle();
}

void Form_Head::HeadVerticalMove()
{
	ui->HeadVertical_label_value->setText(QString::number(ui->HeadVertical_Slider->value()));
    CHeadandLifting->HeadMotorCommand(0, ui->HeadVertical_Slider->value());
}

void Form_Head::HeadHorizontalMove()
{
	ui->HeadHorizontal_label_value->setText(QString::number(ui->HeadHorizontal_Slider->value()));
    CHeadandLifting->HeadMotorCommand(1, ui->HeadHorizontal_Slider->value());
}

void Form_Head::Display()
{
    /* HeadandLifting */
	// ID
	ui->Head_label_ID->setText(QString::number(CHeadandLifting->GetMotor_ID(0)));
	ui->Head_label_ID_2->setText(QString::number(CHeadandLifting->GetMotor_ID(1)));
	// ui->Head_label_ID_3->setText(QString::number(CHeadandLifting->GetMotor_ID(2)));
	// ui->Head_label_ID_4->setText(QString::number(CHeadandLifting->GetMotor_ID(3)));
	isOK(CHeadandLifting->GetMotor_Connected(0), ui->Head_label_ID);
	isOK(CHeadandLifting->GetMotor_Connected(1), ui->Head_label_ID_2);
	// isOK(CHeadandLifting->GetMotor_Connected(2), ui->Head_label_ID_3);
	// isOK(CHeadandLifting->GetMotor_Connected(3), ui->Head_label_ID_4);
	// Angle
	ui->Head_label_PresentAngle->setText(QString::number((int)CHeadandLifting->GetMotor_PresentAngle(0)));
	ui->Head_label_PresentAngle_2->setText(QString::number((int)CHeadandLifting->GetMotor_PresentAngle(1)));
	// ui->Head_label_PresentAngle_3->setText(QString::number((int)CHeadandLifting->GetMotor_PresentAngle(2)));
	// ui->Head_label_PresentAngle_4->setText(QString::number((int)CHeadandLifting->GetMotor_PresentAngle(3)));
}

void Form_Head::isOK(bool checked_thing, QLabel *label)
{
	if (checked_thing == true)
		Green(label);
	else
		Red(label);
}

void Form_Head::Red(QLabel *label)
{
	QMetaObject::invokeMethod(label, "setStyleSheet", Qt::QueuedConnection, Q_ARG(QString, "QLabel { background-color: rgb(255, 60, 63)}"));
}

void Form_Head::Green(QLabel *label)
{
	QMetaObject::invokeMethod(label, "setStyleSheet", Qt::QueuedConnection, Q_ARG(QString, "QLabel { background-color: rgb(170, 255, 127)}"));
}

