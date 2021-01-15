#include "Robot/Form_ForceSensor.h"

void Form_ForceSensor::LeftForceSensorNormalize()
{
	CLeftForceSensor->DataNormalization();
}

void Form_ForceSensor::RightForceSensorNormalize()
{
	CRightForceSensor->DataNormalization();
}

void Form_ForceSensor::Display()
{
	ui->ForceSensor_label_value_LFx->setText(QString::number((int)CLeftForceSensor->GetNormalizedFx()));
	ui->ForceSensor_label_value_LFy->setText(QString::number((int)CLeftForceSensor->GetNormalizedFy()));
	ui->ForceSensor_label_value_LFz->setText(QString::number((int)CLeftForceSensor->GetNormalizedFz()));
	ui->ForceSensor_label_value_LMx->setText(QString::number((int)CLeftForceSensor->GetNormalizedMx()));
	ui->ForceSensor_label_value_LMy->setText(QString::number((int)CLeftForceSensor->GetNormalizedMy()));
	ui->ForceSensor_label_value_LMz->setText(QString::number((int)CLeftForceSensor->GetNormalizedMz()));

	ui->ForceSensor_label_value_RFx->setText(QString::number((int)CRightForceSensor->GetNormalizedFx()));
	ui->ForceSensor_label_value_RFy->setText(QString::number((int)CRightForceSensor->GetNormalizedFy()));
	ui->ForceSensor_label_value_RFz->setText(QString::number((int)CRightForceSensor->GetNormalizedFz()));
	ui->ForceSensor_label_value_RMx->setText(QString::number((int)CRightForceSensor->GetNormalizedMx()));
	ui->ForceSensor_label_value_RMy->setText(QString::number((int)CRightForceSensor->GetNormalizedMy()));
	ui->ForceSensor_label_value_RMz->setText(QString::number((int)CRightForceSensor->GetNormalizedMz()));
}