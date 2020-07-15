#include "Scara/Form_XYPlatform.h"

void Form_XYPlatform::XYPlatformPosGo()
{
    const int x = ui->XYPlatform_spinBox_X->value();
    const int y = ui->XYPlatform_spinBox_Y->value();
    CXYPlatform->GotoPosition(x, y);
}

void Form_XYPlatform::XYPlatformReset()
{
    ui->XYPlatform_spinBox_X->setValue(0);
    ui->XYPlatform_spinBox_Y->setValue(0);
}

void Form_XYPlatform::Initial()
{
    ui->XYPlatform_spinBox_X->setMaximum(CXYPlatform->GetMaxX());
    ui->XYPlatform_spinBox_X->setMinimum(-CXYPlatform->GetMaxX());
    ui->XYPlatform_spinBox_Y->setMaximum(CXYPlatform->GetMaxY()); 
    ui->XYPlatform_spinBox_Y->setMinimum(-CXYPlatform->GetMaxY());
}

void Form_XYPlatform::Display()
{
}