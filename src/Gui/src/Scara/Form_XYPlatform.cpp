#include "Scara/Form_XYPlatform.h"

void Form_XYPlatform::XYPlatformPosGo()
{
    const int x = ui->XYPlatform_lineEdit_X->text().toInt();
    const int y = ui->XYPlatform_lineEdit_Y->text().toInt();
    CXYPlatform->GotoPosition(x, y);
}

void Form_XYPlatform::XYPlatformReset()
{
    ui->XYPlatform_lineEdit_X->setText(QString::number(0));
    ui->XYPlatform_lineEdit_Y->setText(QString::number(0));
    CXYPlatform->GotoPosition(0, 0);
}

void Form_XYPlatform::Display()
{
}