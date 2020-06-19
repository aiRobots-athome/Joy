#include "Scara/Form_XYPlatform.h"

void Form_XYPlatform::XYPlatformSliderMove()
{
    ui->XYPlatform_lineEdit_X->setText(QString::number(ui->XYPlatform_slider_X->value()));
    ui->XYPlatform_lineEdit_Y->setText(QString::number(ui->XYPlatform_slider_Y->value()));
    CXYPlatform->GotoPosition(ui->XYPlatform_slider_X->value(), ui->XYPlatform_slider_Y->value());
}

void Form_XYPlatform::XYPlatformPosGo()
{
    const int x = ui->XYPlatform_lineEdit_X->text().toInt();
    const int y = ui->XYPlatform_lineEdit_Y->text().toInt();
    ui->XYPlatform_slider_X->setValue(x);
    ui->XYPlatform_slider_Y->setValue(y);
    CXYPlatform->GotoPosition(x, y);
}

void Form_XYPlatform::XYPlatformReset()
{
    ui->XYPlatform_lineEdit_X->setText(QString::number(0));
    ui->XYPlatform_lineEdit_Y->setText(QString::number(0));
    ui->XYPlatform_slider_X->setValue(0);
    ui->XYPlatform_slider_Y->setValue(0);
    CXYPlatform->GotoPosition(0, 0);
}

void Form_XYPlatform::Display()
{
}