#include "Scara/Form_XYPlatform.h"

void Form_XYPlatform::XYPlatformSliderMove()
{
    CXYPlatform->GotoPosition(ui->XYPlatform_slider_X->value(), ui->XYPlatform_slider_Y->value());
}

void Form_XYPlatform::XYPlatformEditInput()
{
    // const int x = ui->XYPlatform_lineEdit_X->text().toInt();
    // const int y = ui->XYPlatform_lineEdit_Y->text().toInt();
    // CXYPlatform->GotoPosition(x, y);
    // ui->XYPlatform_slider_X->setValue(x);
    // ui->XYPlatform_slider_Y->setValue(y);
}

void Form_XYPlatform::XYPlatformReset()
{
    CXYPlatform->GotoPosition(0, 0);
    ui->XYPlatform_slider_X->setValue(0);
    ui->XYPlatform_slider_Y->setValue(0);
}

void Form_XYPlatform::Display()
{
    ui->XYPlatform_label_X->setText(QString::number(CXYPlatform->GetPresentX()));
    ui->XYPlatform_label_Y->setText(QString::number(CXYPlatform->GetPresentY()));
}