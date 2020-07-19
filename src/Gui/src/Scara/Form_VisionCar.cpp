#include "Scara/Form_VisionCar.h"

/**
 * Set vision car angle, screw and camera manually
 */
void Form_VisionCar::on_VisionCar_btn_PosGo_clicked()
{
    const int oz = ui->VisionCar_spin_CarAngle->value();
    const int h = ui->VisionCar_spin_Screw->value();
    const int oc = ui->VisionCar_spin_CamAngle->value();

	CVisionCar->GotoPosition(oz, h, oc);
}

/**
 * Move camera in/out
 */
void Form_VisionCar::on_VisionCar_Cam_CTL_clicked()
{
    int cam_pos = CVisionCar->GetCamPos();
    if(cam_pos != VisionCar::INSIDE){
        CVisionCar->GoCameraIO(VisionCar::INSIDE);
    }
    else{
        CVisionCar->GoCameraIO(VisionCar::OUTSIDE);
    }
}

/**
 * Move screw up/down
 */
void Form_VisionCar::on_VisionCar_Screw_CTL_clicked()
{
    int screw_pos = CVisionCar->GetScrewPos();
    if(screw_pos != VisionCar::DOWN){
        CVisionCar->GoScrewHeight(VisionCar::DOWN);
    }
    else{
        CVisionCar->GoScrewHeight(VisionCar::UP);
    }
}

/**
 * Move vision car to default position
 */
void Form_VisionCar::on_VisionCar_btn_Reset_clicked()
{
    CVisionCar->Reset();
}

/**
 * Get current position and display
 */
void Form_VisionCar::Display()
{
	// ID
	ui->VisionCar_label_ID_0->setText(QString::number(CVisionCar->GetMotor_ID(0)));
	ui->VisionCar_label_ID_1->setText(QString::number(CVisionCar->GetMotor_ID(1)));
	ui->VisionCar_label_ID_2->setText(QString::number(CVisionCar->GetMotor_ID(2)));
	isOK(CVisionCar->GetMotor_Connected(0), ui->VisionCar_label_ID_0);
	isOK(CVisionCar->GetMotor_Connected(1), ui->VisionCar_label_ID_1);
	isOK(CVisionCar->GetMotor_Connected(2), ui->VisionCar_label_ID_2);

    //Angle
    ui->VisionCar_label_CarAngle->setText(QString::number(CVisionCar->GetMotor_PresentAngle(0), 'f', 1));
    ui->VisionCar_label_Screw->setText(QString::number(CVisionCar->GetMotor_PresentAngle(1), 'f', 1));
    ui->VisionCar_label_CamAngle->setText(QString::number(CVisionCar->GetMotor_PresentAngle(2), 'f', 1));
}

void Form_VisionCar::isOK(bool checked_thing, QLabel *label)
{
	if (checked_thing == true)
		QMetaObject::invokeMethod(label, "setStyleSheet", Qt::QueuedConnection, Q_ARG(QString, "QLabel { background-color: rgb(170, 255, 127)}"));
	else
		QMetaObject::invokeMethod(label, "setStyleSheet", Qt::QueuedConnection, Q_ARG(QString, "QLabel { background-color: rgb(255, 60, 63)}"));
}
