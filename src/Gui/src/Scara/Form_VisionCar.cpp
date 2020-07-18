#include "Scara/Form_VisionCar.h"

/**
 * Set vision car angle, screw and camera manually
 * @brief - Get value from user interface and pass to VisionCar
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
    ui->VisionCar_label_CarAngle->setText(QString::number((int)CVisionCar->GetMotor_PresentAngle(0)));
    ui->VisionCar_label_Screw->setText(QString::number((int)CVisionCar->GetMotor_PresentAngle(1)));
    ui->VisionCar_label_CamAngle->setText(QString::number((int)CVisionCar->GetMotor_PresentAngle(2)));
}