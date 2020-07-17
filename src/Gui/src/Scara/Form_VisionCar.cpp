#include "Scara/Form_VisionCar.h"

/**
 * Set vision car angle, screw and camera manually
 * @brief - Get value from user interface and pass to VisionCar
 */
void Form_VisionCar::on_VisionCar_btn_PosGo_clicked()
{
    // /* Vision car angle */
	// const float oz = ui->VisionCar_lineEdit_Oz->text().toFloat();

    // /* Vision car screw up/down */
	// const float pz = ui->VisionCar_lineEdit_Z->text().toFloat();

    // /* Vision car camera position */
    // const float cam = ui->VisionCar_lineEdit_cam->text().toFloat();

	// CVisionCar->GotoPosition(oz, pz, cam);

    CVisionCar->GoCarAngle(0);
	printf("Go clicked\n");
}

/**
 * Move camera in/out
 */
void Form_VisionCar::on_VisionCar_Cam_CTL_clicked()
{

}

/**
 * Move screw up/down
 */
void Form_VisionCar::on_VisionCar_Screw_CTL_clicked()
{

}

/**
 * Move vision car to default position
 */
void Form_VisionCar::on_VisionCar_btn_Reset_clicked()
{
    CVisionCar->Reset();
}