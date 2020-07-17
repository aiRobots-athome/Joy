#include "VisionCar.h"

VisionCar *VisionCar::inst_ = nullptr;

/**
 * Alternatively new scara arm object
 * To ensure we only have one scara arm object alive at a time
 * @retval inst_ - pointer of VisionCar object
 */
VisionCar *VisionCar::getVisionCar()
{
	if (inst_ == nullptr)
		inst_ = new VisionCar();
	return inst_;
}

VisionCar::VisionCar()
	: MotorUnion({0, 1, 2}, {"Pro200", "Pro20", "Pro20"}),
	  FIRST_MOTOR_ID(0),
	  Degree2Resolution(1003846 / 360)
{
	Start();
	SetMotor_Operating_Mode(FIRST_MOTOR_ID  , 3);	    // Car angle control motor set to position mode
	SetMotor_Operating_Mode(FIRST_MOTOR_ID+1, 1);	    // Screw control motor set to velocity mode
	SetMotor_Operating_Mode(FIRST_MOTOR_ID+2, 3);	    // Cam angle control motor set to position mode
	cout << "\t\tClass constructed: VisionCar" << endl;
}

/**
 * Enable all motors, and set up the speed
 */
void VisionCar::Start() {
	// Car angle motor
	SetMotor_Velocity(FIRST_MOTOR_ID, 100);
	SetMotor_Accel(FIRST_MOTOR_ID, 50);

	// Screw motor
	SetMotor_Accel(FIRST_MOTOR_ID + 1, 50);

	// Cam angle motor
	SetMotor_Velocity(FIRST_MOTOR_ID + 2, 100);
	SetMotor_Accel(FIRST_MOTOR_ID + 2, 50);

	SetAllMotorsTorqueEnable(true);
}

/**
 * Disable all motors
 */
void VisionCar::Stop() {
	SetAllMotorsTorqueEnable(false);
}

/**
 * Move car, camera and camera height to desired position.
 * Before moving the car, the camera will be moved inside and the screw will be moved down for sake of safety
 * @param oz - int, vision car angle
 * @param h  - int, screw high (>0) or low (else)
 * @param oc - int, camera outside (>0) or inside (else)
 */
void VisionCar::GotoPosition(const float &oz, const int &h,  const int &oc) {

	// Move camera inside
	if(current_cam_io != INSIDE){
		GoCameraIO(INSIDE);
	}

	// Screw down
	if (current_screw_io != DOWN){
		GoScrewHeight(DOWN);
	}

	// Set car angle
	GoCarAngle(oz);

	// Screw up or down
	bool screw_io = (h > 0)? UP: DOWN;
	if (current_screw_io != screw_io){
		GoScrewHeight(screw_io);
	}

	// Camera out or in
	bool cam_io = (oc > 0)? OUTSIDE: INSIDE;
	if(current_cam_io != cam_io){
		GoCameraIO(cam_io);
	}
}

/**
 * Move vision car to goal angle
 * ##  No checking if already in position or not
 * @param goal_angle - float, target angle to go
 */
void VisionCar::GoCarAngle(const float &goal_angle){
	SetMotor_Angle(FIRST_MOTOR_ID, goal_angle);
}

/**
 * Move camera to desired height
 * @param dir - move screw up or down
 * @retval - bool, current screw position
 */
bool VisionCar::GoScrewHeight(const bool &dir) {
	/*Moving*/

	printf("Not yet\n");
	current_screw_io = dir;
	return current_screw_io;
}

/**
 * Move camera out or in
 * ##  No checking if already in position or not
 * @param IO - bool, inside (true), outside (false)
 * @retval current_cam_io - bool, current camera position
 */
bool VisionCar::GoCameraIO(const bool &IO){
	if (IO==INSIDE){
		SetMotor_Angle(FIRST_MOTOR_ID+2, CamInDegree);
	}
	else{
		SetMotor_Angle(FIRST_MOTOR_ID+2, CamOutDegree);
	}
	current_cam_io = IO;
	return current_cam_io;
}

/**
 * Move scara arm to initial point
 * Camera in, Screw down and car to 0.
 */
void VisionCar::Reset(){
	// camera in
	GoCameraIO(true);
	// Screw down
	GoScrewHeight(DOWN);
	// Car to 0 degree
	GoCarAngle(0);
}