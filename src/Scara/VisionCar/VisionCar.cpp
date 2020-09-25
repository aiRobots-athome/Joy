#include "VisionCar.h"
#include <unistd.h>

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

/**
 * Vision Car constructor
 */
VisionCar::VisionCar()
	: MotorUnion({4, 5, 6}, {"Pro200", "Pro20", "Pro20"}),	// Pro20 resolution: 303750*2/rev
	  FIRST_MOTOR_ID(0),
	  REV_2_SCREW(181)
{
	SetMotor_Operating_Mode(FIRST_MOTOR_ID  , 3);	    // Car angle control motor set to position mode
	SetMotor_Operating_Mode(FIRST_MOTOR_ID+1, 4);	    // Screw control motor set to extended position mode
	SetMotor_Operating_Mode(FIRST_MOTOR_ID+2, 3);	    // Cam angle control motor set to position mode
	Start();
	cout << "\t\tClass constructed: VisionCar" << endl;
}

/**
 * Enable all motors, and set up the speed
 */
void VisionCar::Start()
{
	// Car angle motor
	SetMotor_Velocity(FIRST_MOTOR_ID, 100);
	SetMotor_Accel(FIRST_MOTOR_ID, 50);

	// Screw motor
	SetMotor_Velocity(FIRST_MOTOR_ID + 1, 2000);
	SetMotor_Accel(FIRST_MOTOR_ID + 1, 10765);

	// Cam angle motor
	SetMotor_Velocity(FIRST_MOTOR_ID + 2, 200);
	SetMotor_Accel(FIRST_MOTOR_ID + 2, 100);
	sleep(1);
	SetAllMotorsTorqueEnable(true);
}

/**
 * Disable all motors
 */
void VisionCar::Stop()
{
	SetAllMotorsTorqueEnable(false);
}

/**
 * Move car, camera and camera height to desired position.
 * Before moving the car, the camera will be moved inside and the screw will be moved down for sake of safety
 * @param oz - int, vision car angle
 * @param h  - int, screw high (1) or low (0), pos_state enum is better
 * @param oc - int, camera outside (1) or inside (0), pos_state enum is better
 */
void VisionCar::GotoPosition(const int &oz, const int &h,  const int &oc)
{

	// Move camera inside if camera is now outside or undefined (undefined mean initial)
	if(current_cam_io != INSIDE){
		GoCameraIO(INSIDE);
		WaitMotorArrival(2);
	}

	// Screw down if screw is now upside or undefined (undefined mean initial)
	if (current_screw_io != DOWN){
		GoScrewHeight(DOWN);
	}

	// Set car angle
	GoCarAngle(oz);
	WaitMotorArrival(0);

	// Screw up or down
	if (current_screw_io != h){
		GoScrewHeight(h);
	}

	// Camera out or in
	if(current_cam_io != oc){
		GoCameraIO(oc);
		WaitMotorArrival(2);
	}
}

// Car

/**
 * Move vision car to goal angle
 * ##  No checking if already in position or not
 * @param goal_angle - int, target angle to go
 */
void VisionCar::GoCarAngle(const int &goal_angle)
{
	SetMotor_Angle(FIRST_MOTOR_ID, goal_angle);
}

// Screw

/**
 * Read height from file
 */
void VisionCar::ReadHeight()
{
	// Read Height
	char height[200];
	fstream heightfile;
	string path = string(getenv("PWD")) + "/src/Scara/VisionCar/Height.txt";
	heightfile.open(path, ios::in);
	if (heightfile.fail())
		cout << "[VisionCar] Cannot open Height.txt" << endl;
	else
	{
		heightfile.read(height, sizeof(height));
		heightfile.close();
		now_height = stof(height);
	}
}

/**
 * Write height into height.txt
 */
void VisionCar::WriteHeight(const float &height) const
{
	fstream heightfile;
	string path = string(getenv("PWD")) + "/src/Scara/VisionCar/Height.txt";
	heightfile.open(path, ios::out);
	if (heightfile.fail())
		cout << "[VisionCar] Cannot open Height.txt" << endl;
	else
	{
		heightfile << height;
		heightfile.close();
	}
}

/**
 * Move camera to desire height
 * 
 * @param goal_height - Height we desired to achieve, in mm 
 * @retval - int, screw state (UP, DOWN, UNDEFINED) after moving
 */
int VisionCar::GoScrewHeight(const int &dir) {

	float goal_height;
	int target_state = UNDEFINED;
	if (dir == DOWN){
		goal_height = ScrewDownHeight;
		target_state = DOWN;
	}
	else{
		goal_height = ScrewUpHeight;
		target_state = UP;
	}
	
	ReadHeight();
	if (goal_height == now_height) {
		current_screw_io = target_state;

		cout << "[VisionCar] Screw arrival !" << endl;
		return current_screw_io;
	}
	else {
		float delta_height = goal_height - now_height;
		float delta_angle = -1 * delta_height / REV_2_SCREW * 360; // Delta angle <0: move up.

		// Motor angle is set to present angle + angle needed
		SetMotor_Angle(FIRST_MOTOR_ID+1, delta_angle + GetMotor_PresentAngle(FIRST_MOTOR_ID+1));

		// cout << "delta_height: " << delta_height << ", delta_angle: " << delta_angle << endl;
		// cout << "desire angle: " << GetMotor_Angle(FIRST_MOTOR_ID) << endl;
		this_thread::sleep_for(chrono::milliseconds(500));

		WaitMotorArrival(FIRST_MOTOR_ID+1);
		this_thread::sleep_for(chrono::milliseconds(50));

		// Update record data
		WriteHeight(goal_height);
		now_height = goal_height;
		current_screw_io = target_state;

		cout << "[VisionCar] Screw arrival !" << endl;
		return current_screw_io;
	}
}

// Camera

/**
 * Move camera out or in
 * ##  No checking if already in position or not
 * @param io - int, inside (0), outside (1)
 * @retval current_cam_io - int, current camera position
 */
int VisionCar::GoCameraIO(const int &io)
{
	if (io == INSIDE){
		SetMotor_Angle(FIRST_MOTOR_ID+2, CamInDegree);
		current_cam_io = INSIDE;
	}
	else{
		SetMotor_Angle(FIRST_MOTOR_ID+2, CamOutDegree);
		current_cam_io = OUTSIDE;
	}
	return current_cam_io;
}

/**
 * Get the Camera position data
 * @retval - int, inside (true), outside (false)
 */
int VisionCar::GetCamPos()
{
	return current_cam_io;
}

/**
 * Get the Screw position data
 * @retval - int, up (true), down (false)
 */
int VisionCar::GetScrewPos()
{
	return current_screw_io;
}

/**
 * Move scara arm to initial point
 * Camera in, Screw down and car to 0.
 */
void VisionCar::Reset()
{
	// camera in
	GoCameraIO(VisionCar::INSIDE);
	// Screw down
	GoScrewHeight(VisionCar::DOWN);
	// Car to 0 degree
	GoCarAngle(0);
	WaitMotorArrival(0);
}
