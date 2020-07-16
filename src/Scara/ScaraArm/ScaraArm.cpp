#include "ScaraArm.h"

ScaraArm *ScaraArm::inst_ = nullptr;

/**
 * Alternatively new scara arm object
 * To ensure we only have one scara arm object alive at a time
 */
ScaraArm *ScaraArm::getScaraArm()
{
	if (inst_ == nullptr)
		inst_ = new ScaraArm();
	return inst_;
}

ScaraArm::ScaraArm()
	/* Big */
	: MotorUnion({0, 1, 2, 3}, {"Pro200", "Pro200", "Pro20", "Pro20"}),
	  J2_sign(-1),
	  Arm1_Length(168),
	  Arm2_Length(390),
	  Arm3_Length(238),
	  Arm4_Length(242),
	  FIRST_HAND_ID(0),
	  Degree2Resolution(1003846 / 360),
	  USING_BIG(true),

	/* Small */
	// : MotorUnion({0, 1, 2, 3}, {"Mx106", "Mx106", "Mx106", "Mx106"}),
	//   J2_sign(1),
	//   Arm1_Length(53), 
	//   Arm2_Length(92), 
	//   Arm3_Length(92),
	//   Arm4_Length(69), 
	//   FIRST_HAND_ID(0),
	//   Degree2Resolution(4096 / 360),
	//   USING_BIG(false),
	

	/* Common variable */
	  PRO200_RESOL(1003846),
	  MX106_RESOL(4096),
	  REV_2_SCREW(226)
{
	if (USING_BIG)
		Height_Resol = PRO200_RESOL;
	else 
		
		Height_Resol = MX106_RESOL;
	Start();
	ReadHeight();
	SetMotor_Operating_Mode(FIRST_HAND_ID, 1);	//Pro 200 change operating mode to velocity mode
	cout << "\t\tClass constructed: ScaraArm" << endl;
}

/**
 * Enable all motors, and set up the speed
 */
void ScaraArm::Start() {
	/* Big */
	SetMotor_Accel(FIRST_HAND_ID, 200);
	SetMotor_Velocity(FIRST_HAND_ID + 1, 50);
	SetMotor_Accel(FIRST_HAND_ID + 1, 25);
	SetMotor_Velocity(FIRST_HAND_ID + 2, 250);
	SetMotor_Accel(FIRST_HAND_ID + 2, 200);
	SetMotor_Velocity(FIRST_HAND_ID + 3, 1000);
	SetMotor_Accel(FIRST_HAND_ID+3, 700);
	SetAllMotorsTorqueEnable(true);

	/* Small */
	// SetMotor_Accel(FIRST_HAND_ID, 50);
	// SetMotor_Velocity(FIRST_HAND_ID + 1, 10);
	// SetMotor_Velocity(FIRST_HAND_ID + 2, 20);
	// SetMotor_Velocity(FIRST_HAND_ID + 3, 20);
	// SetAllMotorsTorqueEnable(true);
}

/**
 * Disable all motors
 */
void ScaraArm::Stop() {
	SetAllMotorsTorqueEnable(false);
}

cv::Mat ScaraArm::GetKinematics()
{
	float fRadian1 = GetMotor_PresentAngle(FIRST_HAND_ID + 1) * Angle2Rad;
	float fRadian2 = J2_sign * GetMotor_PresentAngle(FIRST_HAND_ID + 2) * Angle2Rad;
	float fRadian3 = GetMotor_PresentAngle(FIRST_HAND_ID + 3) * Angle2Rad;
	return Calculate_ArmForwardKinematics(fRadian1, fRadian2, fRadian3);
}

float &ScaraArm::GetPresentHeight()
{
	return now_height;
}

/**
 * Use forward kinematics to calculate end effector position
 * 
 * @param J1 - Joint 1 angle, in radius
 * @param J2 - Joint 2 angle, in radius
 * @param J3 - Joint 3 angle, in radius
 * @retval - End effector orientation/transportation matrix
 */
cv::Mat ScaraArm::Calculate_ArmForwardKinematics(const float &J1, const float &J2, const float &J3) {
	cv::Matx44f TransMatrix_01(1, 0, 0, Arm1_Length,
							   0, 1, 0, 0,
							   0, 0, 1, 0,
							   0, 0, 0, 1);
	cv::Matx44f TransMatrix_12(cos(J1), -sin(J1), 0, Arm2_Length * cos(J1),
							   sin(J1), cos(J1), 0, Arm2_Length * sin(J1),
							   0, 0, 1, 0,
							   0, 0, 0, 1);
	cv::Matx44f TransMatrix_23(cos(J2), -sin(J2), 0, Arm3_Length * cos(J2),
							   sin(J2), cos(J2), 0, Arm3_Length * sin(J2),
							   0, 0, 1, 0,
							   0, 0, 0, 1);
	cv::Matx44f TransMatrix_34(cos(J3), -sin(J3), 0, Arm4_Length * cos(J3),
							   sin(J3), cos(J3), 0, Arm4_Length * sin(J3),
							   0, 0, 1, 0,
							   0, 0, 0, 1);

	cv::Mat Temp(TransMatrix_01 * TransMatrix_12 * TransMatrix_23 * TransMatrix_34);
	return Temp.clone();
}

/**
 * Use inverse kinematics to calculate each angle of motors
 * 
 * @param T - normal, orientation, acceleration, and position of end effector
 * @retval - The angle each motor has to achieve, 
 */
float *ScaraArm::Arm_InverseKinematics(const cv::Mat &T) {
	ScaraArmMotionEnable = false;
	float J1, J2, J3;
	float delta_Pos = 150000 * 2 + 250950 * 4; // default value for calculate delta solution(very large scale)

	float nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz;
	nx = T.at<float>(0, 0);
	ny = T.at<float>(1, 0);
	nz = T.at<float>(2, 0);
	ox = T.at<float>(0, 1);
	oy = T.at<float>(1, 1);
	oz = T.at<float>(2, 1);
	ax = T.at<float>(0, 2);
	ay = T.at<float>(1, 2);
	az = T.at<float>(2, 2);
	px = T.at<float>(0, 3);
	py = T.at<float>(1, 3);
	pz = T.at<float>(2, 3);

	float theta2[2] = {-999, -999}; // J1
	float theta3[2] = {-999, -999}; // J2
	float theta4[2] = {-999, -999}; // J3

	float *Solution = new float[3]; // J1 J2 J3
	for (int i = 0; i < 3; i++)
	{
		Solution[i] = 0;
	}

	// J2 solution have two
	float J2_1 = acos((pow(px - (Arm4_Length * nx) - Arm1_Length, 2) +
					   pow(py - (Arm4_Length * ny), 2) -
					   pow(Arm3_Length, 2) -
					   pow(Arm2_Length, 2)) /
					  (Arm3_Length * Arm2_Length * 2));

	theta3[0] = (J2_1)*Rad2Angle; //positive solution
	theta3[1] = -theta3[0];		  //nagetive solution

	//angel normalization
	for (int i = 0; i < 2; i++)
	{
		if ((theta3[i] > -180) && (theta3[i] < -155)) //critical point
		{
			theta3[i] = -999;
		}
		else if ((theta3[i] > 155) && (theta3[i] < 180)) //critical point
		{
			theta3[i] = -999;
		}
	}

	for (int i = 0; i < 2; i++) /* two solution */
	{
		// J2 sol
		if (theta3[i] != -999)
		{
			J2 = theta3[i] * Angle2Rad;
			float J1_r = sqrt(pow(Arm3_Length * cos(J2) + Arm2_Length, 2) + pow(Arm3_Length * sin(J2), 2));
			float J1_phi = atan2(Arm3_Length * sin(J2), Arm3_Length * cos(J2) + Arm2_Length);
			theta2[i] = (-J1_phi + atan2((py - Arm4_Length * ny) / J1_r,
										 sqrt(1 - pow((py - Arm4_Length * ny) / J1_r, 2)))) *
						Rad2Angle;

			if (theta2[i] > 180)
			{
				theta2[i] = theta2[i] - 360;
			}
			if (theta2[i] < -180)
			{
				theta2[i] = theta2[i] + 360;
			}
			if ((theta2[i] > -180) && (theta2[i] < -155)) //limit
			{
				theta2[i] = -999;
			}
			else if ((theta2[i] > 155) && (theta2[i] < 180)) //limit
			{
				theta2[i] = -999;
			}
		}

		// J1 sol
		if (theta2[i] != -999)
		{
			J1 = theta2[i] * Angle2Rad; // change to radius
			theta4[i] = atan2(
							py * cos(J1 + J2) + Arm1_Length * sin(J1 + J2) - px * sin(J1 + J2) + Arm2_Length * sin(J2),
							px * cos(J1 + J2) - Arm1_Length * cos(J1 + J2) - Arm3_Length + py * sin(J1 + J2) - Arm2_Length * cos(J2)) *
						Rad2Angle;
			if (theta4[i] > 180)
				theta4[i] = theta4[i] - 360;
			if (theta4[i] < -180)
				theta4[i] = theta4[i] + 360;
			if ((theta4[i] > -180) && (theta4[i] < -118)) //limit
				theta4[i] = -999;
			else if ((theta4[i] > 118) && (theta4[i] < 180)) //limit
				theta4[i] = -999;
		}

		// J3 sol
		if (theta4[i] != -999)
		{
			J3 = theta4[i] * Angle2Rad; //change to radius

			bool check = true;

			cv::Mat tmpT = Calculate_ArmForwardKinematics(J1, J2, J3); // use inverse kinematics solution to get forward kinematics

			for (int o = 0; o < 4; o++)
			{
				for (int p = 0; p < 4; p++)
				{
					// check does it same inverse and forward
					if (abs(round(tmpT.at<float>(o, p)) - round(T.at<float>(o, p))) > 1)
					{
						check = false;
					}
				}
			}

			if (check)
			{
				double temp_theta[3] = {J1 * Rad2Angle, J2 * Rad2Angle, J3 * Rad2Angle};
				for (int o = 0; o < 3; o++)
				{
					if (temp_theta[o] > 180)
					{
						temp_theta[o] = temp_theta[o] - 360;
					}
					else if (temp_theta[o] < -180)
					{
						temp_theta[o] = temp_theta[o] + 360;
					}
				}

				temp_theta[0] = temp_theta[0];
				temp_theta[1] = J2_sign * temp_theta[1];
				temp_theta[2] = temp_theta[2];

				// get delta angle
				int tmp_delta_Ang =
					abs(temp_theta[0] - GetMotor_PresentAngle(FIRST_HAND_ID + 1)) +
					abs(temp_theta[1] - GetMotor_PresentAngle(FIRST_HAND_ID + 2)) +
					abs(temp_theta[2] - GetMotor_PresentAngle(FIRST_HAND_ID + 3));

				// check delta limit
				if (tmp_delta_Ang < delta_Pos)
				{
					delta_Pos = tmp_delta_Ang;

					for (int o = 0; o < 3; o++)
					{
						Solution[o] = temp_theta[o];
					}
					ScaraArmMotionEnable = true; // let torque enable be true
				}
			}
		}
	}

	return Solution;
}

/**
 * Goto position, height seperate version
 * seperate height for more accurate movement
 * 
 * @param ox - orientation along x axis, in degree
 * @param oy - orientation along y axis, in degree
 * @param oz - orientation along z axis, in degree
 * @param x - position on x asix, in mm
 * @param y - position on y asix, in mm 
 * @param height - height of scara arm, in mm
 */
void ScaraArm::GotoPosition(const int &ox, const int &oy, const int &oz, const int &x, const int &y, const float &height) {
	GoScrewHeight(height);
	GotoPosition(ox, oy, oz, x, y, 0);
}

/**
 * Goto position, including height
 * But for scara, it,s impossible to change z, therefore, set z to zero
 * 
 * @param ox - orientation along x axis, in degree
 * @param oy - orientation along y axis, in degree
 * @param oz - orientation along z axis, in degree
 * @param x - position on x asix, in mm
 * @param y - position on y asix, in mm 
 * @param height - height of scara arm, in mm
 */
void ScaraArm::GotoPosition(const int &ox, const int &oy, const int &oz, const int &x, const int &y, const int &z) {
	cv::Mat T = TransRotate(0, 0, oz);
	float data[16] = {T.at<float>(0, 0), T.at<float>(0, 1), T.at<float>(0, 2), float(x),
					  T.at<float>(1, 0), T.at<float>(1, 1), T.at<float>(1, 2), float(y),
					  T.at<float>(2, 0), T.at<float>(2, 1), T.at<float>(2, 2), float(0),
					  0, 0, 0, 1};
	cv::Mat tmp = cv::Mat(4, 4, CV_32FC1, data);
	GotoPosition(tmp);
}

/**
 * Goto position by calcualting inverse kinematics
 * 
 * @param T - End effector matrix, for inverse kinematics
 */
void ScaraArm::GotoPosition(const cv::Mat &T)
{
	float *tmp = Arm_InverseKinematics(T);
	if (ScaraArmMotionEnable)
	{
		SetMotor_Angle(FIRST_HAND_ID, GetMotor_PresentAngle(FIRST_HAND_ID));
		SetMotor_Angle(FIRST_HAND_ID + 1, tmp[0]);
		SetMotor_Angle(FIRST_HAND_ID + 2, tmp[1]);
		SetMotor_Angle(FIRST_HAND_ID + 3, tmp[2]);
		WaitAllMotorsArrival();
		delete tmp;
		cout << "[ScaraArm] Arm arrival !" << endl;
	}
	else
	{
		cout << "[ScaraArm] Arm fail to arrive !" << endl;
	}
}

/**
 * Find (n, o, a)'s mapping on (x, y, z)
 * 
 * @param ox - End effector orientation along x axis
 * @param oy - End effector orientation along y axis
 * @param oz - End effector orientation along z axis
 * @retval - 3*3 matrix, which is [ nx ox ax
 * 									ny oy ay
 * 									nz oz az ]
 */
cv::Mat ScaraArm::TransRotate(const float &ox, const float &oy, const float &oz) {
	float x_angle = ox * Angle2Rad;
	float y_angle = oy * Angle2Rad;
	float z_angle = oz * Angle2Rad;

	float temp_nx = cos(z_angle) * cos(y_angle);
	float temp_ny = sin(z_angle) * cos(y_angle);
	float temp_nz = -sin(y_angle);
	float temp_ox = cos(z_angle) * sin(y_angle) * sin(x_angle) - sin(z_angle) * cos(x_angle);
	float temp_oy = sin(z_angle) * sin(y_angle) * sin(x_angle) + cos(z_angle) * cos(x_angle);
	float temp_oz = cos(y_angle) * sin(x_angle);
	float temp_ax = cos(z_angle) * sin(y_angle) * cos(x_angle) + sin(z_angle) * sin(x_angle);
	float temp_ay = sin(z_angle) * sin(y_angle) * cos(x_angle) - cos(z_angle) * sin(x_angle);
	float temp_az = cos(y_angle) * cos(x_angle);

	float data[9] = {temp_nx, temp_ox, temp_ax,
					 temp_ny, temp_oy, temp_ay,
					 temp_nz, temp_oz, temp_az};
	cv::Mat tmpMat = cv::Mat(3, 3, CV_32FC1, data);
	return tmpMat.clone();
}

/**
 * Read height from file
 */
void ScaraArm::ReadHeight()
{
	// Read Height
	char height[200];
	fstream heightfile;
	string path = string(getenv("PWD")) + "/src/Scara/ScaraArm/Height.txt";
	heightfile.open(path, ios::in);
	if (heightfile.fail())
		cout << "[ScaraArm] Cannot open Height.txt" << endl;
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
void ScaraArm::WriteHeight(const float &height) const
{
	fstream heightfile;
	string path = string(getenv("PWD")) + "/src/Scara/ScaraArm/Height.txt";
	heightfile.open(path, ios::out);
	if (heightfile.fail())
		cout << "[ScaraArm] Cannot open Height.txt" << endl;
	else
	{
		heightfile << height;
		heightfile.close();
	}
}

/**
 * Move arm to desire height
 * 
 * @param goal_height - Height we desired to achieve, in mm 
 * @retval - true if height is legal, false if illegal
 */
// Need to check
bool ScaraArm::GoScrewHeight(const float &goal_height) {
	ReadHeight();
	if (goal_height > 400)	{
		cout << "[ScaraArm] Too high" << endl;
		return false;
	}
	else if (goal_height < 20)	{
		cout << "[ScaraArm] Too low" << endl;
		return false;
	}
	else if (goal_height == now_height)
	{
		cout << "[ScaraArm] Screw arrival !" << endl;
		return true;
	}
	else
	{
		float delta_height = goal_height - now_height;
		int dir = 0;		// Move up or down for the robot arm
		if (delta_height > 0)
			dir = 1;
		else
			dir = -1;
		int now_position = GetMotor_PresentAngle(FIRST_HAND_ID) * Degree2Resolution;
		// 224(Speed ​​increaser ratio 1:11.05, Pro200 1rev = Screw 224mm)  1003846(Pro200 resolution)
		int delta_postion = round(delta_height / REV_2_SCREW * Height_Resol);
		int need_position = now_position + delta_postion;
		int delta_height_f = 0;
		if (abs(delta_height) > 10)
		{
			delta_height_f = delta_height - dir * 2;

			float motorspeed = 400 * 0.01 / 60 * REV_2_SCREW;					//Velocity(set 400) * Scale(rev/min) / 60(to 1sec) * 226(mm) (Pro200 1rev = Screw 226)
			float waittime = (abs(delta_height_f) / motorspeed) * 1000; //unit:milliseconds

			SetMotor_Velocity(FIRST_HAND_ID, dir * 400);
			WaitAllMotorsArrival(waittime);
			SetMotor_Velocity(FIRST_HAND_ID, 0);
			this_thread::sleep_for(chrono::milliseconds(500));
		}
		now_position = GetMotor_PresentAngle(FIRST_HAND_ID) * Degree2Resolution;
		delta_height_f = need_position - now_position;

		if (delta_height_f > 0)
			dir = 1;
		else
			dir = -1;

		// Use to integrate the moved angle of motor
		int pos_integrator = 0;

		// Last position of motor, in resolution
		int last_pos = GetMotor_PresentAngle(FIRST_HAND_ID) * Degree2Resolution;

		// Set motor speed to 1
		SetMotor_Velocity(FIRST_HAND_ID, dir*10);

		printf("det_h_f: %d", delta_height_f);
		while(abs(delta_height_f - pos_integrator) >= 500) {
			this_thread::sleep_for(chrono::milliseconds(50));
			int present_pos = GetMotor_PresentAngle(FIRST_HAND_ID) * Degree2Resolution;

			// Integrate the position change
			// position difference
			int pos_diff = present_pos - last_pos;
			// Assume every angle difference is acute
			if (abs(pos_diff) > 180 * Degree2Resolution)
				pos_diff = dir * Height_Resol - pos_diff;
			pos_integrator += pos_diff;
			last_pos = present_pos;
			printf("det_h_f: %d, inte: %d\n", delta_height_f, pos_integrator);
		} 

		// Set motor speed to 0
		SetMotor_Velocity(FIRST_HAND_ID, 0);
		WriteHeight(goal_height);
		now_height = goal_height;
		cout << "[ScaraArm] Screw arrival !" << endl;
		return true;
	}
}

/**
 * Move scara arm to initial point
 */
void ScaraArm::Reset(){
	GotoPosition(0, 0, 62, 437, 459, 250.0f);
}