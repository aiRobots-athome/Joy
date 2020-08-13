#include "ScaraArm.h"

#define USE_BIG

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
#ifdef USE_BIG
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
#else
	/* Small */
	: MotorUnion({0, 1, 2, 3}, {"Mx106", "Mx106", "Mx106", "Mx106"}),
	  J2_sign(1),
	  Arm1_Length(53), 
	  Arm2_Length(92), 
	  Arm3_Length(92),
	  Arm4_Length(69), 
	  FIRST_HAND_ID(0),
	  Degree2Resolution(4096 / 360),
	  USING_BIG(false),
	
#endif

	/* Common variable */
	  PRO200_RESOL(1003846),
	  MX106_RESOL(4096),
	  REV_2_SCREW(226),
	  Acc_Factor(2)
{
#ifdef USE_BIG
		Height_Resol = PRO200_RESOL;
		printf("pro200!!");
#else 
		Height_Resol = MX106_RESOL;
		printf("mx106!!");
#endif
	Start();
	ReadHeight();
	SetMotor_Operating_Mode(FIRST_HAND_ID, 4);	//Pro 200 change operating mode to extended mode
	cout << "\t\tClass constructed: ScaraArm" << endl;
}

/**
 * Enable all motors, and set up the speed
 */
void ScaraArm::Start() {
#ifdef USE_BIG
	/* Big */
	SetMotor_Velocity(FIRST_HAND_ID, 500);
	SetMotor_Accel(FIRST_HAND_ID, 250);
	SetMotor_Velocity(FIRST_HAND_ID + 1, 500);
	SetMotor_Accel(FIRST_HAND_ID + 1, 250);
	SetMotor_Velocity(FIRST_HAND_ID + 2, 500);
	SetMotor_Accel(FIRST_HAND_ID + 2, 250);
	SetMotor_Velocity(FIRST_HAND_ID + 3, 1000);
	SetMotor_Accel(FIRST_HAND_ID+3, 250);
	SetAllMotorsTorqueEnable(true);
	
#else
	/* Small */
	SetMotor_Velocity(FIRST_HAND_ID, 50);
	SetMotor_Accel(FIRST_HAND_ID, 50);
	SetMotor_Velocity(FIRST_HAND_ID + 1, 10);
	SetMotor_Velocity(FIRST_HAND_ID + 2, 20);
	SetMotor_Velocity(FIRST_HAND_ID + 3, 20);
	SetAllMotorsTorqueEnable(true);
#endif
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

float &ScaraArm::GetPresentHeight() {
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
void ScaraArm::GotoPosition(const cv::Mat &T) {
	float *tmp = Arm_InverseKinematics(T);
	if (ScaraArmMotionEnable)
	{
		this_thread::sleep_for(chrono::milliseconds(500));
		SetMotor_Angle(FIRST_HAND_ID, GetMotor_PresentAngle(FIRST_HAND_ID));
		SetMotor_Angle(FIRST_HAND_ID + 1, tmp[0]);
		SetMotor_Angle(FIRST_HAND_ID + 2, tmp[1]);
		SetMotor_Angle(FIRST_HAND_ID + 3, tmp[2]);
		WaitAllMotorsArrival();
		cout << "[ScaraArm] Arm arrival !" << endl;
	}
	else
	{
		cout << "[ScaraArm] Arm fail to arrive !" << endl;
	}
	delete tmp;
}

/**
 * Set position for motors by calcualting inverse kinematics
 * 
 * @param T - End effector matrix, for inverse kinematics
 */
void ScaraArm::SetPosition(const cv::Mat &T) {
	float *tmp = Arm_InverseKinematics(T);
	if (ScaraArmMotionEnable) {
		this_thread::sleep_for(chrono::milliseconds(500));
		SetMotor_Angle(FIRST_HAND_ID, GetMotor_PresentAngle(FIRST_HAND_ID));
		SetMotor_Angle(FIRST_HAND_ID + 1, tmp[0]);
		SetMotor_Angle(FIRST_HAND_ID + 2, tmp[1]);
		SetMotor_Angle(FIRST_HAND_ID + 3, tmp[2]);
	}
	delete tmp;
}

/**
 * Set motor position
 * But for scara, it,s impossible to change z, therefore, set z to zero
 * 
 * @param ox - orientation along x axis, in degree
 * @param oy - orientation along y axis, in degree
 * @param oz - orientation along z axis, in degree
 * @param x - position on x asix, in mm
 * @param y - position on y asix, in mm 
 * @param height - height of scara arm, in mm
 */
void ScaraArm::SetPosition(const int &ox, const int &oy, const int &oz, const int &x, const int &y, const int &z) {
	cv::Mat T = TransRotate(0, 0, oz);
	float data[16] = {T.at<float>(0, 0), T.at<float>(0, 1), T.at<float>(0, 2), float(x),
					  T.at<float>(1, 0), T.at<float>(1, 1), T.at<float>(1, 2), float(y),
					  T.at<float>(2, 0), T.at<float>(2, 1), T.at<float>(2, 2), float(0),
					  0, 0, 0, 1};
	cv::Mat tmp = cv::Mat(4, 4, CV_32FC1, data);
	SetPosition(tmp);
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
	if (goal_height > 400) {
		cout << "[ScaraArm] Too high" << endl;
		return false;
	}
	else if (goal_height < 20) {
		cout << "[ScaraArm] Too low" << endl;
		return false;
	}
	else if (goal_height == now_height) {
		cout << "[ScaraArm] Screw arrival !" << endl;
		return true;
	}
	else {
		float delta_height = goal_height - now_height;
		float delta_angle = delta_height / REV_2_SCREW * 360;

		// Motor angle is set to present angle + angle needed
		SetMotor_Angle(FIRST_HAND_ID, delta_angle + GetMotor_PresentAngle(FIRST_HAND_ID));

		// cout << "delta_height: " << delta_height << ", delta_angle: " << delta_angle << endl;
		// cout << "desire angle: " << GetMotor_Angle(FIRST_HAND_ID) << endl;
		this_thread::sleep_for(chrono::milliseconds(500));

		WaitMotorArrival(FIRST_HAND_ID);
		this_thread::sleep_for(chrono::milliseconds(50));

		SetMotor_Velocity(FIRST_HAND_ID, 500);

		WriteHeight(goal_height);
		now_height = goal_height;

		// cout << "sp_angle: " << GetMotor_PresentAngle(FIRST_HAND_ID) << endl;
		cout << "[ScaraArm] Screw arrival !" << endl;
		return true;
	}
}

/**
 * Calculate vector length
 * 
 * @param vec - vector we want to calculate, in cv::Mat
 * @retval - vector length
 */
float vec_length(cv::Mat vec) {
	return sqrt(vec.at<float>(0, 0)*vec.at<float>(0, 0) + vec.at<float>(1, 0)*vec.at<float>(1, 0) + vec.at<float>(2, 0)*vec.at<float>(2, 0));
}

/**
 * Calculate speed vector for the end effector to move
 * 
 * @param head - start of the movement
 * @param tail - end of the movement
 * @param speed - moving speed of the scara arm
 * @param ans - result to return
 * @retval - (3, 1) matrix
 */
cv::Mat ScaraArm::cal_vel(cv::Mat head, cv::Mat tail, float speed) {
	cv::Mat dir = tail - head;
	float dir_len = vec_length(dir);
	float scale = speed / dir_len;

	dir.at<float>(0,0) = dir.at<float>(0,0) * scale;
	dir.at<float>(0,1) = dir.at<float>(0,1) * scale;
	dir.at<float>(0,2) = 0;
	
	return dir.clone();
}

/**
 * Go straight function
 * For scara able to move in straight line
 * 
 * @param goal - goal of the movement
 * @param h - Move in height
 * @param speed_max - speed of the end effector
 */
void ScaraArm::go_straight(float *goal, float h, float speed_max) {
	// Go to height first
	GoScrewHeight(h);

	// Calculate start position for this action
	// Motor present state
	float theta1 = GetMotor_PresentAngle(FIRST_HAND_ID + 1) * Angle2Rad;
	float theta2 = J2_sign * GetMotor_PresentAngle(FIRST_HAND_ID + 2) * Angle2Rad;
	float theta3 = GetMotor_PresentAngle(FIRST_HAND_ID + 3) * Angle2Rad;
	cv::Mat start_pos = Calculate_ArmForwardKinematics(theta1, theta2, theta3).colRange(3,4).rowRange(0, 3);

	// Create a Mat data type
	cv::Mat goal_m(3,1,cv::DataType<float>::type);
	goal_m.at<float>(0,0) = goal[3];
	goal_m.at<float>(1,0) = goal[4];
	
	// Set all velocity to zero
	for (int i = 1; i < 4; i++) {
		SetMotor_Operating_Mode(FIRST_HAND_ID + i, 1);	//Change motor i operating mode to velocity mode	
		SetMotor_Velocity(FIRST_HAND_ID + i, 0);
		SetMotor_Accel(FIRST_HAND_ID + i, 0);
		SetMotor_Operating_Mode(FIRST_HAND_ID + i, 3);	//Change motor i operating mode to position mode	
	}

	// Set motor desire angle
	SetPosition(0,0, goal[2], goal[3], goal[4], 0);

	// Speed we want our end effector to move through time
	float speed_p = 0;

	// Adjust motor speed until all motor has arrived
	while(!CheckAllMotorsArrival()) {
		// Motor present state
		theta1 = GetMotor_PresentAngle(FIRST_HAND_ID + 1) * Angle2Rad;
		theta2 = J2_sign * GetMotor_PresentAngle(FIRST_HAND_ID + 2) * Angle2Rad;
		theta3 = GetMotor_PresentAngle(FIRST_HAND_ID + 3) * Angle2Rad;

		// Get end effector position
		cv::Mat effector_pos = Calculate_ArmForwardKinematics(theta1, theta2, theta3).colRange(3,4).rowRange(0, 3);

		// Calculate the speed we want our end effector to move with,
		// because we want the speed is set as trapezoid.
		// Use the distance between start, present, and end position,
		// Use the distance directly as the speed 
		
		// Length from start to present position
		float start_dis = vec_length( start_pos - effector_pos );
		// Length from end to present position
		float end_dis = vec_length( goal_m - effector_pos );

		// Find minimum speed
		speed_p = start_dis * Acc_Factor;
		if (end_dis < start_dis)
			speed_p = end_dis * Acc_Factor;
		if (speed_max < speed_p)
			speed_p = speed_max;

        // Calculate Jacobian
        float a1_s1   = Arm2_Length * sin(theta1);
        float a1_c1   = Arm2_Length * cos(theta1);
        float a2_s12  = Arm3_Length * sin(theta1 + theta2);
        float a2_c12  = Arm3_Length * cos(theta1 + theta2);
        float a3_s123 = Arm4_Length * sin(theta1 + theta2 + theta3);
        float a3_c123 = Arm4_Length * cos(theta1 + theta2 + theta3);

		float J_f[] = { -1*a1_s1-a2_s12-a3_s123, -1*a2_s12-a3_s123, -1*a3_s123,
						 1*a1_c1+a2_c12+a3_c123,  1*a2_c12+a3_c123,  1*a3_c123,
						 1                     ,  1               , 		1};
		cv::Mat J(3,3,cv::DataType<float>::type, J_f);
        
		// Calculate velocity direction, aka v_dir
		cv::Mat v_dir = cal_vel(effector_pos, goal_m, speed_p);

		// Use Jacobian to map back to joint space
		cv::Mat J_inv = J.inv();
		cv::Mat j_speed = J_inv * v_dir;
		// cout << j_speed << endl << endl;

		for (int i = 1; i < 4; i++) {
			int motor_speed = round( j_speed.at<float>(i-1) * 60 / 6.28318 / 0.01 );			// Angular velocity(rad/s) * 60(1sec to 1min) / 2pi / unit scale(0.01) = ? rev / min
			if (motor_speed == 0)		// if the speed is 0, set it to 1
				motor_speed += 1;

			// Set motor i to desire speed
			SetMotor_Velocity(FIRST_HAND_ID + i, abs( motor_speed ));
			// SetMotor_Accel(FIRST_HAND_ID + i, 0);
			// cout << i << "'s speed = " << motor_speed ;
		}
		//cout << "test" << endl;

		this_thread::sleep_for(chrono::milliseconds(200));	// sleep time is decided by try and error, 
															// Too short, the motor is unable to set velocity
															// Too long, the adjustment is not real-time

	}	// End of while

	// Turn back to original speed and acceleration
	SetMotor_Velocity(FIRST_HAND_ID + 1, 500);
	SetMotor_Accel(FIRST_HAND_ID + 1, 250);
	SetMotor_Velocity(FIRST_HAND_ID + 2, 500);
	SetMotor_Accel(FIRST_HAND_ID + 2, 250);
	SetMotor_Velocity(FIRST_HAND_ID + 3, 1000);
	SetMotor_Accel(FIRST_HAND_ID+3, 250);
	
	cout << "[ScaraArm] Arm arrival !" << endl;

}

/**
 * Go straight function
 * For scara able to move in straight line
 * 
 * @param head - start of the movement
 * @param tail - end of the movement
 * @param h - height of the scara arm
 * @param div - How many pieces we want to cut our movement
 */
void ScaraArm::go_straight_tmp(float *head, float *tail, float h, float div) {
    float step[] = {0, 0};

    // unit vector of target position
    step[0] = (tail[3] - head[3])/div;
    step[1] = (tail[4] - head[4])/div;

    // Move straight by insert points into line
    for (int i = 0; i < div; i++) {
        GotoPosition(0,0, head[2], (head[3] + i * step[0]), (head[4] + i * step[1]), h);
        // printf("readyx: %f, readyy: %f, x: %f, y: %f, z: %f\n",head[3], head[4], (head[3] + i * step[0]), (head[4] + i * step[1]), h);   // DEBUG
    }
}

/**
 * Move scara arm to initial point
 */
void ScaraArm::Reset(){
	GotoPosition(0, 0, 62, 437, 459, 0);
	GoScrewHeight(250.0f);
}