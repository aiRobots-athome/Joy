#include "ScaraArm.h"

ScaraArm *ScaraArm::inst_ = nullptr;
ScaraArm *ScaraArm::getScaraArm()
{
	if (inst_ == nullptr)
		inst_ = new ScaraArm();
	return inst_;
}

ScaraArm::ScaraArm()
	: MotorUnion({0, 1, 2, 3}, {"Pro200", "Pro200", "Pro20", "Pro20"}),
	  FIRST_HAND_ID(0),
	  Arm1_Length(168), // 53-small
	  Arm2_Length(390), // 92
	  Arm3_Length(238), // 92
	  Arm4_Length(242), // 69
	  Degree2Resolution(1003846 / 360)
{
	ReadHeight();

	SetMotor_Operating_Mode(FIRST_HAND_ID, 1);	//Pro 200 change operating mode to velocity mode
	SetMotor_Accel(FIRST_HAND_ID, 200);			//because of setting velocity in GOScrewHeight to reduce the raising time
	SetMotor_Velocity(FIRST_HAND_ID, 0);		// Pro200 if set velocity motor will operate, because of velocity mode
	SetMotor_Velocity(FIRST_HAND_ID + 1, 500);	// Pro200 initial
	SetMotor_Velocity(FIRST_HAND_ID + 2, 1000); // Pro20 initial
	SetMotor_Velocity(FIRST_HAND_ID + 3, 1500); // Pro20 initial

	cout << "\t\tClass constructed: ScaraArm" << endl;
}

cv::Mat ScaraArm::GetKinematics()
{
	float fRadian1 = GetMotor_PresentAngle(FIRST_HAND_ID + 1) * Angle2Rad;
	float fRadian2 = -GetMotor_PresentAngle(FIRST_HAND_ID + 2) * Angle2Rad;
	float fRadian3 = GetMotor_PresentAngle(FIRST_HAND_ID + 3) * Angle2Rad;

	return Calculate_ArmForwardKinematics(fRadian1, fRadian2, fRadian3);
}

float &ScaraArm::GetPresentHeight()
{
	return now_height;
}

cv::Mat ScaraArm::Calculate_ArmForwardKinematics(const float &J1, const float &J2, const float &J3)
{
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

float *ScaraArm::Arm_InverseKinematics(const cv::Mat &T)
{
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
					  Arm3_Length * Arm2_Length * 2);

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
				temp_theta[1] = -temp_theta[1];
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

	// Is it possible?
	if (Solution[0] == 0 && Solution[1] == 0 && Solution[2] == 0)
	{
		ScaraArmMotionEnable = false;
	}

	return Solution;
}

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
	}
	else
	{
		cout << "[ScaraArm] Fail to arrive" << endl;
	}
}

void ScaraArm::GotoPosition(const int &ox, const int &oy, const int &oz, const int &x, const int &y, const int &z)
{
	cv::Mat T = TransRotate(ox, oy, oz);
	float data[16] = {T.at<float>(0, 0), T.at<float>(0, 1), T.at<float>(0, 2), float(x),
					  T.at<float>(1, 0), T.at<float>(1, 1), T.at<float>(1, 2), float(y),
					  T.at<float>(2, 0), T.at<float>(2, 1), T.at<float>(2, 2), float(z),
					  0, 0, 0, 1};
	cv::Mat tmp = cv::Mat(4, 4, CV_32FC1, data);
	GotoPosition(tmp);
}

cv::Mat ScaraArm::TransRotate(const float &ox, const float &oy, const float &oz)
{
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

void ScaraArm::WriteHeight(const float &height) const
{
	fstream heightfile;
	string path = string(getenv("PWD")) + "/src/Scara/ScaraArm/Height.txt";
	heightfile.open(path, ios::trunc);
	if (heightfile.fail())
		cout << "[ScaraArm] Cannot open Height.txt" << endl;
	else
	{
		heightfile << height;
		heightfile.close();
	}
}
// Need to check
void ScaraArm::GoScrewHeight(const float &goal_height) //unit : mm
{
	if (goal_height >= 400)
	{
		cout << "[ScaraArm] Too high" << endl;
	}
	else if (goal_height <= 20)
	{
		cout << "[ScaraArm] Too low" << endl;
	}

	float delta_height = goal_height - now_height;
	int now_position = GetMotor_PresentAngle(FIRST_HAND_ID) * Degree2Resolution; //!!!!! Present angle isn't accuracy
	// 224(Speed ​​increaser ratio 1:11.05, Pro200 1rev = Screw 224mm)  1003846(Pro200 resolution)
	int delta_postion = round(delta_height / 224 * 1003846);
	int need_position = now_position + delta_postion;
	int delta_height_f = 0;
	if (abs(delta_height) > 10)
	{
		if (delta_height > 0)
			delta_height_f = delta_height - 5;
		else if (delta_height < 0)
			delta_height_f = delta_height + 5;
		float motorspeed = 200 * 0.01 / 60 * 226;					//Velocity(set 400) * Scale(rev/min) / 60(to 1sec) * 226(mm) (Pro200 1rev = Screw 226)
		float waittime = (abs(delta_height_f) / motorspeed) * 1000; //unit:milliseconds
		if (delta_height < 0)
		{
			SetMotor_Velocity(FIRST_HAND_ID, -200);
		}
		else if (delta_height > 0)
		{
			SetMotor_Velocity(FIRST_HAND_ID, 200);
		}
		WaitAllMotorsArrival(waittime);
		SetMotor_Velocity(FIRST_HAND_ID, 0);
		this_thread::sleep_for(chrono::milliseconds(500));
	}
	int present_position = GetMotor_PresentAngle(FIRST_HAND_ID) * Degree2Resolution;
	int delta_resolution = need_position - present_position;

	for (int j = 0;; j++)
	{
		SetMotor_Velocity(FIRST_HAND_ID, copysign(20, delta_resolution));
		this_thread::sleep_for(chrono::milliseconds(100));
		int changing_position2 = GetMotor_PresentAngle(FIRST_HAND_ID) * Degree2Resolution;

		if (abs(changing_position2 - need_position) < 2500)
			break;
	}
	SetMotor_Velocity(FIRST_HAND_ID, 0);
	WriteHeight(goal_height);
}

void ScaraArm::GotoPosition(const int &height,
							const int &ox,
							const int &oy,
							const int &oz,
							const int &x,
							const int &y,
							const int &z)
{
	GoScrewHeight(height);
	GotoPosition(ox, oy, oz, x, y, 0); //Can we input z?
}

void ScaraArm::SetAllMotorsTorqueEnable(const bool &torque)
{
	SetAllMotorsTorqueEnable(torque);
}