#include "ScaraArm.h"

ScaraArm::ScaraArm(const vector<unsigned char>& IDArray,
						   const vector<string>& MotorModelArrayList,
						   vector<unsigned char>& AllPortNumber)
:MotorUnion(IDArray, MotorModelArrayList, AllPortNumber),
	  FIRST_HAND_ID(0),
	  HAND_AMOUNT(3)
{
	// Scara Arm Length
    //BIG
	alength1_ini = 168;
    alength2_ini = 390;
    alength3_ini = 238;
    alength4_ini = 242;
	//total = 1038

    //SMALL
    // alength1_ini = 53;
    // alength2_ini = 92;
    // alength3_ini = 92;
    // alength4_ini = 69;
	int FIRST_HAND_ID = 0;
	ArmForward = new cv::Mat(4, 4, CV_32FC1);
	InitArmMotor();
}

void ScaraArm::InitArmMotor()
{
	SetMotor_Operating_Mode(FIRST_HAND_ID, 1);  //Pro 200 change operating mode to velocity mode
	SetMotor_Velocity(FIRST_HAND_ID, 0);		// Pro200 if set velocity motor will operate, because of velocity mode
	SetMotor_Accel(FIRST_HAND_ID, 200);         //because of setting velocity in GOScrewHeight to reduce the raising time
	SetMotor_Velocity(FIRST_HAND_ID + 1, 1500); // Pro200 initial
	SetMotor_Velocity(FIRST_HAND_ID + 2, 1500); // Pro20 initial
	SetMotor_Velocity(FIRST_HAND_ID + 3, 1500); // Pro20 initial
}

cv::Mat* ScaraArm::GetKinematics(void)
{
    float fRadian1 = GetMotor_PresentAngle(FIRST_HAND_ID + 1) * Angle2Rad; 
	float fRadian2 = -GetMotor_PresentAngle(FIRST_HAND_ID + 2) * Angle2Rad;
	float fRadian3 = GetMotor_PresentAngle(FIRST_HAND_ID + 3) * Angle2Rad;

    return Calculate_ArmForwardKinematics(fRadian1, fRadian2, fRadian3);
}

cv::Mat* ScaraArm::Calculate_ArmForwardKinematics(float J1, float J2, float J3)
{
	cv::Matx44f TransMatrix_01(1, 0, 0, alength1_ini,
					           0, 1, 0, 0,
					           0, 0, 1, 0,
					           0, 0, 0, 1 );
	cv::Matx44f TransMatrix_12( cos( J1 ), -sin( J1 ), 0, alength2_ini*cos( J1 ),
					            sin( J1 ), cos( J1 ), 0, alength2_ini*sin( J1 ),
		                        0, 0, 1, 0,
		                        0, 0, 0, 1 );
	cv::Matx44f TransMatrix_23(cos(J2), -sin(J2), 0, alength3_ini*cos(J2),
		                       sin( J2 ), cos( J2 ),0, alength3_ini*sin( J2 ),
		                       0, 0, 1, 0,
		                       0, 0, 0, 1 );
	cv::Matx44f TransMatrix_34(cos(J3), -sin(J3), 0, alength4_ini*cos(J3),
		                       sin(J3), cos(J3), 0, alength4_ini*sin(J3),
		                       0, 0, 1, 0,
		                       0, 0, 0, 1);
	
	cv::Matx44f Temp( TransMatrix_01*TransMatrix_12*TransMatrix_23*TransMatrix_34 );

	
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			this->ArmForward->at<float>( i, j ) = Temp( i, j );
		}
	}
	return this->ArmForward;
}

float* ScaraArm::Arm_InverseKinematics(cv::Mat* T)
{
	float nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz;

	cv::Mat* tempT = T;
	nx = T->at<float>( 0, 0 );
	ny = T->at<float>( 1, 0 );
	nz = T->at<float>( 2, 0 );
	ox = T->at<float>( 0, 1 );
	oy = T->at<float>( 1, 1 );
	oz = T->at<float>( 2, 1 );
	ax = T->at<float>( 0, 2 );
	ay = T->at<float>( 1, 2 );
	az = T->at<float>( 2, 2 );
	px = T->at<float>( 0, 3 );
	py = T->at<float>( 1, 3 );
	pz = T->at<float>( 2, 3 );

	this->ScaraArmMotionEnable = false;
	float J1, J2, J3 ;

	float delta_Pos = 150000 * 2 + 250950 * 4;  // default value for calculate delta solution(very large scale)                  

	float theta2[2];  // J1
	float theta3[2];  // J2
	float theta4[2];  // J3

	for (int i = 0; i < 2; i++)
	{
		theta2[i] = -999;  // J1
		theta3[i] = -999;  // J2
		theta4[i] = -999;  // J3
	}

	float* Solution = new float[3];  // J1 J2 J3
	for (int i = 0; i < 3; i++)
	{
		Solution[i] = 0;
	}

	// J2 solution have two
	float J2_1 = acos((pow((px - (alength4_ini*nx) - alength1_ini), 2) + pow(py - (alength4_ini*ny), 2) - pow(alength3_ini, 2) - pow(alength2_ini, 2)) / (alength3_ini * alength2_ini * 2));
	theta3[0] = (J2_1) / Angle2Rad; //positive solution
	theta3[1] = - theta3[0]; //nagetive solution

	//angel normalization
	for (int i = 0; i < 2; i++)
	{
		if (theta3[i] > 180)
		{
			theta3[i] = theta3[i] - 360;
		}
		if (theta3[i] < -180)
		{
			theta3[i] = theta3[i] + 360;
		}

		if ((theta3[i] > -180) && (theta3[i] < -155)) //critical point
		{
			theta3[i] = -999;
		}
		else if ((theta3[i] > 155) && (theta3[i] < 180)) //critical point
		{
			theta3[i] = -999;
		}
	}

	for (int i = 0; i < 2; i++)  /* two solution */
	{
		if (theta3[i] != -999)  // J2 sol
		{
			J2 = theta3[i] * Angle2Rad;  
			float J1_r = pow((pow((alength3_ini*cos(J2) + alength2_ini), 2) + pow((alength3_ini*sin(J2)), 2)), 0.5);
			float J1_phi = atan2((alength3_ini*sin(J2)), (alength3_ini*cos(J2) + alength2_ini));
			theta2[i] = (-J1_phi + atan2(((py - (alength4_ini*ny)) / J1_r), pow((1 - (pow(((py - (alength4_ini*ny)) / J1_r), 2))), 0.5))) * Rad2Angle;

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
		if (theta2[i] != -999)  // J1 sol
		{
			J1 = theta2[i] * Angle2Rad;  // change to radius
			theta4[i] = atan2((py*cos(J1 + J2) + alength1_ini*sin(J1 + J2) - px*sin(J1 + J2) + alength2_ini*sin(J2)), px*cos(J1 + J2) - alength1_ini*cos(J1 + J2) - alength3_ini + py*sin(J1 + J2) - alength2_ini*cos(J2)) * Rad2Angle;

			//angel normalize
			if (theta4[i] > 180)
			{
				theta4[i] = theta4[i] - 360;
			}
			if (theta4[i] < -180)
			{
				theta4[i] = theta4[i] + 360;
			}

			if ((theta4[i] > -180) && (theta4[i] < -155)) //limit
			{
				theta4[i] = -999;
			}
			else if ((theta4[i] > 155) && (theta4[i] < 180)) //limit
			{
				theta4[i] = -999;
			}
		}

		if (theta4[i] != -999)  // J3 sol
		{
			J3 = theta4[i] * Angle2Rad;  //change to radius

			bool check = true;
			//check kinematics
			float  j1, j2, j3;
			// solution change to degree
			j1 = J1*Rad2Angle;
			j2 = J2*Rad2Angle;
			j3 = J3*Rad2Angle;

			cv::Mat* tmpT = Calculate_ArmForwardKinematics(J1, J2, J3);  // use inverse kinematics solution to get forward kinematics

			for (int o = 0; o < 4; o++)
			{
				for (int p = 0; p < 4; p++)
				{
					// check does it same inverse and forward
					if (round(round_value*tmpT->at<float>(o, p)) != round(round_value*T->at<float>(o, p)))
					{
						if (std::abs(round(round_value*tmpT->at<float>(o, p)) - round(round_value*T->at<float>(o, p))) > 1)
						{
							check = false;
						}
					}
				}
			}
			if (check)  
			{
				float temp_theta[3]; //re_theta[4]
				temp_theta[0] = J1 * Rad2Angle;
				temp_theta[1] = J2 * Rad2Angle;
				temp_theta[2] = J3 * Rad2Angle;
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

				temp_theta[0] = (temp_theta[0]);
				temp_theta[1] = -(temp_theta[1]);
				temp_theta[2] = (temp_theta[2]);

				// get delta angle
				int tmp_delta_Ang =
					std::abs(temp_theta[0] - GetMotor_PresentAngle(FIRST_HAND_ID + 1))
					+ std::abs(temp_theta[1] - GetMotor_PresentAngle(FIRST_HAND_ID + 2))
					+ std::abs(temp_theta[2] - GetMotor_PresentAngle(FIRST_HAND_ID + 3));

				// check delta limit
				if (tmp_delta_Ang < delta_Pos)
				{
					delta_Pos = tmp_delta_Ang;

					for (int o = 0; o < 3; o++)
					{
						Solution[o] = temp_theta[o];
					}
					this->ScaraArmMotionEnable = true;  // let torque enable be true
				}
			}
		}
	}

	if (Solution[0] == 0 && Solution[1] == 0 && Solution[2] == 0 )
	{
		this->ScaraArmMotionEnable = false;  
	}
    
	return Solution;
}

void ScaraArm::GotoPosition(cv::Mat *&T)
{
	float *temp = Arm_InverseKinematics(T);
	if (ScaraArmMotionEnable)
	{
		SetMotor_Angle(FIRST_HAND_ID, GetMotor_PresentAngle(FIRST_HAND_ID));
		SetMotor_Angle(FIRST_HAND_ID + 1, temp[0]);
		SetMotor_Angle(FIRST_HAND_ID + 2, temp[1]);
		SetMotor_Angle(FIRST_HAND_ID + 3, temp[2]);
		WaitAllMotorsArrival();
	}
	else
	{
		cout << "Fail to arrive" << endl;
	}
}

void ScaraArm::GotoPosition(const int &ox, const int &oy, const int &oz, const int &x, const int &y, const int &z)
{
	cv::Mat *tmp = new cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));
	cv::Mat *T = TransRotate(ox, oy, oz);

	tmp->at<float>(0, 0) = T->at<float>(0, 0);
	tmp->at<float>(0, 1) = T->at<float>(0, 1);
	tmp->at<float>(0, 2) = T->at<float>(0, 2);
	tmp->at<float>(0, 3) = x;
	tmp->at<float>(1, 0) = T->at<float>(1, 0);
	tmp->at<float>(1, 1) = T->at<float>(1, 1);
	tmp->at<float>(1, 2) = T->at<float>(1, 2);
	tmp->at<float>(1, 3) = y;
	tmp->at<float>(2, 0) = T->at<float>(2, 0);
	tmp->at<float>(2, 1) = T->at<float>(2, 1);
	tmp->at<float>(2, 2) = T->at<float>(2, 2);
	tmp->at<float>(2, 3) = z;
	tmp->at<float>(3, 0) = 0;
	tmp->at<float>(3, 1) = 0;
	tmp->at<float>(3, 2) = 0;
	tmp->at<float>(3, 3) = 1;
	GotoPosition(tmp);
	
	delete tmp;
}

cv::Mat *ScaraArm::TransRotate(const float &x, const float &y, const float &z)
{
	float x_angle = x * Angle2Rad;
	float y_angle = y * Angle2Rad;
	float z_angle = z * Angle2Rad;
	float temp_nx = cos(z_angle) * cos(y_angle);
	float temp_ny = sin(z_angle) * cos(y_angle);
	float temp_nz = -sin(y_angle);
	float temp_ox = cos(z_angle) * sin(y_angle) * sin(x_angle) - sin(z_angle) * cos(x_angle);
	float temp_oy = sin(z_angle) * sin(y_angle) * sin(x_angle) + cos(z_angle) * cos(x_angle);
	float temp_oz = cos(y_angle) * sin(x_angle);
	float temp_ax = cos(z_angle) * sin(y_angle) * cos(x_angle) + sin(z_angle) * sin(x_angle);
	float temp_ay = sin(z_angle) * sin(y_angle) * cos(x_angle) - cos(z_angle) * sin(x_angle);
	float temp_az = cos(y_angle) * cos(x_angle);
	cv::Mat *tmpMat = new cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
	tmpMat->at<float>(0, 0) = temp_nx;
	tmpMat->at<float>(0, 1) = temp_ox;
	tmpMat->at<float>(0, 2) = temp_ax;
	tmpMat->at<float>(1, 0) = temp_ny;
	tmpMat->at<float>(1, 1) = temp_oy;
	tmpMat->at<float>(1, 2) = temp_ay;
	tmpMat->at<float>(2, 0) = temp_nz;
	tmpMat->at<float>(2, 1) = temp_oz;
	tmpMat->at<float>(2, 2) = temp_az;

	return tmpMat;
}

void ScaraArm::StopScrew()
{
	SetMotor_Velocity(FIRST_HAND_ID, 0);
	// SetMotor_TorqueEnable(FIRST_HAND_ID, false);
}

float ScaraArm::ReadSaveHeight()
{
	char save_height[200];
	float now_height;
	std::fstream heightfile;
	std::string homepath = getenv("HOME");
	std::string filepath = "/Joy/src/Scara/ScaraArm/Height.txt";
	std::string path = homepath + filepath;
	heightfile.open(path,ios::in);
	if (!heightfile)
		std::cout<< "Can't open" << std::endl;
	else
	{
		heightfile.read(save_height, sizeof(save_height));
		heightfile.close();
		now_height = std::stof(save_height);
	}

	return now_height;
}

void ScaraArm::WriteSaceHeight(float data)
{
	fstream heightfile;
	std::string homepath = getenv("HOME");
	std::string filepath = "/Joy/src/Scara/ScaraArm/Height.txt";
	std::string path = homepath + filepath;
	heightfile.open(path,ios::out);
	if (heightfile.fail())
		std::cout<< "Can't open" << std::endl;
	else
	{
		heightfile << data;
		heightfile.close();
	}
}

void ScaraArm::GOScrewHeight(float goal_height) //unit : mm
{
	tmp_height = 0;
	float degreetoresolution =  1003846 / 360;
	if (goal_height >=400)
	{
		std::cout << "Too high" << std::endl;
		StopScrew();
	}
	else if (goal_height <=20)
	{
		std::cout << "Too low" << std::endl;
		StopScrew();
	}
	else
	{
		tmp_height = ReadSaveHeight();
	}
	
	float delta_height = goal_height - tmp_height;
	int now_position = GetMotor_PresentAngle(FIRST_HAND_ID) * degreetoresolution;
	int delta_postion = round(delta_height / 226 * 1003846); //224(Speed ​​increaser ratio 1:11.05, Pro200 1rev = Screw 224mm)  1003846(Pro200 resolution)
	need_position = now_position + delta_postion;
	if (delta_height > 10 || delta_height <-10)
	{
		if (delta_height > 0)
			delta_height_f = delta_height - 5;
		else if(delta_height < 0)
			delta_height_f = delta_height + 5; 
		float motorspeed = 200*0.01/60*226;  //Velocity(set 400) * Scale(rev/min) / 60(to 1sec) * 226(mm) (Pro200 1rev = Screw 226)
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
	int present_position = GetMotor_PresentAngle(FIRST_HAND_ID) * degreetoresolution;
	int delta_resolution = need_position - present_position;
	if(delta_resolution > 0)
	{
		for (int j = 0; ; j++)
		{
			SetMotor_Velocity(FIRST_HAND_ID, 20);
			this_thread::sleep_for(chrono::milliseconds(100));
			int changing_position2 = GetMotor_PresentAngle(FIRST_HAND_ID) * degreetoresolution;
			if (changing_position2 > (need_position-2500) && changing_position2 < (need_position+2500))
			{
				break;
			}
		}
	}
	else if (delta_resolution < 0)
	{
		for (int i = 0; ; i++)
		{
			SetMotor_Velocity(FIRST_HAND_ID, -20);
			this_thread::sleep_for(chrono::milliseconds(100));
			int changing_position2 = GetMotor_PresentAngle(FIRST_HAND_ID) * degreetoresolution;
			if (changing_position2 > (need_position-2500) && changing_position2 < (need_position+2500))
			{
				break;
			}
		}
	}
	SetMotor_Velocity(FIRST_HAND_ID, 0);
	WriteSaceHeight(goal_height);
}

void ScaraArm::ScaraGO(float ox, float oy, float oz, float x, float y, float z)
{
	GOScrewHeight(z);
	GotoPosition(ox, oy, oz, x, y, 0);
}

void ScaraArm::Reset()
{
	GOScrewHeight(224);
	GotoPosition(0,0,0,0,700,0);
}

void ScaraArm::AllmoterTorque(bool torque)
{
	if (torque == true)
		SetAllMotorsTorqueEnable(true);
	else if (torque == false)
		SetAllMotorsTorqueEnable(false);
}
