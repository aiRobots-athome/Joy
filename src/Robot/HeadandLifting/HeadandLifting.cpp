#include "HeadandLifting.h"

HeadandLifting *HeadandLifting::inst_ = nullptr;
HeadandLifting *HeadandLifting::getHeadandLifting()
{
	if (inst_ == nullptr)
		inst_ = new HeadandLifting();
	return inst_;
}

HeadandLifting::HeadandLifting()
	: MotorUnion({8, 9}, {"Mx64", "Mx64"})
{
	/* ID Initial */
	this->FIRST_HEAD_MOTOR_ID = 0;

	this->LEFT_LIFTING_MOTOR_ID = 2;
	this->RIGHT_LIFTING_MOTOR_ID = 3;

	/* Distance Initial */
	this->DisKinectToMotor8 = 90;
	this->DisMotor8ToCenter = 110;
	this->DisShoulder2Waist = 0;

	R_ID8 = new cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0)); // overview
	R_ID9 = new cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0)); // horizon

	T_DisID8_ID9 = new cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));
	T_DisCamera2ID8 = new cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));
	T_Lens2CameraCenter = new cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));
	T_CamCoord2RobotCoord = new cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));
	T_CamCoord2RobotCoord_inv = new cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));

	this->InitHeadMotorVelocity();
	cout << "\tClass constructed: HeadandLifting" << endl;
}

HeadandLifting::~HeadandLifting()
{
	delete R_ID8;
	delete R_ID9; 
	delete T_DisID8_ID9;
	delete T_DisCamera2ID8;
	delete T_Lens2CameraCenter;
	delete T_CamCoord2RobotCoord;
	delete T_CamCoord2RobotCoord_inv;
	inst_ = nullptr;
}

void HeadandLifting::InitHeadMotorVelocity(void)
{
	this->SetMotor_Velocity(FIRST_HEAD_MOTOR_ID, 200);	   // Mx64
	this->SetMotor_Velocity(FIRST_HEAD_MOTOR_ID + 1, 200); // Mx64
	SetAllMotorsTorqueEnable(true);
}

void HeadandLifting::HeadMotorCommand(const int &MotorID, const float &Angle)
{
	// Bcuz SetMotor_Angle is protected in MotorUnion, it must be implemented in HeadandLifting
	this->SetMotor_Angle(MotorID, Angle);
}

void HeadandLifting::ResetAllMotorAngle()
{
	this->SetAllMotorsTorqueEnable(true);
	this->SetMotor_Angle(FIRST_HEAD_MOTOR_ID, 0);
	this->SetMotor_Angle(FIRST_HEAD_MOTOR_ID + 1, 0);
	this->WaitAllMotorsArrival();
}

void HeadandLifting::HeadCoordinate(float X_kinect, float Y_kinect, float depth, float &X_real, float &Y_real, float &Z_real)
{
	int type = 1;
	float Theta_x, Theta_y, World_X, World_Y, J1, J2, J3, Kx, Ky;

	// In degree
	J1 = this->GetMotor_PresentAngle(FIRST_HEAD_MOTOR_ID);
	J2 = this->GetMotor_PresentAngle(FIRST_HEAD_MOTOR_ID + 1);
	J3 = 0;

	J1 = J1 * M_PI / 180.0;
	J2 = -J2 * M_PI / 180.0;
	J3 = J3 * M_PI / 180.0;

	if (type == 1)
	{
		Kx = X_kinect * 1000 - 35;
		Ky = Y_kinect * 1000;
		depth = depth * 1000 + 27;

		if (depth > -1000 && depth < 6000)
		{
			X_real = sin(J3) * (sin(J2) * (sin(J1) * (DisKinectToMotor8 + Ky) - depth * cos(J1)) + Kx * cos(J2)) + cos(J3) * (DisShoulder2Waist - cos(J2) * (sin(J1) * (DisKinectToMotor8 + Ky) - depth * cos(J1)) + Kx * sin(J2)) + 100;
			Y_real = cos(J3) * (sin(J2) * (sin(J1) * (DisKinectToMotor8 + Ky) - depth * cos(J1)) + Kx * cos(J2)) -
					 sin(J3) * (DisShoulder2Waist - cos(J2) * (sin(J1) * (DisKinectToMotor8 + Ky) - depth * cos(J1)) + Kx * sin(J2)) + 80;
			Z_real = DisMotor8ToCenter + cos(J1) * (DisKinectToMotor8 + Ky) + (depth)*sin(J1);
		}
		else
		{
			X_real = -1;
			Y_real = -1;
			Z_real = -1;
		}
	}
	else if (type == 0)
	{
		Kx = X_kinect * 1000;
		Ky = Y_kinect * 1000;
		depth = depth * 1000;

		if (depth > -1000 && depth < 6000)
		{
			X_real = Ky + 609 + 3;
			Y_real = Kx - 60 + 50;
			Z_real = -depth + 916 + 6;
		}
		else
		{
			X_real = -1;
			Y_real = -1;
			Z_real = -1;
		}
	}
	else if (type == 2)
	{

		Kx = X_kinect * 1000 - 35 - 25 - 5 - 15 + 27;
		Ky = Y_kinect * 1000 + 605 + 76 - 119 + 44 - 16;
		depth = depth * 1000 - 1000 - 106 + 130 + 40 - 22 - 4 + 80 - 25;

		if (depth > -1000 && depth < 6000)
		{
			X_real = Ky;
			Y_real = Kx;
			Z_real = -depth;
		}
		else
		{
			X_real = -1;
			Y_real = -1;
			Z_real = -1;
		}
	}
}

void HeadandLifting::RobotToCamera(float X_real, float Y_real, float Z_real, float &X_kinect, float &Y_kinect, float &depth)
{
	float kx, ky, kz;

	cv::Mat *Pt_Robot_Coordinate = new cv::Mat(4, 1, CV_32FC1, cv::Scalar::all(0));
	Pt_Robot_Coordinate->at<float>(0, 0) = X_real;
	Pt_Robot_Coordinate->at<float>(1, 0) = Y_real;
	Pt_Robot_Coordinate->at<float>(2, 0) = Z_real;
	Pt_Robot_Coordinate->at<float>(3, 0) = 1;

	this->Get_T_CamCoord2RobotCoord();

	cv::Mat *Pt_Camera_Coordinate = new cv::Mat(4, 1, CV_32FC1, cv::Scalar::all(0));

	*this->T_CamCoord2RobotCoord_inv = this->T_CamCoord2RobotCoord->inv();
	*Pt_Camera_Coordinate = *this->T_CamCoord2RobotCoord_inv * (*Pt_Robot_Coordinate);

	kx = Pt_Camera_Coordinate->at<float>(0, 0);
	ky = Pt_Camera_Coordinate->at<float>(1, 0);
	kz = Pt_Camera_Coordinate->at<float>(2, 0);

	X_kinect = kx / 1000;
	Y_kinect = ky / 1000;
	depth = kz / 1000;
}

void HeadandLifting::Get_T_CamCoord2RobotCoord()
{
	float J1 = this->GetMotor_PresentAngle(FIRST_HEAD_MOTOR_ID);
	float J2 = this->GetMotor_PresentAngle(FIRST_HEAD_MOTOR_ID + 1);

	J1 = J1 * M_PI / 180.0;
	J2 = J2 * M_PI / 180.0;

	this->R_ID9->at<float>(0, 0) = cos(J2);
	this->R_ID9->at<float>(0, 1) = -sin(J2);
	this->R_ID9->at<float>(0, 2) = 0;
	this->R_ID9->at<float>(0, 3) = 0;

	this->R_ID9->at<float>(1, 0) = sin(J2);
	this->R_ID9->at<float>(1, 1) = cos(J2);
	this->R_ID9->at<float>(1, 2) = 0;
	this->R_ID9->at<float>(1, 3) = 0;

	this->R_ID9->at<float>(2, 0) = 0;
	this->R_ID9->at<float>(2, 1) = 0;
	this->R_ID9->at<float>(2, 2) = 1;
	this->R_ID9->at<float>(2, 3) = 0;

	this->R_ID9->at<float>(3, 0) = 0;
	this->R_ID9->at<float>(3, 1) = 0;
	this->R_ID9->at<float>(3, 2) = 0;
	this->R_ID9->at<float>(3, 3) = 1;

	this->T_DisID8_ID9->at<float>(0, 0) = 1;
	this->T_DisID8_ID9->at<float>(0, 1) = 0;
	this->T_DisID8_ID9->at<float>(0, 2) = 0;
	this->T_DisID8_ID9->at<float>(0, 3) = 0;

	this->T_DisID8_ID9->at<float>(1, 0) = 0;
	this->T_DisID8_ID9->at<float>(1, 1) = 1;
	this->T_DisID8_ID9->at<float>(1, 2) = 0;
	this->T_DisID8_ID9->at<float>(1, 3) = 0;

	this->T_DisID8_ID9->at<float>(2, 0) = 0;
	this->T_DisID8_ID9->at<float>(2, 1) = 0;
	this->T_DisID8_ID9->at<float>(2, 2) = 1;
	this->T_DisID8_ID9->at<float>(2, 3) = DisMotor8ToCenter;

	this->T_DisID8_ID9->at<float>(3, 0) = 0;
	this->T_DisID8_ID9->at<float>(3, 1) = 0;
	this->T_DisID8_ID9->at<float>(3, 2) = 0;
	this->T_DisID8_ID9->at<float>(3, 3) = 1;

	this->R_ID8->at<float>(0, 0) = cos(-J1);
	this->R_ID8->at<float>(0, 1) = 0;
	this->R_ID8->at<float>(0, 2) = sin(-J1);
	this->R_ID8->at<float>(0, 3) = 0;

	this->R_ID8->at<float>(1, 0) = 0;
	this->R_ID8->at<float>(1, 1) = 1;
	this->R_ID8->at<float>(1, 2) = 0;
	this->R_ID8->at<float>(1, 3) = 0;

	this->R_ID8->at<float>(2, 0) = -sin(-J1);
	this->R_ID8->at<float>(2, 1) = 0;
	this->R_ID8->at<float>(2, 2) = cos(-J1);
	this->R_ID8->at<float>(2, 3) = 0;

	this->R_ID8->at<float>(3, 0) = 0;
	this->R_ID8->at<float>(3, 1) = 0;
	this->R_ID8->at<float>(3, 2) = 0;
	this->R_ID8->at<float>(3, 3) = 1;

	this->T_DisCamera2ID8->at<float>(0, 0) = 1;
	this->T_DisCamera2ID8->at<float>(0, 1) = 0;
	this->T_DisCamera2ID8->at<float>(0, 2) = 0;
	this->T_DisCamera2ID8->at<float>(0, 3) = 0;

	this->T_DisCamera2ID8->at<float>(1, 0) = 0;
	this->T_DisCamera2ID8->at<float>(1, 1) = 1;
	this->T_DisCamera2ID8->at<float>(1, 2) = 0;
	this->T_DisCamera2ID8->at<float>(1, 3) = 0;

	this->T_DisCamera2ID8->at<float>(2, 0) = 0;
	this->T_DisCamera2ID8->at<float>(2, 1) = 0;
	this->T_DisCamera2ID8->at<float>(2, 2) = 1;
	this->T_DisCamera2ID8->at<float>(2, 3) = DisKinectToMotor8;

	this->T_DisCamera2ID8->at<float>(3, 0) = 0;
	this->T_DisCamera2ID8->at<float>(3, 1) = 0;
	this->T_DisCamera2ID8->at<float>(3, 2) = 0;
	this->T_DisCamera2ID8->at<float>(3, 3) = 1;

	this->T_Lens2CameraCenter->at<float>(0, 0) = 1;
	this->T_Lens2CameraCenter->at<float>(0, 1) = 0;
	this->T_Lens2CameraCenter->at<float>(0, 2) = 0;
	this->T_Lens2CameraCenter->at<float>(0, 3) = 0;

	this->T_Lens2CameraCenter->at<float>(1, 0) = 0;
	this->T_Lens2CameraCenter->at<float>(1, 1) = 1;
	this->T_Lens2CameraCenter->at<float>(1, 2) = 0;
	this->T_Lens2CameraCenter->at<float>(1, 3) = 17.5 + 67;

	this->T_Lens2CameraCenter->at<float>(2, 0) = 0;
	this->T_Lens2CameraCenter->at<float>(2, 1) = 0;
	this->T_Lens2CameraCenter->at<float>(2, 2) = 1;
	this->T_Lens2CameraCenter->at<float>(2, 3) = 0;

	this->T_Lens2CameraCenter->at<float>(3, 0) = 0;
	this->T_Lens2CameraCenter->at<float>(3, 1) = 0;
	this->T_Lens2CameraCenter->at<float>(3, 2) = 0;
	this->T_Lens2CameraCenter->at<float>(3, 3) = 1;

	*this->T_CamCoord2RobotCoord = (*R_ID9) * (*T_DisID8_ID9) * (*R_ID8) * (*T_DisCamera2ID8) * (*T_Lens2CameraCenter);
}

void HeadandLifting::HeadCoordinateKJ(float X_kinect, float Y_kinect, float depth, float &X_real, float &Y_real, float &Z_real)
{
	float J1, J2, J3, Kx, Ky;

	J1 = this->GetMotor_PresentAngle(FIRST_HEAD_MOTOR_ID);
	J2 = this->GetMotor_PresentAngle(FIRST_HEAD_MOTOR_ID + 1);
	J3 = 0;

	J1 = J1 * M_PI / 180.0;
	J2 = J2 * M_PI / 180.0;
	J3 = J3 * M_PI / 180.0;

	Kx = X_kinect * 1000;
	Ky = Y_kinect * 1000;
	depth = depth * 1000;

	if (depth > -1000 && depth < 6000)
	{
		this->Get_T_CamCoord2RobotCoord();

		cv::Mat *Pt_Camera_Coordinate = new cv::Mat(4, 1, CV_32FC1, cv::Scalar::all(0));
		Pt_Camera_Coordinate->at<float>(0, 0) = Kx;
		Pt_Camera_Coordinate->at<float>(1, 0) = Ky;
		Pt_Camera_Coordinate->at<float>(2, 0) = depth;
		Pt_Camera_Coordinate->at<float>(3, 0) = 1;

		cv::Mat *Pt_Robot_Coordinate = new cv::Mat(4, 1, CV_32FC1, cv::Scalar::all(0));

		*Pt_Robot_Coordinate = *this->T_CamCoord2RobotCoord * (*Pt_Camera_Coordinate);

		X_real = Pt_Robot_Coordinate->at<float>(0, 0);
		Y_real = Pt_Robot_Coordinate->at<float>(1, 0);
		Z_real = Pt_Robot_Coordinate->at<float>(2, 0);
	}
	else
	{
		X_real = -1;
		Y_real = -1;
		Z_real = -1;
	}
}

void HeadandLifting::GetObjectRotate_JU()
{
	float J1 = this->GetMotor_PresentAngle(FIRST_HEAD_MOTOR_ID);

	J1 = J1 * M_PI / 180.0;
	rotate_angle = M_PI / 2 + J1;
}
