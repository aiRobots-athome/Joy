#include "Arm.h"

Arm::Arm(const vector<unsigned char> &IDArray,
		 const vector<string> &MotorModelArrayList,
		 vector<unsigned char> &AllPortNumber)
	: MotorUnion(IDArray, MotorModelArrayList, AllPortNumber),
	  FIRST_SHOULDER_ID(0),
	  FIRST_HAND_ID(1),
	  HAND_AMOUNT(6),
	  FIRST_FINGER_ID(FIRST_HAND_ID + 6)
{
	VitualGrippingJaw_Enable = false;
	ArmForward = new cv::Mat(4, 4, CV_32FC1);

	Motor_Distance6_ini = 187.98;
	alength6_ini = 200;
	round_value = 10;
}

void Arm::InitArmMotor()
{
	SetMotor_Velocity(FIRST_SHOULDER_ID, 1000); // Pro200
	SetMotor_Velocity(FIRST_HAND_ID, 1000);		// Pro200
	SetMotor_Velocity(FIRST_HAND_ID + 1, 1000); // Pro200
	SetMotor_Velocity(FIRST_HAND_ID + 2, 1500); // Pro20
	SetMotor_Velocity(FIRST_HAND_ID + 3, 1500); // Pro20
	SetMotor_Velocity(FIRST_HAND_ID + 4, 500);  // Mx106
	SetMotor_Velocity(FIRST_HAND_ID + 5, 1500); // Pro20
	SetMotor_Velocity(FIRST_HAND_ID + 6, 500);  // Mx106
}

void Arm::ResetAllMotorAngle()
{
	SetAllMotorsTorqueEnable(true);
	InitArmMotor();
}
void Arm::SetSixLength(float length)
{
	Motor_Distance_6 = length;
}
void Arm::GraspObj(int angle)
{
	SetMotor_Angle(FIRST_FINGER_ID, angle);
}
void Arm::ReleaseObj()
{
	SetMotor_Angle(FIRST_FINGER_ID, -90);
}
void Arm::Stop()
{
	SetAllMotorsTorqueEnable(false);
}
// CORE FUNCTION for 6 DOFs
void Arm::GotoPosition(cv::Mat *&T)
{
	float *temp = Arm_InverseKinematics(T);
	if (ArmMotionEnable)
	{
		SetMotor_Angle(FIRST_SHOULDER_ID, GetMotor_PresentAngle(FIRST_SHOULDER_ID));
		SetMotor_Angle(FIRST_HAND_ID, temp[0]);
		SetMotor_Angle(FIRST_HAND_ID + 1, temp[1]);
		SetMotor_Angle(FIRST_HAND_ID + 2, temp[2]);
		SetMotor_Angle(FIRST_HAND_ID + 3, temp[3]);
		SetMotor_Angle(FIRST_HAND_ID + 4, temp[4]);
		SetMotor_Angle(FIRST_HAND_ID + 5, temp[5]);
		SetMotor_Angle(FIRST_FINGER_ID, GetMotor_PresentAngle(FIRST_FINGER_ID));
		WaitAllMotorsArrival();
	}
	else
	{
		cout << "Fail to arrive" << endl;
	}
}

// 6 DOFs
void Arm::GotoPosition(const int &ox, const int &oy, const int &oz, const int &x, const int &y, const int &z)
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

// CORE FUNCTION for 7 DOFs
void Arm::GotoPosition(const float &pre_angle, cv::Mat *&T)
{
	float *temp = Arm_InverseKinematics(pre_angle, T);

	// cout << temp[0] << endl
	// 	 << temp[1] << endl
	// 	 << temp[2] << endl
	// 	 << temp[3] << endl
	// 	 << temp[4] << endl
	// 	 << temp[5] << endl
	// 	 << temp[6] << endl;

	if (ArmMotionEnable)
	{

		SetMotor_Angle(FIRST_HAND_ID, temp[0]);
		SetMotor_Angle(FIRST_HAND_ID + 1, temp[1]);
		SetMotor_Angle(FIRST_HAND_ID + 2, temp[2]);
		SetMotor_Angle(FIRST_HAND_ID + 3, temp[3]);
		SetMotor_Angle(FIRST_HAND_ID + 4, temp[4]);
		SetMotor_Angle(FIRST_HAND_ID + 5, temp[5]);
		SetMotor_Angle(FIRST_SHOULDER_ID, temp[6]);
		SetMotor_Angle(FIRST_FINGER_ID, GetMotor_PresentAngle(FIRST_FINGER_ID));
		WaitAllMotorsArrival();
		cout << "Arrive !" << endl;
	}
	else
	{
		cout << "Fail to Arrive" << endl;
	}
}

// 7 DOFs
void Arm::GotoPosition(const float &pre_angle, const int &ox, const int &oy, const int &oz, const int &x, const int &y, const int &z)
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

	GotoPosition(pre_angle, tmp);
	delete tmp;
}

void Arm::VitualGrippingJaw(void)
{
	if (VitualGrippingJaw_Enable)
	{
		Motor_Distance_6 = Motor_Distance6_ini + Vitual_Distance6;
		alength_6 = alength6_ini + Vitual_Distance6;
	}
	else
	{
		Motor_Distance_6 = Motor_Distance6_ini;
		alength_6 = alength6_ini;
	}
}

cv::Mat *Arm::TransRotate(const float &x, const float &y, const float &z)
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

cv::Mat *Arm::Jacobian(float J1, float J2, float J3, float J4, float J5, float J6, cv::Mat *T0)
{
	float a = 0.8;
	float DJ1, DJ2, DJ3, DJ4, DJ5, DJ6;
	float delta_angle[6][3];
	cv::Matx44f T_0, T_1, T_2, T_3, T_4, T_5, T_6;
	cv::Mat *tmpT0 = new cv::Mat(4, 4, CV_32FC1);
	cv::Mat *tmpT1 = new cv::Mat(4, 4, CV_32FC1);
	cv::Mat *tmpT2 = new cv::Mat(4, 4, CV_32FC1);
	cv::Mat *tmpT3 = new cv::Mat(4, 4, CV_32FC1);
	cv::Mat *tmpT4 = new cv::Mat(4, 4, CV_32FC1);
	cv::Mat *tmpT5 = new cv::Mat(4, 4, CV_32FC1);
	cv::Mat *tmpT6 = new cv::Mat(4, 4, CV_32FC1);
	float *angle0;
	float *angle1;
	float *angle2;
	float *angle3;
	float *angle4;
	float *angle5;
	float *angle6;
	DJ1 = pow((float)10, -a) + J1;
	DJ2 = pow((float)10, -a) + J2;
	DJ3 = pow((float)10, -a) + J3;
	DJ4 = pow((float)10, -a) + J4;
	DJ5 = pow((float)10, -a) + J5;
	DJ6 = pow((float)10, -a) + J6;

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			T_0(i, j) = T0->at<float>(i, j);
			tmpT0->at<float>(i, j) = T0->at<float>(i, j);
		}
	}
	cv::Mat *T1 = Calculate_ArmForwardKinematics(DJ1, J2, J3, J4, J5, J6);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			T_1(i, j) = T1->at<float>(i, j);
			tmpT1->at<float>(i, j) = T1->at<float>(i, j);
		}
	}
	cv::Mat *T2 = Calculate_ArmForwardKinematics(J1, DJ2, J3, J4, J5, J6);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			T_2(i, j) = T2->at<float>(i, j);
			tmpT2->at<float>(i, j) = T2->at<float>(i, j);
		}
	}

	cv::Mat *T3 = Calculate_ArmForwardKinematics(J1, J2, DJ3, J4, J5, J6);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			T_3(i, j) = T3->at<float>(i, j);
			tmpT3->at<float>(i, j) = T3->at<float>(i, j);
		}
	}

	cv::Mat *T4 = Calculate_ArmForwardKinematics(J1, J2, J3, DJ4, J5, J6);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			T_4(i, j) = T4->at<float>(i, j);
			tmpT4->at<float>(i, j) = T4->at<float>(i, j);
		}
	}

	cv::Mat *T5 = Calculate_ArmForwardKinematics(J1, J2, J3, J4, DJ5, J6);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			T_5(i, j) = T5->at<float>(i, j);
			tmpT5->at<float>(i, j) = T5->at<float>(i, j);
		}
	}

	cv::Mat *T6 = Calculate_ArmForwardKinematics(J1, J2, J3, J4, J5, DJ6);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			T_6(i, j) = T6->at<float>(i, j);
			tmpT6->at<float>(i, j) = T6->at<float>(i, j);
		}
	}

	int size_J[] = {6, 6};
	cv::Mat *J = new cv::Mat(2, size_J, CV_32FC1);
	J->at<float>(0, 0) = (T_1(0, 3) - T_0(0, 3)) * pow((float)10, a);
	J->at<float>(0, 1) = (T_2(0, 3) - T_0(0, 3)) * pow((float)10, a);
	J->at<float>(0, 2) = (T_3(0, 3) - T_0(0, 3)) * pow((float)10, a);
	J->at<float>(0, 3) = (T_4(0, 3) - T_0(0, 3)) * pow((float)10, a);
	J->at<float>(0, 4) = (T_5(0, 3) - T_0(0, 3)) * pow((float)10, a);
	J->at<float>(0, 5) = (T_6(0, 3) - T_0(0, 3)) * pow((float)10, a);

	J->at<float>(1, 0) = (T_1(1, 3) - T_0(1, 3)) * pow((float)10, a);
	J->at<float>(1, 1) = (T_2(1, 3) - T_0(1, 3)) * pow((float)10, a);
	J->at<float>(1, 2) = (T_3(1, 3) - T_0(1, 3)) * pow((float)10, a);
	J->at<float>(1, 3) = (T_4(1, 3) - T_0(1, 3)) * pow((float)10, a);
	J->at<float>(1, 4) = (T_5(1, 3) - T_0(1, 3)) * pow((float)10, a);
	J->at<float>(1, 5) = (T_6(1, 3) - T_0(1, 3)) * pow((float)10, a);

	J->at<float>(2, 0) = (T_1(2, 3) - T_0(2, 3)) * pow((float)10, a);
	J->at<float>(2, 1) = (T_2(2, 3) - T_0(2, 3)) * pow((float)10, a);
	J->at<float>(2, 2) = (T_3(2, 3) - T_0(2, 3)) * pow((float)10, a);
	J->at<float>(2, 3) = (T_4(2, 3) - T_0(2, 3)) * pow((float)10, a);
	J->at<float>(2, 4) = (T_5(2, 3) - T_0(2, 3)) * pow((float)10, a);
	J->at<float>(2, 5) = (T_6(2, 3) - T_0(2, 3)) * pow((float)10, a);

	angle0 = CalculateRotateAngle(tmpT0);
	angle1 = CalculateRotateAngle(tmpT1);
	angle2 = CalculateRotateAngle(tmpT2);
	angle3 = CalculateRotateAngle(tmpT3);
	angle4 = CalculateRotateAngle(tmpT4);
	angle5 = CalculateRotateAngle(tmpT5);
	angle6 = CalculateRotateAngle(tmpT6);

	for (int j = 0; j < 3; j++)
	{
		delta_angle[0][j] = angle1[j] - angle0[j];
		delta_angle[1][j] = angle2[j] - angle0[j];
		delta_angle[2][j] = angle3[j] - angle0[j];
		delta_angle[3][j] = angle4[j] - angle0[j];
		delta_angle[4][j] = angle5[j] - angle0[j];
		delta_angle[5][j] = angle6[j] - angle0[j];
	}

	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (delta_angle[i][j] > M_PI)
				delta_angle[i][j] = delta_angle[i][j] - 2 * M_PI;
			else if (delta_angle[i][j] < -M_PI)
				delta_angle[i][j] = delta_angle[i][j] + 2 * M_PI;
		}
	}

	J->at<float>(3, 0) = (delta_angle[0][0]) * pow((float)10, a);
	J->at<float>(3, 1) = (delta_angle[1][0]) * pow((float)10, a);
	J->at<float>(3, 2) = (delta_angle[2][0]) * pow((float)10, a);
	J->at<float>(3, 3) = (delta_angle[3][0]) * pow((float)10, a);
	J->at<float>(3, 4) = (delta_angle[4][0]) * pow((float)10, a);
	J->at<float>(3, 5) = (delta_angle[5][0]) * pow((float)10, a);

	J->at<float>(4, 0) = (delta_angle[0][1]) * pow((float)10, a);
	J->at<float>(4, 1) = (delta_angle[1][1]) * pow((float)10, a);
	J->at<float>(4, 2) = (delta_angle[2][1]) * pow((float)10, a);
	J->at<float>(4, 3) = (delta_angle[3][1]) * pow((float)10, a);
	J->at<float>(4, 4) = (delta_angle[4][1]) * pow((float)10, a);
	J->at<float>(4, 5) = (delta_angle[5][1]) * pow((float)10, a);

	J->at<float>(5, 0) = (delta_angle[0][2]) * pow((float)10, a);
	J->at<float>(5, 1) = (delta_angle[1][2]) * pow((float)10, a);
	J->at<float>(5, 2) = (delta_angle[2][2]) * pow((float)10, a);
	J->at<float>(5, 3) = (delta_angle[3][2]) * pow((float)10, a);
	J->at<float>(5, 4) = (delta_angle[4][2]) * pow((float)10, a);
	J->at<float>(5, 5) = (delta_angle[5][2]) * pow((float)10, a);

	return J;
}
float *Arm::CalculateRotateAngle(cv::Mat *T)
{

	float nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz;
	float x_angle, y_angle, z_angle;
	float *angle = new float[3];
	nx = T->at<float>(0, 0);
	ny = T->at<float>(1, 0);
	nz = T->at<float>(2, 0);
	ox = T->at<float>(0, 1);
	oy = T->at<float>(1, 1);
	oz = T->at<float>(2, 1);
	ax = T->at<float>(0, 2);
	ay = T->at<float>(1, 2);
	az = T->at<float>(2, 2);
	z_angle = atan2(ny, nx);
	if (z_angle < -M_PI)
		z_angle = z_angle + 2 * M_PI;
	else if (z_angle > M_PI)
		z_angle = z_angle - 2 * M_PI;
	y_angle = atan2(-nz, nx * cos(z_angle) + ny * sin(z_angle));
	if (y_angle < -M_PI)
		y_angle = y_angle + 2 * M_PI;
	else if (y_angle > M_PI)
		y_angle = y_angle - 2 * M_PI;
	x_angle = atan2(ax * sin(z_angle) - ay * cos(z_angle), oy * cos(z_angle) - ox * sin(z_angle));
	if (x_angle < -M_PI)
		x_angle = x_angle + 2 * M_PI;
	else if (x_angle > M_PI)
		x_angle = x_angle - 2 * M_PI;

	angle[0] = x_angle;
	angle[1] = y_angle;
	angle[2] = z_angle;
	return angle;
}