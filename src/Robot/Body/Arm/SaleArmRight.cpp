#include "SaleArmRight.h"

SaleArmRight::SaleArmRight(const vector<unsigned char>& IDArray,
						   const vector<string>& MotorModelArrayList,
						   vector<unsigned char>& AllPortNumber)
:Arm(IDArray, MotorModelArrayList, AllPortNumber)
{
	// Arm Member
	alength_6 = 170;
	Motor_Distance_3 = -250;
	Motor_Distance_5 = 250;

	F_SholderCenter = 1985;
	B_SholderCenter = 2027;

	dis_CenterZ_To_ShoulderY_Ori = 170;

	InitArmMotor();
}

cv::Mat *SaleArmRight::GetKinematics(void)
{
	float fRadian1 = GetMotor_PresentAngle(FIRST_HAND_ID) * Angle2Rad; 
	float fRadian2 = GetMotor_PresentAngle(FIRST_HAND_ID + 1) * Angle2Rad;
	float fRadian3 = GetMotor_PresentAngle(FIRST_HAND_ID + 2) * Angle2Rad;
	float fRadian4 = GetMotor_PresentAngle(FIRST_HAND_ID + 3) * Angle2Rad;
	float fRadian5 = GetMotor_PresentAngle(FIRST_HAND_ID + 4) * Angle2Rad;
	float fRadian6 = GetMotor_PresentAngle(FIRST_HAND_ID + 5) * Angle2Rad;

	return Calculate_ArmForwardKinematics(fRadian1, fRadian2, fRadian3, fRadian4, fRadian5, fRadian6);
}
cv::Mat *SaleArmRight::Calculate_ArmForwardKinematics(float J1, float J2, float J3, float J4, float J5, float J6)
{
	float b = 120;
	double shoulder_angle = GetMotor_PresentAngle(FIRST_SHOULDER_ID);

	cv::Matx44f Ref2Shoulder(
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, -b,
		0, 0, 0, 1);

	cv::Matx44f Shoulder_Turn(
		cos(-shoulder_angle * 3.14 / 180), 0, sin(-shoulder_angle * 3.14 / 180), 0,
		0, 1, 0, 0,
		-sin(-shoulder_angle * 3.14 / 180), 0, cos(-shoulder_angle * 3.14 / 180), 0,
		0, 0, 0, 1);

	cv::Matx44f Shoulder2Base(
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, b,
		0, 0, 0, 1);

	cv::Matx44f Tz_L1(
		cos(J1), 0, sin(J1), 0,
		sin(J1), 0, -cos(J1), 0,
		0, 1, 0, 0,
		0, 0, 0, 1);
	cv::Matx44f Tz_L2(cos(J2), 0, -sin(J2), 0,
					  sin(J2), 0, cos(J2), 0,
					  0, -1, 0, 0,
					  0, 0, 0, 1);
	cv::Matx44f Tz_L3(cos(J3), 0, sin(J3), 0,
					  sin(J3), 0, -cos(J3), 0,
					  0, 1, 0, Motor_Distance_3,
					  0, 0, 0, 1);
	cv::Matx44f Tz_L4(cos(J4), 0, sin(J4), 0,
					  sin(J4), 0, -cos(J4), 0,
					  0, 1, 0, 0,
					  0, 0, 0, 1);
	cv::Matx44f Tz_L5(cos(J5), 0, sin(J5), 0,
					  sin(J5), 0, -cos(J5), 0,
					  0, 1, 0, Motor_Distance_5,
					  0, 0, 0, 1);
	cv::Matx44f Tz_L6(cos(J6), -sin(J6), 0, alength_6 * cos(J6),
					  sin(J6), cos(J6), 0, alength_6 * sin(J6),
					  0, 0, 1, 0,
					  0, 0, 0, 1);
	cv::Matx44f Temp(Ref2Shoulder * Shoulder_Turn * Shoulder2Base * Tz_L1 * Tz_L2 * Tz_L3 * Tz_L4 * Tz_L5 * Tz_L6); // RefT5

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			ArmForward->at<float>(i, j) = Temp(i, j);
		}
	}
	return ArmForward;
}
cv::Mat *SaleArmRight::Calculate_ArmForwardKinematics(float pre_angle, float J1, float J2, float J3, float J4, float J5, float J6)
{
	float b = 120;
	double shoulder_angle = pre_angle;

	cv::Matx44f Ref2Shoulder(
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, -b,
		0, 0, 0, 1);

	cv::Matx44f Shoulder_Turn(
		cos(-shoulder_angle * 3.14 / 180), 0, sin(-shoulder_angle * 3.14 / 180), 0,
		0, 1, 0, 0,
		-sin(-shoulder_angle * 3.14 / 180), 0, cos(-shoulder_angle * 3.14 / 180), 0,
		0, 0, 0, 1);

	cv::Matx44f Shoulder2Base(
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, b,
		0, 0, 0, 1);

	cv::Matx44f Tz_L1(
		cos(J1), 0, sin(J1), 0,
		sin(J1), 0, -cos(J1), 0,
		0, 1, 0, 0,
		0, 0, 0, 1);
	cv::Matx44f Tz_L2(cos(J2), 0, -sin(J2), 0,
					  sin(J2), 0, cos(J2), 0,
					  0, -1, 0, 0,
					  0, 0, 0, 1);
	cv::Matx44f Tz_L3(cos(J3), 0, sin(J3), 0,
					  sin(J3), 0, -cos(J3), 0,
					  0, 1, 0, Motor_Distance_3,
					  0, 0, 0, 1);
	cv::Matx44f Tz_L4(cos(J4), 0, sin(J4), 0,
					  sin(J4), 0, -cos(J4), 0,
					  0, 1, 0, 0,
					  0, 0, 0, 1);
	cv::Matx44f Tz_L5(cos(J5), 0, sin(J5), 0,
					  sin(J5), 0, -cos(J5), 0,
					  0, 1, 0, Motor_Distance_5,
					  0, 0, 0, 1);
	cv::Matx44f Tz_L6(cos(J6), -sin(J6), 0, alength_6 * cos(J6),
					  sin(J6), cos(J6), 0, alength_6 * sin(J6),
					  0, 0, 1, 0,
					  0, 0, 0, 1);
	cv::Matx44f Temp(Ref2Shoulder * Shoulder_Turn * Shoulder2Base * Tz_L1 * Tz_L2 * Tz_L3 * Tz_L4 * Tz_L5 * Tz_L6); // RefT5

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			ArmForward->at<float>(i, j) = Temp(i, j);
		}
	}
	return ArmForward;
}
// 6 DOFs InverseKinematics
float *SaleArmRight::Arm_InverseKinematics(cv::Mat* &T)
{
	cv::Mat *T_OriArm2Shoulder = new cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));
	cv::Mat *T_ShoulderTurn = new cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));
	cv::Mat *T_Shoulder2Arm = new cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));
	float b = 120;
	double shoulder_angle = GetMotor_PresentAngle(FIRST_SHOULDER_ID);

	float nx_T, ny_T, nz_T, ox_T, oy_T, oz_T, ax_T, ay_T, az_T, px_T, py_T, pz_T;
	nx_T = T->at<float>(0, 0);
	ny_T = T->at<float>(1, 0);
	nz_T = T->at<float>(2, 0);
	ox_T = T->at<float>(0, 1);
	oy_T = T->at<float>(1, 1);
	oz_T = T->at<float>(2, 1);
	ax_T = T->at<float>(0, 2);
	ay_T = T->at<float>(1, 2);
	az_T = T->at<float>(2, 2);
	px_T = T->at<float>(0, 3);
	py_T = T->at<float>(1, 3);
	pz_T = T->at<float>(2, 3);
	//==========================================
	T_OriArm2Shoulder->at<float>(0, 0) = 1;
	T_OriArm2Shoulder->at<float>(0, 1) = 0;
	T_OriArm2Shoulder->at<float>(0, 2) = 0;
	T_OriArm2Shoulder->at<float>(0, 3) = 0;

	T_OriArm2Shoulder->at<float>(1, 0) = 0;
	T_OriArm2Shoulder->at<float>(1, 1) = 1;
	T_OriArm2Shoulder->at<float>(1, 2) = 0;
	T_OriArm2Shoulder->at<float>(1, 3) = 0;

	T_OriArm2Shoulder->at<float>(2, 0) = 0;
	T_OriArm2Shoulder->at<float>(2, 1) = 0;
	T_OriArm2Shoulder->at<float>(2, 2) = 1;
	T_OriArm2Shoulder->at<float>(2, 3) = -b;

	T_OriArm2Shoulder->at<float>(3, 0) = 0;
	T_OriArm2Shoulder->at<float>(3, 1) = 0;
	T_OriArm2Shoulder->at<float>(3, 2) = 0;
	T_OriArm2Shoulder->at<float>(3, 3) = 1;

	//===========================================
	T_ShoulderTurn->at<float>(0, 0) = cos(shoulder_angle * 3.14 / 180); //shoulder_angle�S���["-"�O�]���t�t�o��
	T_ShoulderTurn->at<float>(0, 1) = 0;
	T_ShoulderTurn->at<float>(0, 2) = -sin(shoulder_angle * 3.14 / 180);
	T_ShoulderTurn->at<float>(0, 3) = 0;

	T_ShoulderTurn->at<float>(1, 0) = 0;
	T_ShoulderTurn->at<float>(1, 1) = 1;
	T_ShoulderTurn->at<float>(1, 2) = 0;
	T_ShoulderTurn->at<float>(1, 3) = 0;

	T_ShoulderTurn->at<float>(2, 0) = sin(shoulder_angle * 3.14 / 180);
	T_ShoulderTurn->at<float>(2, 1) = 0;
	T_ShoulderTurn->at<float>(2, 2) = cos(shoulder_angle * 3.14 / 180);
	T_ShoulderTurn->at<float>(2, 3) = 0;

	T_ShoulderTurn->at<float>(3, 0) = 0;
	T_ShoulderTurn->at<float>(3, 1) = 0;
	T_ShoulderTurn->at<float>(3, 2) = 0;
	T_ShoulderTurn->at<float>(3, 3) = 1;

	//===========================================
	T_Shoulder2Arm->at<float>(0, 0) = 1;
	T_Shoulder2Arm->at<float>(0, 1) = 0;
	T_Shoulder2Arm->at<float>(0, 2) = 0;
	T_Shoulder2Arm->at<float>(0, 3) = 0;

	T_Shoulder2Arm->at<float>(1, 0) = 0;
	T_Shoulder2Arm->at<float>(1, 1) = 1;
	T_Shoulder2Arm->at<float>(1, 2) = 0;
	T_Shoulder2Arm->at<float>(1, 3) = 0;

	T_Shoulder2Arm->at<float>(2, 0) = 0;
	T_Shoulder2Arm->at<float>(2, 1) = 0;
	T_Shoulder2Arm->at<float>(2, 2) = 1;
	T_Shoulder2Arm->at<float>(2, 3) = b;

	T_Shoulder2Arm->at<float>(3, 0) = 0;
	T_Shoulder2Arm->at<float>(3, 1) = 0;
	T_Shoulder2Arm->at<float>(3, 2) = 0;
	T_Shoulder2Arm->at<float>(3, 3) = 1;

	////////////////////////////////////////////////////////////First ver.
	ArmMotionEnable = false;

	float J1, J2, J3, J4, J5, J6;

	float delta_Pos = 4096 * 1 + 151875 * 3 + 250950 * 2;

	float theta1[4];
	float theta2[4];
	float theta3[4];
	float theta4[4];
	float theta5[4];
	float theta6;
	float nx1, ny1, nz1, ox1, oy1, oz1, ax1, ay1, az1, px1, py1, pz1;
	float nx_test, ny_test, nz_test, ox_test, oy_test, oz_test, ax_test, ay_test, az_test, px_test, py_test, pz_test;
	float nx_goal, ny_goal, nz_goal, ox_goal, oy_goal, oz_goal, ax_goal, ay_goal, az_goal, px_goal, py_goal, pz_goal;

	int delta_Ang = 180 * 6;

	//cv::Mat T1 = T->inv(); //ori
	//   cv::Mat T_test = *T_Shoulder2Arm*(*T_ShoulderTurn)*(*T_OriArm2Shoulder)*T->inv();
	//cv::Mat T_test = T->inv(); //ori
	//cv::Mat T_test = *T; //ori

	//cv::Mat T_goal = (T_Shoulder2Arm->inv())*(T_ShoulderTurn->inv())*(T_OriArm2Shoulder->inv())*(*T);
	cv::Mat T_goal = (T_Shoulder2Arm->inv()) * (T_ShoulderTurn->inv()) * (T_OriArm2Shoulder->inv()) * (*T);
	/*T_test.at<float>(0, 3) = round(T_goal.at<float>(0, 3));
	T_test.at<float>(1, 3) = round(T_goal.at<float>(1, 3));
	T_test.at<float>(2, 3) = round(T_goal.at<float>(2, 3));*/
	/*T_goal.at<float>(0, 3) = round(T_goal.at<float>(0, 3));
	T_goal.at<float>(1, 3) = round(T_goal.at<float>(1, 3));
	T_goal.at<float>(2, 3) = round(T_goal.at<float>(2, 3));*/
	cv::Mat T1 = T_goal.inv();
	//cv::Mat T1 = T_test.inv();

	//T1 = T_test;

	//cv::Mat T_test = T_test->inv();
	//cv::Mat T_test = *T_Shoulder2Arm*(*T_ShoulderTurn)*(*T_OriArm2Shoulder)*T->inv();

	//initial------------------------------------------------------------//
	nx_T = T->at<float>(0, 0);
	ny_T = T->at<float>(1, 0);
	nz_T = T->at<float>(2, 0);
	ox_T = T->at<float>(0, 1);
	oy_T = T->at<float>(1, 1);
	oz_T = T->at<float>(2, 1);
	ax_T = T->at<float>(0, 2);
	ay_T = T->at<float>(1, 2);
	az_T = T->at<float>(2, 2);
	px_T = T->at<float>(0, 3);
	py_T = T->at<float>(1, 3);
	pz_T = T->at<float>(2, 3);

	//--------------------goal T
	nx_goal = T_goal.at<float>(0, 0);
	ny_goal = T_goal.at<float>(1, 0);
	nz_goal = T_goal.at<float>(2, 0);
	ox_goal = T_goal.at<float>(0, 1);
	oy_goal = T_goal.at<float>(1, 1);
	oz_goal = T_goal.at<float>(2, 1);
	ax_goal = T_goal.at<float>(0, 2);
	ay_goal = T_goal.at<float>(1, 2);
	az_goal = T_goal.at<float>(2, 2);
	px_goal = T_goal.at<float>(0, 3);
	py_goal = T_goal.at<float>(1, 3);
	pz_goal = T_goal.at<float>(2, 3);
	//T = Calculate_ArmForwardKinematics(0* Angle2Rad, 10 * Angle2Rad, 90 * Angle2Rad, 0 * Angle2Rad, 70 * Angle2Rad, 0 * Angle2Rad);

	//-----------------------------------------------
	nx1 = T1.at<float>(0, 0);
	ny1 = T1.at<float>(1, 0);
	nz1 = T1.at<float>(2, 0);
	ox1 = T1.at<float>(0, 1);
	oy1 = T1.at<float>(1, 1);
	oz1 = T1.at<float>(2, 1);
	ax1 = T1.at<float>(0, 2);
	ay1 = T1.at<float>(1, 2);
	az1 = T1.at<float>(2, 2);
	px1 = T1.at<float>(0, 3);
	py1 = T1.at<float>(1, 3);
	pz1 = T1.at<float>(2, 3);

	for (int i = 0; i < 4; i++)
	{
		theta1[i] = -999;
		theta2[i] = -999;
		theta3[i] = -999;
		theta4[i] = -999;
		theta5[i] = -999;
		theta6 = -999;
	}

	float *Solution = new float[6];
	for (int i = 0; i < 6; i++)
	{
		Solution[i] = 0;
	}
	//--------------------------------------------------------------------//
	{
		//k = a6 ^ 2 + 2 * a6*px1 + px1 ^ 2 + py1 ^ 2 + pz1 ^ 2 - d3 ^ 2 - d5 ^ 2;

		float k = pow(alength_6, 2) + 2 * alength_6 * px1 + pow(px1, 2) + pow(py1, 2) + pow(pz1, 2) - pow(Motor_Distance_3, 2) - pow(Motor_Distance_5, 2);

		float a = -2 * Motor_Distance_3 * Motor_Distance_5;
		float tc = (k / a);

		if (tc > 1)
			tc = 1;
		else if (tc < -1)
			tc = -1;

		float ts = pow((1 - pow(tc, 2)), 0.5);
		if (ts > 1)
			ts = 1;
		else if (ts < -1)
			ts = -1;

		theta4[0] = atan2(ts, -tc) / Angle2Rad;
		theta4[1] = atan2(ts, tc) / Angle2Rad;
		theta4[2] = -(atan2(ts, -tc) / Angle2Rad);
		theta4[3] = -(atan2(ts, tc) / Angle2Rad);

		//System::Console::WriteLine(k);
		//System::Console::WriteLine(a);
		//System::Console::WriteLine(b);
		//System::Console::WriteLine(ts);
		//System::Console::WriteLine(tc);
		//System::Console::WriteLine(theta3[0]*180/M_PI);
		//System::Console::WriteLine(theta3[1]*180/M_PI);
	}
	for (int i = 0; i < 4; i++)
	{
		if (theta4[i] > 180)
		{
			theta4[i] = theta4[i] - 360;
		}
		if (theta4[i] < -180)
		{
			theta4[i] = theta4[i] + 360;
		}

		if (theta4[i] <= -120 || theta4[i] >= 120) //�_���I
		{
			theta4[i] = -999;
		}
	}

	for (int i = 0; i < 4; i++)
	{
		if (theta4[i] != -999)
		{
			J4 = theta4[i] * Angle2Rad;
			//J5-----------------------------------------------------------------------------
			//Matlab
			//ts= (pz1)/(-d3*sin(J4));
			float ts = (pz1 / (-Motor_Distance_3 * sin(J4)));
			if (ts > 1)
				ts = 1;
			else if (ts < -1)
				ts = -1;
			else if (abs(ts) < 0.001)
				ts = 0;
			float tc = pow((1 - pow(ts, 2)), 0.5);
			if (tc > 1)
				tc = 1;
			else if (tc < -1)
				tc = -1;
			theta5[0] = (atan2(ts, tc)) / Angle2Rad;
			theta5[1] = (atan2(ts, -tc)) / Angle2Rad;
			theta5[2] = (-atan2(ts, -tc)) / Angle2Rad;
			theta5[3] = (-atan2(ts, tc)) / Angle2Rad;
		}
		//���ץ��W��
		for (int i = 0; i < 4; i++)
		{

			if (theta5[i] > 180)
			{
				theta5[i] = theta5[i] - 360;
			}
			if (theta5[i] < -180)
			{
				theta5[i] = theta5[i] + 360;
			}

			//if (( theta5[i]<-25 || theta5[i] > 120 ))//�_���I
			//{
			//	theta5[i] = -999;
			//}
		}

		//-------------------------------------------------------------------------------
		for (int j = 0; j < 4; j++)
		{
			if (theta5[j] != -999)

				J5 = theta5[j] * Angle2Rad;
			{
				//J6-----------------------------------------------------------------------
				//phi = atan2( py1, ( a6 + px1 ) );
				//IK_J6 = ( atan2( ( d3*cos( J4 ) - d5 ), ( -d3*cos( J5 )*sin( J4 ) ) ) - phi ) * 180 / pi;
				float phi = atan2(py1, (px1 + alength_6));

				theta6 = (atan2((Motor_Distance_3 * cos(J4) - Motor_Distance_5), (-Motor_Distance_3 * cos(J5) * sin(J4))) - phi) / Angle2Rad;
			}
			//���ץ��W��

			if (theta6 > 180)
			{
				theta6 = theta6 - 360;
			}
			if (theta6 < -180)
			{
				theta6 = theta6 + 360;
			}
			if (theta6 <= 0 || theta6 > 180)
			{
				theta6 = -999;
			}

			//����F����

			//-------------------------------------------------------------------------------

			if (theta6 != -999)
			{

				J6 = theta6 * Angle2Rad;
				{
					//J2-------------------------------------------------------------------
					//	  tc = az1*sin(J4)*sin(J5) - ay1*(cos(J4)*cos(J6) + cos(J5)*sin(J4)*sin(J6)) - ax1*(cos(J4)*sin(J6) - cos(J5)*cos(J6)*sin(J4));
					float tc = az1 * sin(J4) * sin(J5) - ay1 * (cos(J4) * cos(J6) + cos(J5) * sin(J4) * sin(J6)) - ax1 * (cos(J4) * sin(J6) - cos(J5) * cos(J6) * sin(J4));
					if (tc > 1)
						tc = 1;
					else if (tc < -1)
						tc = -1;
					float ts = pow((1 - pow(tc, 2)), 0.5);
					if (ts > 1)
						ts = 1;
					else if (ts < -1)
						ts = -1;
					theta2[0] = (atan2(ts, -tc)) / Angle2Rad;
					theta2[1] = (atan2(ts, tc)) / Angle2Rad;
					theta2[2] = (-atan2(ts, tc)) / Angle2Rad;
					theta2[3] = (-atan2(ts, -tc)) / Angle2Rad;
				}
				//���ץ��W��
				for (int i = 0; i < 4; i++)
				{

					if (theta2[i] != -999)
					{
						if (theta2[i] > 180)
						{
							theta2[i] = theta2[i] - 360;
						}
						if (theta2[i] < -180)
						{
							theta2[i] = theta2[i] + 360;
						}

						if ((theta2[i] <= 65 || theta2[i] >= 180)) //�_���I
						{
							theta2[i] = -999;
						}
					}
				}

				//-------------------------------------------------------------------------------
				for (int l = 0; l < 4; l++)
				{
					if (theta2[l] != -999)
					{

						J2 = theta2[l] * Angle2Rad;
						{
							//J3--------------------------------------------------------------
							// tc = (ax1*(sin(J4)*sin(J6) + cos(J4)*cos(J5)*cos(J6)) + ay1*(cos(J6)*sin(J4) - cos(J4)*cos(J5)*sin(J6)) + az1*cos(J4)*sin(J5))/sin(J2);
							float tc = (ax1 * (sin(J4) * sin(J6) + cos(J4) * cos(J5) * cos(J6)) + ay1 * (cos(J6) * sin(J4) - cos(J4) * cos(J5) * sin(J6)) + az1 * cos(J4) * sin(J5)) / sin(J2);
							if (tc > 1)
								tc = 1;
							else if (tc < -1)
								tc = -1;
							float ts = pow((1 - pow(tc, 2)), 0.5);
							if (ts > 1)
								ts = 1;
							else if (ts < -1)
								ts = -1;
							theta3[0] = (atan2(ts, -tc)) / Angle2Rad;
							theta3[1] = (atan2(ts, tc)) / Angle2Rad;
							theta3[2] = (-atan2(ts, tc)) / Angle2Rad;
							theta3[3] = (-atan2(ts, -tc)) / Angle2Rad;
						}
						//���ץ��W��
						for (int i = 0; i < 4; i++)
						{
							if (theta3[i] != -999)
							{
								if (theta3[i] > 180)
								{
									theta3[i] = theta3[i] - 360;
								}
								if (theta3[i] < -180)
								{
									theta3[i] = theta3[i] + 360;
								}
							}
						}

						//-------------------------------------------------------------------------------
						for (int m = 0; m < 4; m++)
						{
							if (theta3[m] != -999)
							{

								J3 = theta3[m] * Angle2Rad;
								{
									//J1----------------------------------------------------------------------
									//tc= (nz1*sin(J4)*sin(J5) - ny1*(cos(J4)*cos(J6) + cos(J5)*sin(J4)*sin(J6)) - nx1*(cos(J4)*sin(J6) - cos(J5)*cos(J6)*sin(J4)))/(-sin(J2));

									float tc = (nz1 * sin(J4) * sin(J5) - ny1 * (cos(J4) * cos(J6) + cos(J5) * sin(J4) * sin(J6)) - nx1 * (cos(J4) * sin(J6) - cos(J5) * cos(J6) * sin(J4))) / (-sin(J2));
									if (tc > 1)
										tc = 1;
									else if (tc < -1)
										tc = -1;
									float ts = pow((1 - pow(tc, 2)), 0.5);
									if (ts > 1)
										ts = 1;
									else if (ts < -1)
										ts = -1;
									theta1[0] = (atan2(ts, -tc)) / Angle2Rad;
									theta1[1] = (atan2(ts, tc)) / Angle2Rad;
									theta1[2] = (-atan2(ts, tc)) / Angle2Rad;
									theta1[3] = (-atan2(ts, -tc)) / Angle2Rad;
								}
								//���ץ��W��
								for (int i = 0; i < 4; i++)
								{
									if (theta1[i] != -999)
									{
										if (theta1[i] > 180)
										{
											theta1[i] = theta1[i] - 360;
										}
										if (theta1[i] < -180)
										{
											theta1[i] = theta1[i] + 360;
										}
									}
								}

								//-------------------------------------------------------------------------------
								for (int n = 0; n < 4; n++)
								{
									if (theta1[n] != -999)
									{

										J1 = theta1[n] * Angle2Rad;
										//���i�઺��-------------------------------------------b------------

										bool check = true;
										//�����X�Ӫ��ѻP�쥻���O�_���P
										float j1, j2, j3, j4, j5, j6;
										j1 = J1 * Rad2Angle;
										j2 = J2 * Rad2Angle;
										j3 = J3 * Rad2Angle;
										j4 = J4 * Rad2Angle;
										j5 = J5 * Rad2Angle;
										j6 = J6 * Rad2Angle;
										cv::Mat *tmpT = Calculate_ArmForwardKinematics(J1, J2, J3, J4, J5, J6);

										for (int o = 0; o < 4; o++)
										{
											for (int p = 0; p < 4; p++)
											{

												if (round(round_value * tmpT->at<float>(o, p)) != round(round_value * T->at<float>(o, p)))
												{

													check = false;
												}
											}
											//cout << endl;
										}
										if (check)
										{

											float temp_theta[6], re_theta[6];
											float tmp1 = J1 * Rad2Angle;
											float tmp2 = J2 * Rad2Angle;
											float tmp3 = J3 * Rad2Angle;
											float tmp4 = J4 * Rad2Angle;
											float tmp5 = J5 * Rad2Angle;
											float tmp6 = J6 * Rad2Angle;

											re_theta[0] = J1 * Rad2Angle;
											re_theta[1] = J2 * Rad2Angle;
											re_theta[2] = J3 * Rad2Angle;
											re_theta[3] = J4 * Rad2Angle;
											re_theta[4] = J5 * Rad2Angle;
											re_theta[5] = J6 * Rad2Angle;

											temp_theta[0] = J1 * Rad2Angle + 90;
											temp_theta[1] = J2 * Rad2Angle - 90;
											temp_theta[2] = J3 * Rad2Angle - 90;
											temp_theta[3] = J4 * Rad2Angle;
											temp_theta[4] = J5 * Rad2Angle + 180;
											temp_theta[5] = J6 * Rad2Angle - 90;

											for (int o = 0; o < 6; o++)
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

											// Motor direction adjustment
											temp_theta[0] = (temp_theta[0]);
											temp_theta[1] = -(temp_theta[1]);
											temp_theta[2] = -(temp_theta[2]);
											temp_theta[3] = (temp_theta[3]);
											temp_theta[4] = (temp_theta[4]);
											temp_theta[5] = -(temp_theta[5]);

											int tmp_delta_Ang =
												abs(temp_theta[0] - GetMotor_PresentAngle(FIRST_HAND_ID))
												+ abs(temp_theta[1] - GetMotor_PresentAngle(FIRST_HAND_ID + 1))
												+ abs(temp_theta[2] - GetMotor_PresentAngle(FIRST_HAND_ID + 2))
												+ abs(temp_theta[3] - GetMotor_PresentAngle(FIRST_HAND_ID + 3))
												+ abs(temp_theta[4] - GetMotor_PresentAngle(FIRST_HAND_ID + 4))
												+ abs(temp_theta[5] - GetMotor_PresentAngle(FIRST_HAND_ID + 5));

											if (tmp_delta_Ang < delta_Ang)
											{
												delta_Ang = tmp_delta_Ang;
												jacobian = Jacobian(J1, J2, J3, J4, J5, J6, tmpT);

												for (int o = 0; o < 6; o++)
												{
													Solution[o] = temp_theta[o];
												}
												ArmMotionEnable = true;
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
	if (Solution[0] == 0 && Solution[1] == 0 && Solution[2] == 0 && Solution[3] == 0 && Solution[4] == 0 && Solution[5] == 0)
	{
		ArmMotionEnable = false;
	}
	if (delta_Ang > 150)
	{
		ArmMotionEnable = false;
	}
	return Solution;
}
// 7 DOFs InverseKinematics
float *SaleArmRight::Arm_InverseKinematics(const float &pre_angle, cv::Mat* &T)
{
	cv::Mat *T_OriArm2Shoulder = new cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));
	cv::Mat *T_ShoulderTurn = new cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));
	cv::Mat *T_Shoulder2Arm = new cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));
	float b = 120;
	double shoulder_angle = pre_angle;

	float nx_T, ny_T, nz_T, ox_T, oy_T, oz_T, ax_T, ay_T, az_T, px_T, py_T, pz_T;
	nx_T = T->at<float>(0, 0);
	ny_T = T->at<float>(1, 0);
	nz_T = T->at<float>(2, 0);
	ox_T = T->at<float>(0, 1);
	oy_T = T->at<float>(1, 1);
	oz_T = T->at<float>(2, 1);
	ax_T = T->at<float>(0, 2);
	ay_T = T->at<float>(1, 2);
	az_T = T->at<float>(2, 2);
	px_T = T->at<float>(0, 3);
	py_T = T->at<float>(1, 3);
	pz_T = T->at<float>(2, 3);
	//==========================================
	T_OriArm2Shoulder->at<float>(0, 0) = 1;
	T_OriArm2Shoulder->at<float>(0, 1) = 0;
	T_OriArm2Shoulder->at<float>(0, 2) = 0;
	T_OriArm2Shoulder->at<float>(0, 3) = 0;

	T_OriArm2Shoulder->at<float>(1, 0) = 0;
	T_OriArm2Shoulder->at<float>(1, 1) = 1;
	T_OriArm2Shoulder->at<float>(1, 2) = 0;
	T_OriArm2Shoulder->at<float>(1, 3) = 0;

	T_OriArm2Shoulder->at<float>(2, 0) = 0;
	T_OriArm2Shoulder->at<float>(2, 1) = 0;
	T_OriArm2Shoulder->at<float>(2, 2) = 1;
	T_OriArm2Shoulder->at<float>(2, 3) = -b;

	T_OriArm2Shoulder->at<float>(3, 0) = 0;
	T_OriArm2Shoulder->at<float>(3, 1) = 0;
	T_OriArm2Shoulder->at<float>(3, 2) = 0;
	T_OriArm2Shoulder->at<float>(3, 3) = 1;

	//===========================================
	T_ShoulderTurn->at<float>(0, 0) = cos(shoulder_angle * 3.14 / 180); //shoulder_angle�S���["-"�O�]���t�t�o��
	T_ShoulderTurn->at<float>(0, 1) = 0;
	T_ShoulderTurn->at<float>(0, 2) = -sin(shoulder_angle * 3.14 / 180);
	T_ShoulderTurn->at<float>(0, 3) = 0;

	T_ShoulderTurn->at<float>(1, 0) = 0;
	T_ShoulderTurn->at<float>(1, 1) = 1;
	T_ShoulderTurn->at<float>(1, 2) = 0;
	T_ShoulderTurn->at<float>(1, 3) = 0;

	T_ShoulderTurn->at<float>(2, 0) = sin(shoulder_angle * 3.14 / 180);
	T_ShoulderTurn->at<float>(2, 1) = 0;
	T_ShoulderTurn->at<float>(2, 2) = cos(shoulder_angle * 3.14 / 180);
	T_ShoulderTurn->at<float>(2, 3) = 0;

	T_ShoulderTurn->at<float>(3, 0) = 0;
	T_ShoulderTurn->at<float>(3, 1) = 0;
	T_ShoulderTurn->at<float>(3, 2) = 0;
	T_ShoulderTurn->at<float>(3, 3) = 1;

	//===========================================
	T_Shoulder2Arm->at<float>(0, 0) = 1;
	T_Shoulder2Arm->at<float>(0, 1) = 0;
	T_Shoulder2Arm->at<float>(0, 2) = 0;
	T_Shoulder2Arm->at<float>(0, 3) = 0;

	T_Shoulder2Arm->at<float>(1, 0) = 0;
	T_Shoulder2Arm->at<float>(1, 1) = 1;
	T_Shoulder2Arm->at<float>(1, 2) = 0;
	T_Shoulder2Arm->at<float>(1, 3) = 0;

	T_Shoulder2Arm->at<float>(2, 0) = 0;
	T_Shoulder2Arm->at<float>(2, 1) = 0;
	T_Shoulder2Arm->at<float>(2, 2) = 1;
	T_Shoulder2Arm->at<float>(2, 3) = b;

	T_Shoulder2Arm->at<float>(3, 0) = 0;
	T_Shoulder2Arm->at<float>(3, 1) = 0;
	T_Shoulder2Arm->at<float>(3, 2) = 0;
	T_Shoulder2Arm->at<float>(3, 3) = 1;

	////////////////////////////////////////////////////////////First ver.
	ArmMotionEnable = false;

	float J1, J2, J3, J4, J5, J6;

	float delta_Pos = 4096 * 1 + 151875 * 3 + 250950 * 2;

	float theta1[4];
	float theta2[4];
	float theta3[4];
	float theta4[4];
	float theta5[4];
	float theta6;
	float nx1, ny1, nz1, ox1, oy1, oz1, ax1, ay1, az1, px1, py1, pz1;
	float nx_test, ny_test, nz_test, ox_test, oy_test, oz_test, ax_test, ay_test, az_test, px_test, py_test, pz_test;
	float nx_goal, ny_goal, nz_goal, ox_goal, oy_goal, oz_goal, ax_goal, ay_goal, az_goal, px_goal, py_goal, pz_goal;

	int delta_Ang = 180 * 6;

	//cv::Mat T1 = T->inv(); //ori
	//   cv::Mat T_test = *T_Shoulder2Arm*(*T_ShoulderTurn)*(*T_OriArm2Shoulder)*T->inv();
	//cv::Mat T_test = T->inv(); //ori
	//cv::Mat T_test = *T; //ori

	//cv::Mat T_goal = (T_Shoulder2Arm->inv())*(T_ShoulderTurn->inv())*(T_OriArm2Shoulder->inv())*(*T);
	cv::Mat T_goal = (T_Shoulder2Arm->inv()) * (T_ShoulderTurn->inv()) * (T_OriArm2Shoulder->inv()) * (*T);
	/*T_test.at<float>(0, 3) = round(T_goal.at<float>(0, 3));
	T_test.at<float>(1, 3) = round(T_goal.at<float>(1, 3));
	T_test.at<float>(2, 3) = round(T_goal.at<float>(2, 3));*/
	/*T_goal.at<float>(0, 3) = round(T_goal.at<float>(0, 3));
	T_goal.at<float>(1, 3) = round(T_goal.at<float>(1, 3));
	T_goal.at<float>(2, 3) = round(T_goal.at<float>(2, 3));*/
	cv::Mat T1 = T_goal.inv();
	//cv::Mat T1 = T_test.inv();

	//T1 = T_test;

	//cv::Mat T_test = T_test->inv();
	//cv::Mat T_test = *T_Shoulder2Arm*(*T_ShoulderTurn)*(*T_OriArm2Shoulder)*T->inv();

	//initial------------------------------------------------------------//
	nx_T = T->at<float>(0, 0);
	ny_T = T->at<float>(1, 0);
	nz_T = T->at<float>(2, 0);
	ox_T = T->at<float>(0, 1);
	oy_T = T->at<float>(1, 1);
	oz_T = T->at<float>(2, 1);
	ax_T = T->at<float>(0, 2);
	ay_T = T->at<float>(1, 2);
	az_T = T->at<float>(2, 2);
	px_T = T->at<float>(0, 3);
	py_T = T->at<float>(1, 3);
	pz_T = T->at<float>(2, 3);

	//--------------------goal T
	nx_goal = T_goal.at<float>(0, 0);
	ny_goal = T_goal.at<float>(1, 0);
	nz_goal = T_goal.at<float>(2, 0);
	ox_goal = T_goal.at<float>(0, 1);
	oy_goal = T_goal.at<float>(1, 1);
	oz_goal = T_goal.at<float>(2, 1);
	ax_goal = T_goal.at<float>(0, 2);
	ay_goal = T_goal.at<float>(1, 2);
	az_goal = T_goal.at<float>(2, 2);
	px_goal = T_goal.at<float>(0, 3);
	py_goal = T_goal.at<float>(1, 3);
	pz_goal = T_goal.at<float>(2, 3);
	//T = Calculate_ArmForwardKinematics(0* Angle2Rad, 10 * Angle2Rad, 90 * Angle2Rad, 0 * Angle2Rad, 70 * Angle2Rad, 0 * Angle2Rad);

	//-----------------------------------------------
	nx1 = T1.at<float>(0, 0);
	ny1 = T1.at<float>(1, 0);
	nz1 = T1.at<float>(2, 0);
	ox1 = T1.at<float>(0, 1);
	oy1 = T1.at<float>(1, 1);
	oz1 = T1.at<float>(2, 1);
	ax1 = T1.at<float>(0, 2);
	ay1 = T1.at<float>(1, 2);
	az1 = T1.at<float>(2, 2);
	px1 = T1.at<float>(0, 3);
	py1 = T1.at<float>(1, 3);
	pz1 = T1.at<float>(2, 3);

	for (int i = 0; i < 4; i++)
	{
		theta1[i] = -999;
		theta2[i] = -999;
		theta3[i] = -999;
		theta4[i] = -999;
		theta5[i] = -999;
		theta6 = -999;
	}

	float *Solution = new float[7];
	for (int i = 0; i < 6; i++)
	{
		Solution[i] = 0;
	}
	//--------------------------------------------------------------------//
	{
		//k = a6 ^ 2 + 2 * a6*px1 + px1 ^ 2 + py1 ^ 2 + pz1 ^ 2 - d3 ^ 2 - d5 ^ 2;

		float k = pow(alength_6, 2) + 2 * alength_6 * px1 + pow(px1, 2) + pow(py1, 2) + pow(pz1, 2) - pow(Motor_Distance_3, 2) - pow(Motor_Distance_5, 2);

		float a = -2 * Motor_Distance_3 * Motor_Distance_5;
		float tc = (k / a);

		if (tc > 1)
			tc = 1;
		else if (tc < -1)
			tc = -1;

		float ts = pow((1 - pow(tc, 2)), 0.5);
		if (ts > 1)
			ts = 1;
		else if (ts < -1)
			ts = -1;

		theta4[0] = atan2(ts, -tc) / Angle2Rad;
		theta4[1] = atan2(ts, tc) / Angle2Rad;
		theta4[2] = -(atan2(ts, -tc) / Angle2Rad);
		theta4[3] = -(atan2(ts, tc) / Angle2Rad);

		//System::Console::WriteLine(k);
		//System::Console::WriteLine(a);
		//System::Console::WriteLine(b);
		//System::Console::WriteLine(ts);
		//System::Console::WriteLine(tc);
		//System::Console::WriteLine(theta3[0]*180/M_PI);
		//System::Console::WriteLine(theta3[1]*180/M_PI);
	}
	for (int i = 0; i < 4; i++)
	{
		if (theta4[i] > 180)
		{
			theta4[i] = theta4[i] - 360;
		}
		if (theta4[i] < -180)
		{
			theta4[i] = theta4[i] + 360;
		}

		if (theta4[i] <= -120 || theta4[i] >= 120) //�_���I
		{
			theta4[i] = -999;
		}
	}

	for (int i = 0; i < 4; i++)
	{
		if (theta4[i] != -999)
		{
			J4 = theta4[i] * Angle2Rad;
			//J5-----------------------------------------------------------------------------
			//Matlab
			//ts= (pz1)/(-d3*sin(J4));
			float ts = (pz1 / (-Motor_Distance_3 * sin(J4)));
			if (ts > 1)
				ts = 1;
			else if (ts < -1)
				ts = -1;
			else if (abs(ts) < 0.001)
				ts = 0;
			float tc = pow((1 - pow(ts, 2)), 0.5);
			if (tc > 1)
				tc = 1;
			else if (tc < -1)
				tc = -1;
			theta5[0] = (atan2(ts, tc)) / Angle2Rad;
			theta5[1] = (atan2(ts, -tc)) / Angle2Rad;
			theta5[2] = (-atan2(ts, -tc)) / Angle2Rad;
			theta5[3] = (-atan2(ts, tc)) / Angle2Rad;
		}
		//���ץ��W��
		for (int i = 0; i < 4; i++)
		{

			if (theta5[i] > 180)
			{
				theta5[i] = theta5[i] - 360;
			}
			if (theta5[i] < -180)
			{
				theta5[i] = theta5[i] + 360;
			}

			//if (( theta5[i]<-25 || theta5[i] > 120 ))//�_���I
			//{
			//	theta5[i] = -999;
			//}
		}

		//-------------------------------------------------------------------------------
		for (int j = 0; j < 4; j++)
		{
			if (theta5[j] != -999)

				J5 = theta5[j] * Angle2Rad;
			{
				//J6-----------------------------------------------------------------------
				//phi = atan2( py1, ( a6 + px1 ) );
				//IK_J6 = ( atan2( ( d3*cos( J4 ) - d5 ), ( -d3*cos( J5 )*sin( J4 ) ) ) - phi ) * 180 / pi;
				float phi = atan2(py1, (px1 + alength_6));

				theta6 = (atan2((Motor_Distance_3 * cos(J4) - Motor_Distance_5), (-Motor_Distance_3 * cos(J5) * sin(J4))) - phi) / Angle2Rad;
			}
			//���ץ��W��

			if (theta6 > 180)
			{
				theta6 = theta6 - 360;
			}
			if (theta6 < -180)
			{
				theta6 = theta6 + 360;
			}
			if (theta6 <= 0 || theta6 > 180)
			{
				theta6 = -999;
			}

			//����F����

			//-------------------------------------------------------------------------------

			if (theta6 != -999)
			{

				J6 = theta6 * Angle2Rad;
				{
					//J2-------------------------------------------------------------------
					//	  tc = az1*sin(J4)*sin(J5) - ay1*(cos(J4)*cos(J6) + cos(J5)*sin(J4)*sin(J6)) - ax1*(cos(J4)*sin(J6) - cos(J5)*cos(J6)*sin(J4));
					float tc = az1 * sin(J4) * sin(J5) - ay1 * (cos(J4) * cos(J6) + cos(J5) * sin(J4) * sin(J6)) - ax1 * (cos(J4) * sin(J6) - cos(J5) * cos(J6) * sin(J4));
					if (tc > 1)
						tc = 1;
					else if (tc < -1)
						tc = -1;
					float ts = pow((1 - pow(tc, 2)), 0.5);
					if (ts > 1)
						ts = 1;
					else if (ts < -1)
						ts = -1;
					theta2[0] = (atan2(ts, -tc)) / Angle2Rad;
					theta2[1] = (atan2(ts, tc)) / Angle2Rad;
					theta2[2] = (-atan2(ts, tc)) / Angle2Rad;
					theta2[3] = (-atan2(ts, -tc)) / Angle2Rad;
				}
				//���ץ��W��
				for (int i = 0; i < 4; i++)
				{

					if (theta2[i] != -999)
					{
						if (theta2[i] > 180)
						{
							theta2[i] = theta2[i] - 360;
						}
						if (theta2[i] < -180)
						{
							theta2[i] = theta2[i] + 360;
						}
						if ((theta2[i] <= 65 || theta2[i] >= 180)) //�_���I
						{
							theta2[i] = -999;
						}
					}
				}

				//-------------------------------------------------------------------------------
				for (int l = 0; l < 4; l++)
				{
					if (theta2[l] != -999)
					{

						J2 = theta2[l] * Angle2Rad;
						{
							//J3--------------------------------------------------------------
							// tc = (ax1*(sin(J4)*sin(J6) + cos(J4)*cos(J5)*cos(J6)) + ay1*(cos(J6)*sin(J4) - cos(J4)*cos(J5)*sin(J6)) + az1*cos(J4)*sin(J5))/sin(J2);
							float tc = (ax1 * (sin(J4) * sin(J6) + cos(J4) * cos(J5) * cos(J6)) + ay1 * (cos(J6) * sin(J4) - cos(J4) * cos(J5) * sin(J6)) + az1 * cos(J4) * sin(J5)) / sin(J2);
							if (tc > 1)
								tc = 1;
							else if (tc < -1)
								tc = -1;
							float ts = pow((1 - pow(tc, 2)), 0.5);
							if (ts > 1)
								ts = 1;
							else if (ts < -1)
								ts = -1;
							theta3[0] = (atan2(ts, -tc)) / Angle2Rad;
							theta3[1] = (atan2(ts, tc)) / Angle2Rad;
							theta3[2] = (-atan2(ts, tc)) / Angle2Rad;
							theta3[3] = (-atan2(ts, -tc)) / Angle2Rad;
						}
						//���ץ��W��
						for (int i = 0; i < 4; i++)
						{
							if (theta3[i] != -999)
							{
								if (theta3[i] > 180)
								{
									theta3[i] = theta3[i] - 360;
								}
								if (theta3[i] < -180)
								{
									theta3[i] = theta3[i] + 360;
								}
							}
						}

						//-------------------------------------------------------------------------------
						for (int m = 0; m < 4; m++)
						{
							if (theta3[m] != -999)
							{

								J3 = theta3[m] * Angle2Rad;
								{
									//J1----------------------------------------------------------------------
									//tc= (nz1*sin(J4)*sin(J5) - ny1*(cos(J4)*cos(J6) + cos(J5)*sin(J4)*sin(J6)) - nx1*(cos(J4)*sin(J6) - cos(J5)*cos(J6)*sin(J4)))/(-sin(J2));

									float tc = (nz1 * sin(J4) * sin(J5) - ny1 * (cos(J4) * cos(J6) + cos(J5) * sin(J4) * sin(J6)) - nx1 * (cos(J4) * sin(J6) - cos(J5) * cos(J6) * sin(J4))) / (-sin(J2));
									if (tc > 1)
										tc = 1;
									else if (tc < -1)
										tc = -1;
									float ts = pow((1 - pow(tc, 2)), 0.5);
									if (ts > 1)
										ts = 1;
									else if (ts < -1)
										ts = -1;
									theta1[0] = (atan2(ts, -tc)) / Angle2Rad;
									theta1[1] = (atan2(ts, tc)) / Angle2Rad;
									theta1[2] = (-atan2(ts, tc)) / Angle2Rad;
									theta1[3] = (-atan2(ts, -tc)) / Angle2Rad;
								}
								//���ץ��W��
								for (int i = 0; i < 4; i++)
								{
									if (theta1[i] != -999)
									{
										if (theta1[i] > 180)
										{
											theta1[i] = theta1[i] - 360;
										}
										if (theta1[i] < -180)
										{
											theta1[i] = theta1[i] + 360;
										}
									}
								}

								//-------------------------------------------------------------------------------
								for (int n = 0; n < 4; n++)
								{
									if (theta1[n] != -999)
									{

										J1 = theta1[n] * Angle2Rad;
										//���i�઺��-------------------------------------------b------------

										bool check = true;
										//�����X�Ӫ��ѻP�쥻���O�_���P
										float j1, j2, j3, j4, j5, j6;
										j1 = J1 * Rad2Angle;
										j2 = J2 * Rad2Angle;
										j3 = J3 * Rad2Angle;
										j4 = J4 * Rad2Angle;
										j5 = J5 * Rad2Angle;
										j6 = J6 * Rad2Angle;
										cv::Mat *tmpT = Calculate_ArmForwardKinematics(pre_angle, J1, J2, J3, J4, J5, J6);

										for (int o = 0; o < 4; o++)
										{
											for (int p = 0; p < 4; p++)
											{

												if (round(round_value * tmpT->at<float>(o, p)) != round(round_value * T->at<float>(o, p)))
												{

													check = false;
												}
											}
											//cout << endl;
										}
										if (check)
										{

											float temp_theta[6], re_theta[6];
											float tmp1 = J1 * Rad2Angle;
											float tmp2 = J2 * Rad2Angle;
											float tmp3 = J3 * Rad2Angle;
											float tmp4 = J4 * Rad2Angle;
											float tmp5 = J5 * Rad2Angle;
											float tmp6 = J6 * Rad2Angle;

											re_theta[0] = J1 * Rad2Angle;
											re_theta[1] = J2 * Rad2Angle;
											re_theta[2] = J3 * Rad2Angle;
											re_theta[3] = J4 * Rad2Angle;
											re_theta[4] = J5 * Rad2Angle;
											re_theta[5] = J6 * Rad2Angle;

											temp_theta[0] = J1 * Rad2Angle + 90;
											temp_theta[1] = J2 * Rad2Angle - 90;
											temp_theta[2] = J3 * Rad2Angle - 90;
											temp_theta[3] = J4 * Rad2Angle;
											temp_theta[4] = J5 * Rad2Angle + 180;
											temp_theta[5] = J6 * Rad2Angle - 90;

											for (int o = 0; o < 6; o++)
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
											
											// Motor direction adjustment
											temp_theta[0] = (temp_theta[0]);
											temp_theta[1] = -(temp_theta[1]);
											temp_theta[2] = -(temp_theta[2]);
											temp_theta[3] = (temp_theta[3]);
											temp_theta[4] = (temp_theta[4]);
											temp_theta[5] = -(temp_theta[5]);

											int tmp_delta_Ang =
												10*abs(temp_theta[0] - GetMotor_PresentAngle(FIRST_HAND_ID))
												+ abs(temp_theta[1] - GetMotor_PresentAngle(FIRST_HAND_ID + 1))
												+ abs(temp_theta[2] - GetMotor_PresentAngle(FIRST_HAND_ID + 2))
												+ abs(temp_theta[3] - GetMotor_PresentAngle(FIRST_HAND_ID + 3))
												+ abs(temp_theta[4] - GetMotor_PresentAngle(FIRST_HAND_ID + 4))
												+ abs(temp_theta[5] - GetMotor_PresentAngle(FIRST_HAND_ID + 5));
											
											if (tmp_delta_Ang < delta_Ang)
											{
												delta_Ang = tmp_delta_Ang;
												jacobian = Jacobian(J1, J2, J3, J4, J5, J6, tmpT);

												for (int o = 0; o < 6; o++)
												{
													Solution[o] = temp_theta[o];
												}
												ArmMotionEnable = true;
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
	if (Solution[0] == 0 && Solution[1] == 0 && Solution[2] == 0 && Solution[3] == 0 && Solution[4] == 0 && Solution[5] == 0)
	{
		ArmMotionEnable = false;
	}

	Solution[6] = pre_angle;
	return Solution;
}
float *SaleArmRight::CenterToArm(float x, float y, float z)
{
	float *Solution = new float[3];

	Solution[0] = x;
	Solution[1] = z;
	Solution[2] = -y - 430 / 2 - 5;
	return Solution;
}
float *SaleArmRight::CenterToShoulder(float x, float y, float z)
{
	float *Solution = new float[3];

	Solution[0] = x;
	Solution[1] = z + dis_CenterZ_To_ShoulderY_Ori - Get_Scrw_Shift(); //����V�U���t�A�V�W�����A�����O�t��
	Solution[2] = -y - 346 / 2;
	return Solution;
}

float *SaleArmRight::ShoulderToArm(float x, float y, float z)
{
	float *Solution = new float[3];
	double shoulder_angle = 0;
	double dis_shoulder2arm = 13;
	double b = 120;

	cv::Mat T_Shift = cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));
	cv::Mat T_Turn = cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));

	cv::Mat Coordinate_Old = cv::Mat(4, 1, CV_32FC1, cv::Scalar::all(0));
	cv::Mat Coordinate_New = cv::Mat(4, 1, CV_32FC1, cv::Scalar::all(0));

	double temp = cos(shoulder_angle * 3.14 / 180);

	T_Turn.at<float>(0, 0) = cos(shoulder_angle * 3.14 / 180);
	T_Turn.at<float>(0, 1) = 0;
	T_Turn.at<float>(0, 2) = sin(shoulder_angle * 3.14 / 180);
	T_Turn.at<float>(0, 3) = 0;

	T_Turn.at<float>(1, 0) = 0;
	T_Turn.at<float>(1, 1) = 1;
	T_Turn.at<float>(1, 2) = 0;
	T_Turn.at<float>(1, 3) = 0;

	T_Turn.at<float>(2, 0) = -sin(shoulder_angle * 3.14 / 180);
	T_Turn.at<float>(2, 1) = 0;
	T_Turn.at<float>(2, 2) = cos(shoulder_angle * 3.14 / 180);
	T_Turn.at<float>(2, 3) = 0;

	T_Turn.at<float>(3, 0) = 0;
	T_Turn.at<float>(3, 1) = 0;
	T_Turn.at<float>(3, 2) = 0;
	T_Turn.at<float>(3, 3) = 1;

	T_Shift.at<float>(0, 0) = 1;
	T_Shift.at<float>(0, 1) = 0;
	T_Shift.at<float>(0, 2) = 0;
	T_Shift.at<float>(0, 3) = -b * sin(shoulder_angle * 3.14 / 180);

	T_Shift.at<float>(1, 0) = 0;
	T_Shift.at<float>(1, 1) = 1;
	T_Shift.at<float>(1, 2) = 0;
	T_Shift.at<float>(1, 3) = 0;

	T_Shift.at<float>(2, 0) = 0;
	T_Shift.at<float>(2, 1) = 0;
	T_Shift.at<float>(2, 2) = 1;
	T_Shift.at<float>(2, 3) = -b * cos(shoulder_angle * 3.14 / 180);

	T_Shift.at<float>(3, 3) = 0;
	T_Shift.at<float>(3, 3) = 0;
	T_Shift.at<float>(3, 3) = 0;
	T_Shift.at<float>(3, 3) = 1;

	Coordinate_Old.at<float>(0, 0) = x;
	Coordinate_Old.at<float>(1, 0) = y;
	Coordinate_Old.at<float>(2, 0) = z;
	Coordinate_Old.at<float>(3, 0) = 1;

	Coordinate_New = T_Shift * T_Turn * Coordinate_Old;

	Solution[0] = Coordinate_New.at<float>(0, 0);
	Solution[1] = Coordinate_New.at<float>(1, 0);
	Solution[2] = Coordinate_New.at<float>(2, 0);
	return Solution;
}
float *SaleArmRight::ArmToShoulder(float shoulder_angle, float x, float y, float z)
{
	// *** shoulder_angle in RADIAN
	float *Solution = new float[3];
	double b = 120;

	cv::Mat T_Shift = cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));
	cv::Mat T_Turn = cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));

	cv::Mat T_Shift_inv = cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));
	cv::Mat T_Turn_inv = cv::Mat(4, 4, CV_32FC1, cv::Scalar::all(0));

	cv::Mat Coordinate_Old = cv::Mat(4, 1, CV_32FC1, cv::Scalar::all(0));
	cv::Mat Coordinate_New = cv::Mat(4, 1, CV_32FC1, cv::Scalar::all(0));

	T_Turn.at<float>(0, 0) = cos(shoulder_angle );
	T_Turn.at<float>(0, 1) = 0;
	T_Turn.at<float>(0, 2) = sin(shoulder_angle);
	T_Turn.at<float>(0, 3) = 0;

	T_Turn.at<float>(1, 0) = 0;
	T_Turn.at<float>(1, 1) = 1;
	T_Turn.at<float>(1, 2) = 0;
	T_Turn.at<float>(1, 3) = 0;

	T_Turn.at<float>(2, 0) = -sin(shoulder_angle);
	T_Turn.at<float>(2, 1) = 0;
	T_Turn.at<float>(2, 2) = cos(shoulder_angle);
	T_Turn.at<float>(2, 3) = 0;

	T_Turn.at<float>(3, 0) = 0;
	T_Turn.at<float>(3, 1) = 0;
	T_Turn.at<float>(3, 2) = 0;
	T_Turn.at<float>(3, 3) = 1;

	T_Turn_inv = T_Turn.inv();

	T_Shift.at<float>(0, 0) = 1;
	T_Shift.at<float>(0, 1) = 0;
	T_Shift.at<float>(0, 2) = 0;
	T_Shift.at<float>(0, 3) = -b * sin(shoulder_angle);

	T_Shift.at<float>(1, 0) = 0;
	T_Shift.at<float>(1, 1) = 1;
	T_Shift.at<float>(1, 2) = 0;
	T_Shift.at<float>(1, 3) = 0;

	T_Shift.at<float>(2, 0) = 0;
	T_Shift.at<float>(2, 1) = 0;
	T_Shift.at<float>(2, 2) = 1;
	T_Shift.at<float>(2, 3) = -b * cos(shoulder_angle);

	T_Shift.at<float>(3, 3) = 0;
	T_Shift.at<float>(3, 3) = 0;
	T_Shift.at<float>(3, 3) = 0;
	T_Shift.at<float>(3, 3) = 1;

	T_Shift_inv = T_Shift.inv();

	Coordinate_Old.at<float>(0, 0) = x;
	Coordinate_Old.at<float>(1, 0) = y;
	Coordinate_Old.at<float>(2, 0) = z;
	Coordinate_Old.at<float>(3, 0) = 1;

	Coordinate_New = T_Turn_inv * T_Shift_inv * Coordinate_Old;

	Solution[0] = Coordinate_New.at<float>(0, 0);
	Solution[1] = Coordinate_New.at<float>(1, 0);
	Solution[2] = Coordinate_New.at<float>(2, 0) + 10;
	return Solution;
}

float *SaleArmRight::ShoulderToCenter(float x, float y, float z)
{
	float *Solution = new float[3];

	Solution[0] = x;
	Solution[2] = y - dis_CenterZ_To_ShoulderY_Ori + Get_Scrw_Shift();
	Solution[1] = -z - 346 / 2 + 10;

	return Solution;
}
float SaleArmRight::Get_Scrw_Shift()
{
	std::string readFileName = "ScrewBall_Height.txt";
	std::ifstream in(readFileName.c_str());
	std::string inputStr;
	std::vector<float> inputContent;
	while (std::getline(in, inputStr))
	{
		inputContent.push_back(std::stof(inputStr));
	}
	in.close();
	if (inputContent.size() > 1)
	{

		height_shift_now = inputContent[0];
		return inputContent[0];
	}
	else
		return height_shift_now;
}
void SaleArmRight::ShoulderTurn(float angle)
{
	SetMotor_Angle(FIRST_SHOULDER_ID, angle);
	WaitAllMotorsArrival();
}