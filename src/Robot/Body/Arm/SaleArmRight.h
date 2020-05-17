#pragma once
#include "./BasicArm/Arm.h"

class SaleArmRight : public Arm
{
public:
	SaleArmRight(const vector<unsigned char> &IDArray,
				 const vector<string> &MotorModelArrayList,
				 vector<unsigned char> &AllPortNumber);
	~SaleArmRight(){};

	/* Kinematics */
	cv::Mat *GetKinematics();
	cv::Mat *Calculate_ArmForwardKinematics(float J1, float J2, float J3, float J4, float J5, float J6);
	cv::Mat *Calculate_ArmForwardKinematics(float pre_angle, float J1, float J2, float J3, float J4, float J5, float J6);

	// 6 DOFs
	float *Arm_InverseKinematics(cv::Mat *&T);
	// 7 DOFs
	float *Arm_InverseKinematics(const float &pre_anlge, cv::Mat *&T);

	/* Coordinate Relative */
	float *CenterToArm(float x, float y, float z);
	float *CenterToShoulder(float x, float y, float z);
	float *ShoulderToArm(float x, float y, float z);
	float *ArmToShoulder(float shoulder_angle, float x, float y, float z);
	float *ShoulderToCenter(float x, float y, float z);

	void ShoulderTurn(float angle);
	float Get_Scrw_Shift();

private:
	float Right_Hand_px, Right_Hand_py, Right_Hand_pz;
	int dis_CenterZ_To_ShoulderY_Ori;
	int CenterZ_To_ShoulderY_Ori_Shift;
	int height_shift_now = 0;
};