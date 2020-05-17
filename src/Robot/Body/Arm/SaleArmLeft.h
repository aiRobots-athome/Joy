#pragma once
#include "./BasicArm/Arm.h"

class SaleArmLeft : public Arm
{
public:
	SaleArmLeft(const vector<unsigned char> &IDArray,
				const vector<string> &MotorModelArrayList,
				vector<unsigned char> &AllPortNumber);
	~SaleArmLeft(){};

	/* Kinematics */
	cv::Mat *GetKinematics();
	cv::Mat *Calculate_ArmForwardKinematics(float J1, float J2, float J3, float J4, float J5, float J6);
	cv::Mat *Calculate_ArmForwardKinematics(float pre_angle, float J1, float J2, float J3, float J4, float J5, float J6);

	// 6 DOFs
	float *Arm_InverseKinematics(cv::Mat *&T);
	// 7 DOFs
	float *Arm_InverseKinematics(const float &pre_angle, cv::Mat *&T);

	/* Coordinate Relative */
	float *CenterToArm(float x, float y, float z);
	float *CenterToShoulder(float x, float y, float z);
	float *ShoulderToArm(float x, float y, float z);
	float *ArmToShoulder(float shoulder_angle, float x, float y, float z);
	float *ShoulderToCenter(float x, float y, float z);
	float Get_Scrw_Shift();

	void ShoulderTurn(float angle);

private:
	float Right_Hand_px, Right_Hand_py, Right_Hand_pz;
	int dis_CenterZ_To_ShoulderY_Ori;
	int height_shift_now;
};