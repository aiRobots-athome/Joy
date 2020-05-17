#pragma once
#include "../../MotorUnion/MotorUnion.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <math.h>

class Arm : public MotorUnion
{
public:
	/* Constructor */
	Arm(const vector<unsigned char> &IDArray,
		const vector<string> &MotorModelArrayList,
		vector<unsigned char> &AllPortNumber);
	~Arm(void){};

	/* Initialize Motor Parameters */
	void InitArmMotor();
	void ResetAllMotorAngle();

	/* Virtual Function */
	// Caculate Forward Kinematics
	virtual cv::Mat *GetKinematics(void) = 0;
	virtual cv::Mat *Calculate_ArmForwardKinematics(float J1, float J2, float J3, float J4, float J5, float J6) = 0;
	virtual cv::Mat *Calculate_ArmForwardKinematics(float pre_angle, float J1, float J2, float J3, float J4, float J5, float J6) = 0;

	// Caculate Inverse Kinematics
	virtual float *Arm_InverseKinematics(cv::Mat *&T) = 0;
	virtual float *Arm_InverseKinematics(const float &pre_angle, cv::Mat *&T) = 0;

	virtual float *CenterToArm(float x, float y, float z) = 0;

	/* Main Arm Kinematic Function */
	// 6 DOFs GotoPosition
	void GotoPosition(cv::Mat *&T);
	void GotoPosition(const int &ox, const int &oy, const int &oz, const int &x, const int &y, const int &z);
	// 7 DOFs GotoPosition
	void GotoPosition(const float &pre_angle, cv::Mat *&T);
	void GotoPosition(const float &pre_angle, const int &ox, const int &oy, const int &oz, const int &x, const int &y, const int &z);

	// Manipulation of end-effector
	void GraspObj(int angle); // 0 in degree
	void ReleaseObj();		  // -90 in degree

	/* Sencondary Kinematic Function */

	// Transforming Pitch-Raw-Yaw into Matrix
	cv::Mat *TransRotate(const float &x_angle, const float &y_angle, const float &z_angle);

	// Six length may differ as the end-effector changes, which is used in inverse kinematics
	void SetSixLength(float length);
	void Stop();
	void VitualGrippingJaw(void);

	// Used in InverseKinematics
	cv::Mat *Jacobian(float J1, float J2, float J3, float J4, float J5, float J6, cv::Mat *T0);
	float *CalculateRotateAngle(cv::Mat *);

protected:
	cv::Mat *jacobian;

	bool ArmMotionEnable;
	bool VitualGrippingJaw_Enable;
	float Vitual_Distance6;

	// First ID Setting for General Arm
	const unsigned char	FIRST_SHOULDER_ID;
	const unsigned char FIRST_HAND_ID;
	const unsigned char	FIRST_FINGER_ID;

	// Amount Setting
	const unsigned char HAND_AMOUNT;

	float Motor_Distance6_ini;
	int alength6_ini;
	int round_value;

	int F_SholderCenter, B_SholderCenter;
	float alength_2,
		alength_3,
		alength_6,
		Motor_Distance_2,
		Motor_Distance_3,
		Motor_Distance_4,
		Motor_Distance_5,
		Motor_Distance_6;

	cv::Mat *ArmForward;
	cv::Mat *ArmForward_r;
};