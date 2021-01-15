#pragma once
#include "../../MotorUnion/MotorUnion.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <vector>
#include <math.h>

class Arm : public MotorUnion
{
public:
	/* Constructor */
	Arm(const vector<unsigned char> &IDArray,
		const vector<string> &MotorModelArrayList);
	~Arm(){};

	void Start();
	void Stop();

    /* Get data function */
	Eigen::Matrix<float, 4, 4> GetTransformMatrix(const float &theta, const float &alpha, const float &a, const float &d);
	Eigen::Matrix<float, 3, 3> GetRotationMatrix(const int &axis_index, const float &theta);
	Eigen::Matrix<float, 6, 6> GetJacobianMatrix(void);
	float GetCurrentPosition(int index);
	float GetCurrentOrientation(int index);
	bool GetWorkingState(void);

	/* CORE function */
	virtual void CalculateJacobianMatrix(void) = 0;
	virtual void TrajectoryPlanning(const float &J0, const float &ox, const float &oy, const float &oz, const float &px, const float &py, const float &pz) = 0;
	
	/* Calculation function */
	int Sign(float x);
	float RootMeanSquareError(Eigen::Matrix<float, 3, 1> target_data, Eigen::Matrix<float, 3, 1> current_data);
	Eigen::Matrix<float, 6, 1> CheckMotorVelocity(Eigen::Matrix<float, 6, 1> motor_velocity);

protected:
    Eigen::Matrix<float, 6, 6> jacobian_matrix_;
	Eigen::Matrix<float, 6, 6> inverse_jacobian_matrix_;

	Eigen::Matrix<float, 3, 1> current_position_;
	Eigen::Matrix<float, 3, 1> current_orientation_;

	const unsigned char FIRST_SHOULDER_ID_;
	const unsigned char FIRST_HAND_ID_;
	const unsigned char FIRST_FINGER_ID_;

    int SHOULDER_LINK_LENGTH_;
    int UPPER_LINK_LENGTH_;
    int LOWER_LINK_LENGTH_;
    int END_EFFECTOR_LENGTH_;

	FILE *port_file_;

	bool is_out_of_limit_;
	bool is_working_;
	
	const float delay_time_;
	const float pro_radpersec2scale_;
};