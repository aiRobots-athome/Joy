#pragma once
#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include "../../Robot/MotorUnion/MotorUnion.h"

class ScaraArm : public MotorUnion
{
public:
    static ScaraArm *getScaraArm();
    ~ScaraArm() { inst_ = nullptr; };

    void Start();
    void Stop();
    void Reset();

    /* CORE function */
    void CalculateJacobianMatrix(void);
    void TrajectoryPlanning(const float &oz, const float &px, const float &py);
    bool GoScrewHeight(const float &goal_height);

    /* Set data function */
    void SetArmVelocity(float v0, float v1, float v2);

    /* Get data function */
    Eigen::Matrix<float, 4, 4> GetTransformMatrix(const float &theta, const float &alpha, const float &a, const float &d);
    Eigen::Matrix<float, 3, 3> GetRotationMatrix(const int &axis_index, const float &theta);
    Eigen::Matrix<float, 3, 3> GetJacobianMatrix(void);
    float GetCurrentPosition(int index);
    float GetCurrentOrientation(int index);
    float GetPresentHeight();
    bool GetWorkingState(void);

    /* Functional */
    float RootMeanSquareError(Eigen::Matrix<float, 3, 1> target_data, Eigen::Matrix<float, 3, 1> current_data);
    Eigen::Matrix<float, 3, 1> CheckMotorVelocity(Eigen::Matrix<float, 3, 1> motor_velocity);

private:
    // Screw
    void ReadHeight();
    void WriteHeight(const float &height) const;

private:
    ScaraArm();
    static ScaraArm *inst_;

    const unsigned char SCREW_MOTOR_ID_;
    const float LINK0_LENGTH_;
    const float LINK1_LENGTH_;
    const float LINK2_LENGTH_;
    const float PRO_RADS2SCALE_; // Used to map from RADIAN to Pro series SCALE (Protocol 2)
    const float MX_RADS2SCALE_;  // Used to map from RADIAN to Mx series SCALE (Protocol 2)

    const int PRO200_RESOLUTION_; // Resolution of Pro200
    const int MX106_RESOLUTION_;  // Resolution of Mx106
    const int REV_2_SCREW;        // 1 rev equal to how many screw

    float current_screw_height_;

    Eigen::Matrix<float, 3, 3> jacobian_matrix_;
    Eigen::Matrix<float, 3, 3> inverse_jacobian_matrix_;

    Eigen::Matrix<float, 3, 1> current_position_;
    Eigen::Matrix<float, 3, 1> current_orientation_;

    bool is_out_of_limit_; // True if predicted motor velocity is out of range
    bool is_working_;      // True if arm is moving
};