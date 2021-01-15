#pragma once
#include "../Robot/MotorUnion/MotorUnion.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <vector>
#include <math.h>

class May : public MotorUnion
{
public:
    /* Constructor */
    static May *getMay();
    May();
    ~May() { inst_ = nullptr; };

    void Start();
    void Stop();

    /* CORE function */
    void CalculateJacobianMatrix(void);
    void TrajectoryPlanning(const float &ox, const float &oy, const float &oz, const float &px, const float &py, const float &pz);
    void ImpedanceControl(const float &om, const float &ob, const float &ok, const float &pm, const float &pb, const float &pk,
                          const float &ox, const float &oy, const float &oz, const float &px, const float &py, const float &pz);

    /* Get data function */
    Eigen::Matrix<float, 4, 4> GetTransformMatrix(const float &theta, const float &alpha, const float &a, const float &d);
    Eigen::Matrix<float, 3, 3> GetRotationMatrix(const int &axis_index, const float &theta);
    Eigen::Matrix<float, 6, 6> GetJacobianMatrix(void);
    float GetCurrentPosition(int index);
    float GetCurrentOrientation(int index);
    bool GetWorkingState(void);

    /* Set data function */
    void SetArmVelocity(float v0, float v1, float v2, float v3, float v4, float v5);

    /* Gripper */
    void PneumaticOn(void);
    void PneumaticOff(void);

    /* Functional */
    int Sign(float x);
    float RootMeanSquareError(Eigen::Matrix<float, 3, 1> target_data, Eigen::Matrix<float, 3, 1> current_data);  
    Eigen::Matrix<float, 6, 1> CheckMotorVelocity(Eigen::Matrix<float, 6, 1> motor_velocity);  

    const float pro_radpersec2scale_;   

private:
    static May *inst_;

    Eigen::Matrix<float, 6, 6> jacobian_matrix_;
    Eigen::Matrix<float, 6, 6> inverse_jacobian_matrix_;

    Eigen::Matrix<float, 3, 1> current_position_;
    Eigen::Matrix<float, 3, 1> current_orientation_;

    int END_EFFECTOR_LENGTH_;

    FILE *port_file_;
    
    bool is_out_of_limit_;
    bool is_working_;    
};