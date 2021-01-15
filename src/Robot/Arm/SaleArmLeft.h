#pragma once
#include "./BasicArm/Arm.h"
#include <time.h>

class SaleArmLeft : public Arm
{
public:
	static SaleArmLeft *getSaleArmLeft();
	~SaleArmLeft() { inst_ = nullptr; };

    void CalculateJacobianMatrix(void);	
	void SetArmVelocity(float v0, float v1, float v2, float v3, float v4, float v5, float v6);
	void TrajectoryPlanning(const float &J0, const float &ox, const float &oy, const float &oz, const float &px, const float &py, const float &pz);
	void ImpedanceControl(const float &J0, const float &ox, const float &oy, const float &oz, const float &px, const float &py, const float &pz);

	void PneumaticOn(void);
	void PneumaticOff(void);

private:
	SaleArmLeft();
	static SaleArmLeft *inst_;
};