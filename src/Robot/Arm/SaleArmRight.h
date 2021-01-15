#pragma once
#include "./BasicArm/Arm.h"
#include <time.h>

class SaleArmRight : public Arm
{
public:
	static SaleArmRight *getSaleArmRight();
	~SaleArmRight() { inst_ = nullptr; };

    void CalculateJacobianMatrix(void);	
	void SetArmVelocity(float v0, float v1, float v2, float v3, float v4, float v5, float v6);
	void TrajectoryPlanning(const float &J0, const float &ox, const float &oy, const float &oz, const float &px, const float &py, const float &pz);

	void PneumaticOn(void);
	void PneumaticOff(void);

private:
	SaleArmRight();
	static SaleArmRight *inst_;
};