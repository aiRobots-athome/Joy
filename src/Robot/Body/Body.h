#pragma once
#include "./MobilePlatform/MobilePlatform.h"
#include "./Arm/SaleArmLeft.h"
#include "./Arm/SaleArmRight.h"
#include "./HeadandLiftingPlatform/HeadandLiftingPlatform.h"
class Body
{
public:
    static Body *getBody();
    ~Body();

    HeadandLiftingPlatform *CHeadandLifting;
    MobilePlatform *CMobilePlatform;
    SaleArmLeft *CSaleArmLeft;
    SaleArmRight *CSaleArmRight;

private:
    static Body *inst_;
    Body();
};