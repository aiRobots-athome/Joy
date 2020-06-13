#include "Scara.h"

Scara *Scara::inst_ = nullptr;
Scara *CScara = Scara::getScara();
ScaraArm *CScaraArm = CScara->CScaraArm;


Scara *Scara::getScara()
{
    if (inst_ == nullptr)
        inst_ = new Scara();
    return inst_;
}

Scara::Scara()
{
    vector<unsigned char> allport = {0};
    /////////////////////////////////////////////////////////////////////
    /// Construct ScaraArm //////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    vector<unsigned char> ScaraArmID = {0, 1, 2, 3};
    vector<string> ScaraArmModel = {"Pro200", "Pro200", "Pro20", "Pro20"};
    CScaraArm = new ScaraArm(ScaraArmID, ScaraArmModel, allport);
    cout << "\t\tClass constructed: ScaraArm" << endl;

    /////////////////////////////////////////////////////////////////////
    /// Construct XY Platform ///////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    
    
    cout << "\tClass constructed: Scara" << endl;
}

Scara::~Scara()
{
    delete CScaraArm;
}