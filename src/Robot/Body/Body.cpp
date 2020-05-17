#include "Body.h"
Body *Body::inst_ = nullptr;
Body *Body::getBody()
{
    if (inst_ == nullptr)
        inst_ = new Body();
    return inst_;
}

Body::Body()
{
    vector<unsigned char> allport = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    /////////////////////////////////////////////////////////////////////
    /// Construct Heads /////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    vector<unsigned char> HeadID = {8, 9};
    vector<string> HeadModel = {"Mx64", "Mx64"};
    CHeadandLifting = new HeadandLiftingPlatform(HeadID, HeadModel, allport);
    cout << "\t\tClass constructed: HeadandLiftingPlatform" << endl;

    /////////////////////////////////////////////////////////////////////
    /// Construct Two Arms //////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    vector<unsigned char> LeftArmID = {13, 14, 15, 16, 17, 18, 19, 20};
    vector<string> LeftArmModel = {"Pro200", "Pro200", "Pro200", "Pro20", "Pro20", "Mx106", "Pro20", "Mx106"};
    CSaleArmLeft = new SaleArmLeft(LeftArmID, LeftArmModel, allport);
    cout << "\t\tClass constructed: SaleArmLeft" << endl;

    vector<unsigned char> RightArmID = {23, 24, 25, 26, 27, 28, 29, 30};
    vector<string> RightArmModel = {"Pro200", "Pro200", "Pro200", "Pro20", "Pro20", "Mx106", "Pro20", "Mx106"};
    CSaleArmRight = new SaleArmRight(RightArmID, RightArmModel, allport);
    cout << "\t\tClass constructed: SaleArmRight" << endl;

    /////////////////////////////////////////////////////////////////////
    /// Construct MobilePlatform ////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    vector<unsigned char> steeringID = {0, 1, 2, 3};
    vector<string> steeringModel = {"Pro20", "Pro20", "Pro20", "Pro20"};
    vector<unsigned char> wheelID = {4, 5, 6, 7};
    vector<string> wheelModel = {"Pro20+", "Pro20+", "Pro20+", "Pro20+"};
    CMobilePlatform = new MobilePlatform(steeringID, steeringModel, wheelID, wheelModel, allport);
    cout << "\t\tClass constructed: MobilePlatform" << endl;
    cout << "\tClass constructed: Body" << endl;
}

Body::~Body()
{
    delete CHeadandLifting;
    delete CMobilePlatform;
    delete CSaleArmLeft;
    delete CSaleArmRight;
}