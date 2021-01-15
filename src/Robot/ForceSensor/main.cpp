#include "ForceSensor.h"
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Joe");

    ForceSensor *CForseSensor;
    CForseSensor = new ForceSensor(0, "/dev/ttyUSB0");

    while(true)
    {
        this_thread::sleep_for(chrono::milliseconds(100));
        std::cout << CForseSensor->GetNormalizedFx() << "\t"
    			  << CForseSensor->GetNormalizedFy() << "\t"
    			  << CForseSensor->GetNormalizedFz() << "\t"
    			  << CForseSensor->GetNormalizedMx() << "\t"
    			  << CForseSensor->GetNormalizedMy() << "\t"
    			  << CForseSensor->GetNormalizedMz() << std::endl;
    }

    return 0;
}