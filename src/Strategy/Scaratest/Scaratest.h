#pragma once
#include "../../Scara/Scara.h"

enum {
    casay = 0,
    stations = 1,
};

class Scaratest
{
    public:
        Scaratest();
        ~Scaratest();
        void test1();
        bool set_pos(bool, int, float*);
        void go_target(bool, int, int, bool);

        // Cassette position, { x oritation, y orit, z orit, x, y, height of the first drawer}
        // Height of the first drawer: The height end effector is able to move in without bumping waffers,
        // and able to lift the first waffer by moving up LIFT_DIS
        float CASAY[2][6] = {   {0, 0, 62, 593, 737, 213},
                                {0, 0, -61, 590, -752, 207} };

        // Station position, { x oritation, y orit, z orit, x, y, height of the station}
        // Height of the station: The height end effector is able to move in without bumping waffers,
        // and able to lift the waffer placing on station by moving up LIFT_DIS
        float STATE[6][6] = {   {0, 0, 33, 921, 432, 240},
                                {0, 0, 32, 930, 410, 111},
                                {0, 0, 0, 1037, -9, 238},
                                {0, 0, 0, 1037, -10, 110},
                                {0, 0, -32, 914, -439, 236},
                                {0, 0, -35, 900, -433, 106} };

        float DRAWER_H = 6.3;   // Height between drawer and drawer
        float STAT_SHIFT = 10;  // Special bias for us to lift the waffer from station
        float LIFT_DIS = 3;     // Lifting height for us to lift the waffer from both cassette and station
        float SAFE_DIS = 300;   // Distance between cassette/station and ready position
        float DIV = 50;         // How many pieces is the route being cut
    
    private:
        Scara *cScara;
        void ready_pos( bool, int, float*);
        void cassette(int, int, bool);
        void station(int, bool);
        void go_straight(float*, float*, float, float);

};