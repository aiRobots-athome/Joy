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
        void go_target(bool, int, int, bool);





        float CASAY[2][6] = {   {0, 0, 62, 593, 737, 213},
                                {0, 0, -61, 590, -752, 207} };
        float STATE[6][6] = {   {0, 0, 33, 921, 432, 240},
                                {0, 0, 32, 930, 410, 111},
                                {0, 0, 0, 1037, -9, 238},
                                {0, 0, 0, 1037, -10, 110},
                                {0, 0, -32, 914, -439, 236},
                                {0, 0, -35, 900, -433, 106} };

        float DRAWER_H = 6.3;
        float STAT_SHIFT = 10;
        float LIFT_DIS = 3;
        float SAFE_DIS = 300;
        float DIV = 50;
    
    private:
        Scara *cScara;
        void ready_pos( bool, int, float*);
        void cassette(int, int, bool);
        void station(int, bool);
        void go_straight(float*, float*, float, float);

};