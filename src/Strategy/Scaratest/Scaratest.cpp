#include "Scaratest.h"

Scaratest::Scaratest() {
    cScara = Scara::getScara();
}

void Scaratest::test1() {
    float tmp[] = {0.f, 0.f, 62.f, 593.f, 737.f, 213.f};
    set_pos(casay, 0, tmp);
    // Get waffer from cassette A
    // Cassette A id = 0
    // Cassette B id = 1
    go_target(casay, 0, 1, 0);

    // At station 0, id = 0
    go_target(stations, 0, 0, 1);   // Put in waffer
    go_target(stations, 0, 0, 0);   // Get waffer out

    // At station 1, id = 1
    go_target(stations, 1, 0, 1);   // Put in waffer
    go_target(stations, 1, 0, 0);   // Get waffer out

    // At station 2, id = 2
    go_target(stations, 2, 0, 1);   // Put in waffer
    go_target(stations, 2, 0, 0);   // Get waffer out

    // At station 3, id = 3
    go_target(stations, 3, 0, 1);   // Put in waffer
    go_target(stations, 3, 0, 0);   // Get waffer out

    // At station 4, id = 4
    go_target(stations, 4, 0, 1);   // Put in waffer
    go_target(stations, 4, 0, 0);   // Get waffer out

    // At station 5, id = 5
    go_target(stations, 5, 0, 1);   // Put in waffer
    go_target(stations, 5, 0, 0);   // Get waffer out

    // Put waffer in cassette B
    go_target(casay, 1, 10, 1);
}

/**
 * Go to target to get or put waffer, no matter cassette or station
 *
 * @param type - cassay or station
 * @param id - ID of the station or cassay
 * @param drawer - which drawer to get waffer, start from 0 ~23, ignore for station
 * @param io - put in or get out, get out for 0, put in for 1
 */
void Scaratest::go_target(bool type, int id, int drawer, bool io) {
    if (type == casay) {
        cassette(id, drawer, io);
    }
    else {
        station(id, io);
    }
}

/**
 * Set cassette/station position
 * 
 * @param type - cassette or station
 * @param id - ID of the station or cassette
 * @param data - station/cassette location, oritation data
 * @retval 1 for success, 0 for fail
 */
bool Scaratest::set_pos(bool type, int id, float *data) {
    if(type == casay) {
        std::copy(data, data+6, std::begin(CASAY[id]));
        return 1;
    }
    else if (type = stations) {
        std::copy(data, data+6, std::begin(STATE[id]));
        return 1;
    }
    else
        return 0;
}


/**
 * Ready position for the scara to action
 * Ready position: The position before/after scara put/get waffer in/out station/cassette 
 * 
 * @param type - cassay or station
 * @param id - which cassette
 * @param ans - answer array for return
 */
void Scaratest::ready_pos(bool type, int id, float* ans) {
    if (type == casay) {             // For casay
        ans[3] = CASAY[id][3] - SAFE_DIS * cos(CASAY[id][2]/180*3.14159);
        ans[4] = CASAY[id][4] - SAFE_DIS * sin(CASAY[id][2]/180*3.14159);
    }
    else {             // For station
        ans[3] = STATE[id][3] - SAFE_DIS * cos(STATE[id][2]/180*3.14159);
        ans[4] = STATE[id][4] - SAFE_DIS * sin(STATE[id][2]/180*3.14159);
        printf("cos: %f, sin: %f", STATE[id][2], STATE[id][2]);
    }
}

/**
 * Proccess to put or get waffer from cassette
 * Height caculation: Height of first drawer + drawer height * drawer number + io * lifting distance
 * if io = 0, get waffer out, so no lifting height
 * if io = 1, put waffer in, need a lifting height to move waffer into cassette
 * 
 * @param id - which cassette
 * @param drawer - which drawer to get or put, start from 0 ~23
 * @param io - put it in or get out, get out for 0, put in for 1
 */
void Scaratest::cassette(int id, int drawer, bool io) {
    float ready[6] = {};
    ready[2] = CASAY[id][2];
    ready_pos(casay, id, ready);    // Get ready pos
    cScara->CScaraArm->go_straight_tmp(ready, CASAY[id], (CASAY[id][5] + drawer * DRAWER_H + (int)io * LIFT_DIS), DIV);    // Move from ready to cassette
    io = !io;   // inverse io value for easier caculation

    // printf("io = %d\n", io);    // FOR DEBUG
    cScara->CScaraArm->go_straight_tmp(CASAY[id], CASAY[id], (CASAY[id][5] + drawer * DRAWER_H + (int)io * LIFT_DIS), 1);  // Lift/place waffer
    cScara->CScaraArm->go_straight_tmp(CASAY[id], ready, (CASAY[id][5] + drawer * DRAWER_H + (int)io * LIFT_DIS), DIV);    // Move back to ready position
}

/**
 * Proccess to put or get waffer from cassette
 * Height caculation: Height of first drawer + io * (lifting distance + station bias)
 * if io = 0, get waffer out, so no lifting height
 * if io = 1, put waffer in, need a lifting height to move waffer into station
 * 
 * @param id - which station
 * @param io - putting it in or out, in for 1, out for 0
 */
void Scaratest::station(int id, bool io) {
    float ready[6] = {};
    ready[2] = STATE[id][2];
    ready_pos(stations, id, ready);    // Get ready pos
    cScara->CScaraArm->go_straight_tmp(ready, STATE[id], (STATE[id][5] + (int)io * (LIFT_DIS + STAT_SHIFT)), DIV);    // Move from ready to cassette

    // io = !io;    // FOR DEBUG
    printf("io = %d\n", io);
    cScara->CScaraArm->go_straight_tmp(STATE[id], STATE[id], (STATE[id][5] + (int)io * (LIFT_DIS + STAT_SHIFT)), 1);  // Lift/place waffer
    cScara->CScaraArm->go_straight_tmp(STATE[id], ready, (STATE[id][5] + (int)io * (LIFT_DIS + STAT_SHIFT)), DIV);    // Move back to ready position
}

/**
 * Go straight function
 * For scara able to move in straight line
 * 
 * @param head - start of the movement
 * @param tail - end of the movement
 * @param h - height of the scara arm
 * @param div - How many pieces we want to cut our movement
 */
// void Scaratest::go_straight(float *head, float *tail, float h, float div) {
//     float step[] = {0, 0};

//     // unit vector of target position
//     step[0] = (tail[3] - head[3])/div;
//     step[1] = (tail[4] - head[4])/div;

//     // Move straight by insert points into line
//     for (int i = 0; i < div; i++) {
//         cScara->CScaraArm->GotoPosition(0,0, head[2], (head[3] + i * step[0]), (head[4] + i * step[1]), h);
//         // printf("readyx: %f, readyy: %f, x: %f, y: %f, z: %f\n",head[3], head[4], (head[3] + i * step[0]), (head[4] + i * step[1]), h);   // DEBUG
//     }
// }

Scaratest::~Scaratest() {}