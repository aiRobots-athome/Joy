#include "Scaratest.h"

Scaratest::Scaratest() {
    cScara = Scara::getScara();
    // float casay1[] = {0.f, 0.f, 58.f, 361.f, 774.f, 212.f};
    float casay1[] = {0.f, 0.f, -60.31f, 381.8f, -750.62f, 215.5f};
    set_pos(casay, 0, casay1);
    float stat1[] = {0.f, 0.f, 27.f, 947.f, 376.f, 238.f};
    set_pos(stations, 0, stat1);
    float stat2[] = {0.f, 0.f, 25.f, 939.f, 401.f, 90.f};
    set_pos(stations, 1, stat2);
    float stat3[] = {0.f, 0.f, -4.f, 1014.f, -40.f, 235.f};
    set_pos(stations, 2, stat3);
    float stat4[] = {0.f, 0.f, -1.f, 1036.f, -50.f, 85.f};
    set_pos(stations, 3, stat4);
    float stat5[] = {0.f, 0.f, -27.f, 891.f, -469.f, 235.f};
    set_pos(stations, 4, stat5);
    float stat6[] = {0.f, 0.f, -28.f, 898.f, -467.f, 87.f};
    set_pos(stations, 5, stat6);
    float casay2[] = {0.f, 0.f, -69.f, 526.f, -749.f, 206.f};
    set_pos(casay, 1, casay2);
}

void Scaratest::test1() {
    // Get waffer from level 1 in cassette A, and put it to level 10
    // Cassette A id = 0
    // Cassette B id = 1
    go_target(casay, 0, 4, 0);
    go_target(casay, 0, 19, 1);

    // Get waffer from level 10 in cassette A, and put it to level 1
    go_target(casay, 0, 19, 0);
    go_target(casay, 0, 4, 1);

    // // At station 0, id = 0
    // go_target(stations, 0, 0, 1);   // Put in waffer
    // go_target(stations, 0, 0, 0);   // Get waffer out

    // // At station 1, id = 1
    // go_target(stations, 1, 0, 1);   // Put in waffer
    // go_target(stations, 1, 0, 0);   // Get waffer out

    // // At station 2, id = 2
    // go_target(stations, 2, 0, 1);   // Put in waffer
    // go_target(stations, 2, 0, 0);   // Get waffer out

    // // At station 3, id = 3
    // go_target(stations, 3, 0, 1);   // Put in waffer
    // go_target(stations, 3, 0, 0);   // Get waffer out

    // // At station 4, id = 4
    // go_target(stations, 4, 0, 1);   // Put in waffer
    // go_target(stations, 4, 0, 0);   // Get waffer out

    // // At station 5, id = 5
    // go_target(stations, 5, 0, 1);   // Put in waffer
    // go_target(stations, 5, 0, 0);   // Get waffer out

    // Put waffer in cassette B
    // go_target(casay, 1, 10, 1);
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
    ready[5] = CASAY[id][5];
    ready_pos(casay, id, ready);    // Get ready pos

    cScara->CScaraArm->GoToPosition(ready, (ready[5] + drawer * DRAWER_H + (int)io * LIFT_DIS), DIV);
    cScara->CScaraArm->GoToPosition(CASAY[id], (CASAY[id][5] + drawer * DRAWER_H + (int)io * LIFT_DIS), DIV);    // Move from ready to cassette
    io = !io;   // inverse io value for easier caculation

    // printf("io = %d\n", io);    // FOR DEBUG
    cScara->CScaraArm->GoToPosition(CASAY[id], (CASAY[id][5] + drawer * DRAWER_H + (int)io * LIFT_DIS), DIV);  // Lift/place waffer
    cScara->CScaraArm->GoToPosition(ready, (CASAY[id][5] + drawer * DRAWER_H + (int)io * LIFT_DIS), DIV);    // Move back to ready position
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
    ready[5] = STATE[id][5];
    ready_pos(stations, id, ready);    // Get ready pos
    
    cScara->CScaraArm->GoToPosition(ready, (ready[5] + (int)io * (LIFT_DIS + STAT_SHIFT)), DIV);
    cScara->CScaraArm->GoToPosition(STATE[id], (STATE[id][5] + (int)io * (LIFT_DIS + STAT_SHIFT)), DIV);    // Move from ready to cassette

    io = !io;    
    // printf("io = %d\n", io);    // FOR DEBUG
    cScara->CScaraArm->GoToPosition(STATE[id], (STATE[id][5] + (int)io * (LIFT_DIS + STAT_SHIFT)), DIV);  // Lift/place waffer
    cScara->CScaraArm->GoToPosition(ready, (STATE[id][5] + (int)io * (LIFT_DIS + STAT_SHIFT)), DIV);    // Move back to ready position
}

Scaratest::~Scaratest() {}