#include "Scaratest.h"

Scaratest::Scaratest()
{
    cScara = Scara::getScara();
}

Scaratest::~Scaratest()
{
}

void Scaratest::test1()
{
    // go_target(casay, 0, 1, 0);

    // go_target(stations, 0, 0, 1);
    // go_target(stations, 0, 0, 0);
    // go_target(stations, 1, 0, 1);
    // go_target(stations, 1, 0, 0);
    go_target(stations, 2, 0, 1);
    go_target(stations, 2, 0, 0);
    go_target(stations, 3, 0, 1);
    go_target(stations, 3, 0, 0);
    go_target(stations, 4, 0, 1);
    go_target(stations, 4, 0, 0);
    go_target(stations, 5, 0, 1);
    go_target(stations, 5, 0, 0);

    // go_target(casay, 1, 10, 1);
    // float ans[2] = {};
    // cScara->CScaraArm->Reset();
    // cScara->CScaraArm->GotoPosition(CASAY[0][0], CASAY[0][1], CASAY[0][2], CASAY[0][3], CASAY[0][4], CASAY[0][5]);
    // ready_pos(casay, 0, 5, ans);
    // cScara->CScaraArm->GotoPosition(0,0, CASAY[0][2],ans[0], ans[1], 218.0f);
    // ready_pos(casay, 0, 3, ans);
    // cScara->CScaraArm->GotoPosition(0,0, CASAY[0][2],ans[0], ans[1], 218.0f);
    // ready_pos(casay, 0, 2, ans);
    // cScara->CScaraArm->GotoPosition(0,0, CASAY[0][2],ans[0], ans[1], 218.0f);
    // ready_pos(casay, 0, 1, ans);
    // cScara->CScaraArm->GotoPosition(0,0, CASAY[0][2],ans[0], ans[1], 218.0f);
    // std::cout<<1<<std::endl;
}

/**
 * Put or get waffer from Station
 *
 * @param type - cassay or station
 * @param id - ID of the station or cassay
 * @param drawer - which drawer to get waffer, start from 0 ~23, ignore for station
 * @param io - put in or get out, get out for 0, put in for 1
 */
void Scaratest::go_target(bool type, int id, int drawer, bool io){
    if (type == casay) {
        cassette(id, drawer, io);
    }
    else {
        station(id, io);
    }
}

/**
 * Ready position for the scara to action
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
 * Put or get waffer from cassette 
 * 
 * @param id - which cassette
 * @param drawer - which drawer to get or put, start from 0 ~23
 * @param io - put it in or get out, get out for 0, put in for 1
 */
void Scaratest::cassette(int id, int drawer, bool io) {
    float ready[6] = {};
    ready[2] = CASAY[id][2];
    ready_pos(casay, id, ready);
    go_straight(ready, CASAY[id], (CASAY[id][5] + drawer * DRAWER_H + (int)io * LIFT_DIS), DIV);
    io = !io;
    printf("io = %d\n", io);
    go_straight(CASAY[id], CASAY[id], (CASAY[id][5] + drawer * DRAWER_H + (int)io * LIFT_DIS), 1);
    go_straight(CASAY[id], ready, (CASAY[id][5] + drawer * DRAWER_H + (int)io * LIFT_DIS), DIV);
}

/**
 * Put or get waffer from Station
 * 
 * @param id - which station
 * @param io - putting it in or out, in for 1, out for 0
 */
void Scaratest::station(int id, bool io) {

    float ready[6] = {};
    ready[2] = STATE[id][2];
    ready_pos(stations, id, ready);
    go_straight(ready, STATE[id], (STATE[id][5] + (int)io * (LIFT_DIS + STAT_SHIFT)), DIV);
    io = !io;
    printf("io = %d\n", io);
    go_straight(STATE[id], STATE[id], (STATE[id][5] + (int)io * (LIFT_DIS + STAT_SHIFT)), 1);
    go_straight(STATE[id], ready, (STATE[id][5] + (int)io * (LIFT_DIS + STAT_SHIFT)), DIV);
}



/**
 * Go straight function
 * For scara able to move in straight line
 * @param head - start of the movement
 * @param tail - end of the movement
 * @param h - height of the scara arm
 * @param div - How many pieces we want to cut our movement
 */
void Scaratest::go_straight(float *head, float *tail, float h, float div) {
    float step[] = {0, 0};
    step[0] = (tail[3] - head[3])/div;
    step[1] = (tail[4] - head[4])/div;
    for (int i = 0; i < div; i++) {
        cScara->CScaraArm->GotoPosition(0,0, head[2], (head[3] + i * step[0]), (head[4] + i * step[1]), h);
        printf("readyx: %f, readyy: %f, x: %f, y: %f, z: %f\n",head[3], head[4], (head[3] + i * step[0]), (head[4] + i * step[1]), h);
    }
}