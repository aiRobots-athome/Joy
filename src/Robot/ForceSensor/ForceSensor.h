#ifndef FORCESENSOR_H
#define FORCESENSOR_H

/* ForseSensor */
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <termios.h>
#include <strings.h>
#include <fcntl.h>
#include <unistd.h>

/* ROS */
#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"

using namespace std;

class ForceSensor
{
public:
    ForceSensor(std::string port_name);
    ~ForceSensor();

    /* set as public in case user need to renormalize after object initialization (this is done once in constructor) */
    void DataNormalization(void);

    /* get data function */
    const int GetNormalizedFx(void);
    const int GetNormalizedFy(void);
    const int GetNormalizedFz(void);

    const int GetNormalizedMx(void);
    const int GetNormalizedMy(void);
    const int GetNormalizedMz(void);

private:
    /* serial port parameters setting */
    void SetComportAttribute(int file_descriptor);

    /* back ground */
    void BGReadDataFromForceSensor(void);

    /* main function to open port */
    void Connect(std::string port_name);

    /* indication of the state of connection based on file descriptor */
    bool is_comport_connected_;
    bool is_comport_setup_;
    int file_descriptor_;

    /* control how fast we fetch data from port */
    const int sampling_time_;

    /* these data can be found in manual */
    const int force_resolution_;
    const int torque_resolution_;
    const int data_zero_point_;

    /* data buffer used when read from port */
    char data_buffer_[256];

    int tick_;

    /* raw data refreshed by back ground runner every sampling time */
    unsigned short raw_fx_, raw_fy_, raw_fz_;
    unsigned short raw_mx_, raw_my_, raw_mz_;

    /* number of samples used to calculate the mean value of data */
    const int correction_data_number_;
    int mean_fx_, mean_fy_, mean_fz_;
    int mean_mx_, mean_my_, mean_mz_;

    std::thread *back_ground_runner_;
    bool is_back_ground_deleted_;

    /* ros publisher */
    void PublishDataToTopic(void);

    ros::NodeHandle node_handler_;
    ros::Publisher pubber_;
    std_msgs::Int16MultiArray msg_;
};
#endif