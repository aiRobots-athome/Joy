#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

class Speech
{
public:
    Speech();
    ~Speech(){};

    std::string GetSubData(void);
    std::string GetPubData(void);

    void SetPubData(std::string str);
    void DeleteThread(void);

private:
    /* Listener */
    ros::NodeHandle n;
    ros::Subscriber subber;

    std::string receive_str;

    void chatterCallback(const std_msgs::String::ConstPtr &msg);
    void Sub(void);

    /* Talker */
    ros::Publisher pubber;
    void Pub(std::string str);

private:
    std::string Pub_str;
    std::string Sub_str;

    std::thread *PubThread;
    std::thread *SubThread;

    bool is_deleted;

    void SubFromPython();
    void PubToPython();
    void test();
};