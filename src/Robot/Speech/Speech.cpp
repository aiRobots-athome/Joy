#include "Speech.h"
Speech::Speech()
{
    // Initailize pubber once (ONLY once needed)
    this->pubber = this->n.advertise<std_msgs::String>("Speech_C2Python", 1);

    this->Pub_str = "default_pub";
    this->Sub_str = "default_sub";

    this->PubThread = new std::thread(&Speech::PubToPython, this);
    this->SubThread = new std::thread(&Speech::SubFromPython, this);

    this->is_deleted = false;
}

/* Public Function */
void Speech::DeleteThread()
{
    this->is_deleted = true;
}
std::string Speech::GetPubData()
{
    return this->Pub_str;
}
std::string Speech::GetSubData()
{
    return this->Sub_str;
}
void Speech::SetPubData(std::string str)
{
    this->Pub_str = str;
}

/* Listener */
void Speech::chatterCallback(const std_msgs::String::ConstPtr &msg)
{
    //save data to member var.
    this->receive_str = msg->data.c_str();
    // ROS_INFO("Sub in string : %s",msg->data.c_str());
}

void Speech::Sub(void)
{
    this->subber = this->n.subscribe("Speech_Python2C", 1, &Speech::chatterCallback, this);
    ros::spinOnce();
}

/* Talker */
void Speech::Pub(std::string str)
{
    std_msgs::String msg;
    msg.data = str;
    // ROS_INFO("Pub out string : %s",msg.data.c_str());
    this->pubber.publish(msg);
    ros::spinOnce();
}

/* Private Function */
void Speech::test()
{
    while (ros::ok() && !is_deleted)
    {
        this->Sub();
        std::string str = this->receive_str + "_DONE";
        this->Pub(str);
    }
}
void Speech::SubFromPython()
{
    while (ros::ok() && !is_deleted)
    {
        this->Sub();
        this->Sub_str = this->receive_str;
    }
}
void Speech::PubToPython()
{
    while (ros::ok() && !is_deleted)
    {
        this->Pub(this->Pub_str);
    }
}