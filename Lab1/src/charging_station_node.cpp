#include "ros/ros.h"
#include "intro_tutorial/msg1.h"



void messageCallback(const intro_tutorial::msg1::ConstPtr& msg)
{
  ROS_INFO("Charging Station: Room %s (ID: %d) - Battery %.1f%%", msg->room_name.c_str(), msg->room_id, msg->battery_level);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "charging_station_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("message", 1000, messageCallback);
    ros::spin();

    return 0;
}