#include "ros/ros.h"
#include "intro_tutorial/msg1.h"
#include <sstream>
#include <cstdlib> 
#include <ctime>    
#include <string>  
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "robot_node");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<intro_tutorial::msg1>("message", 1000);
    ros::Rate loop_rate(5);
    std::string room_names[5] = {"IAS-Lab", "Living Room", "Kitchen", "Bedroom", "Office"};


    while (ros::ok()){
        intro_tutorial::msg1 msg;
        msg.room_id = rand() % 5;
        msg.battery_level = (float)(rand() % 101) / 100.0;
        msg.room_name = room_names[msg.room_id];
        ROS_INFO("Publishing: Room %s (ID: %d) with Battery %.1f%%", msg.room_name.c_str(), msg.room_id, msg.battery_level);
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    
    return 0;
}