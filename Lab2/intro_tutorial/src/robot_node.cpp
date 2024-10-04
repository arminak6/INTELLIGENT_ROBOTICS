#include "ros/ros.h"
#include "intro_tutorial/srv1.h"
#include "intro_tutorial/msg1.h"

std::string room_names[5] = {"IAS-Lab", "Living Room", "Kitchen", "Bedroom", "Office"};

bool handleServiceRequest(intro_tutorial::srv1::Request &req, intro_tutorial::srv1::Response &res)
{
    res.response_header.stamp = ros::Time::now();
    res.response_header.frame_id = req.request_header.frame_id;

    res.message.battery_level = (float)(rand() % 101) / 100.0;
    res.message.room_id = rand() % 5;
    res.message.room_name = room_names[res.message.room_id];


    ROS_INFO("Request received from charging station ID: %d", req.charging_station_id);
    ROS_INFO("Response: Room ID: %d, Room Name: %s, Battery Level: %.2f",
             res.message.room_id, res.message.room_name.c_str(), res.message.battery_level);

    return true;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_node");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("robot_service", handleServiceRequest);
    ROS_INFO("Ready to send data");
    ros::spin();
    return 0;
}