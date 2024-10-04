#include "ros/ros.h"
#include "intro_tutorial/srv1.h"
#include <cstdlib>

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "charging_station_client");
    if (argc != 2)
    {
        ROS_INFO("usage: charging_station_client charging_station_id");
        return 1;
    }
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<intro_tutorial::srv1>("robot_service");
    intro_tutorial::srv1 srv;

    srv.request.request_header.stamp = ros::Time::now();
    srv.request.charging_station_id = atoi(argv[1]);
    if (client.call(srv))
    {
        ROS_INFO("Response received:");
        ROS_INFO("Response: Room ID: %d, Room Name: %s, Battery Level: %.2f",
                srv.response.message.room_id, 
                srv.response.message.room_name.c_str(), 
                srv.response.message.battery_level);
    }
    else
    {
        ROS_ERROR("Failed to call service robot_service");
        return 1;
    }

    return 0;
}