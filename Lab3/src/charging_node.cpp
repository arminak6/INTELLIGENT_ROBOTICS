#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <intro_tutorial/MyActionAction.h>
#include <intro_tutorial/MyActionResult.h>  

int main(int argc, char** argv) {
    ros::init(argc, argv, "charging_node");

    actionlib::SimpleActionClient<intro_tutorial::MyActionAction> ac("charging_action", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); 
    ROS_INFO("Action server started, sending goal.");


    intro_tutorial::MyActionGoal goal;
    goal.max_battery_level = 80;  

    ac.sendGoal(goal,
        [](const actionlib::SimpleClientGoalState& state, const intro_tutorial::MyActionResultConstPtr& result) {
            ROS_INFO("Charging completed with state: %s", state.toString().c_str());
            ROS_INFO("Charging complete: %s", result->charging_complete ? "Yes" : "No");
        },
        []() {
            ROS_INFO("Goal became active.");
        },
        [](const intro_tutorial::MyActionFeedbackConstPtr& feedback) {
            ROS_INFO("Current battery level: %d%%", feedback->current_battery_level);
        }
    );


    bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
    if (finished_before_timeout){

        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());

        intro_tutorial::MyActionResult result = *ac.getResult();
        ROS_INFO("Charging complete: %s", result.charging_complete ? "Yes" : "No");

    }else{
        ROS_INFO("Action did not finish before the time out.");
    }
    


    ros::spin();
    return 0;
}
