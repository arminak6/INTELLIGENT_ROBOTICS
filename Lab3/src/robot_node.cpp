#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <intro_tutorial/MyActionAction.h>

class MyAction {
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<intro_tutorial::MyActionAction> as_;
        std::string action_name_;
        intro_tutorial::MyActionFeedback feedback_;
        intro_tutorial::MyActionResult result_;
        
    public:
        MyAction(std::string name) :
            as_(nh_, name, boost::bind(&MyAction::executeCB, this, _1), false),
            action_name_(name) {
            as_.start();
        }

        ~MyAction(void) {}

        void executeCB(const intro_tutorial::MyActionGoalConstPtr &goal) {
            ros::Rate rate(1);
            bool success = true;
            int current_battery = 5;
            int target_battery = goal->max_battery_level;
            double per_second = (target_battery - current_battery) / 60.0;

            while (current_battery < goal->max_battery_level) {
                if (as_.isPreemptRequested() || !ros::ok()) {
                    ROS_INFO("%s: Preempted", action_name_.c_str());
                    as_.setPreempted();
                    success = false;
                    break;
                }

                current_battery += per_second;

                if (current_battery > target_battery) {
                    current_battery = target_battery;
                }

                feedback_.current_battery_level = current_battery;
                as_.publishFeedback(feedback_);

                rate.sleep();
            }

            if (success) {
                result_.charging_complete = true;
                ROS_INFO("%s: Succeeded", action_name_.c_str());
                as_.setSucceeded(result_);
            }
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_node");
    MyAction my_action("my_action");
    ros::spin();
    return 0;
}
