#include <ros/ros.h>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

tf2_ros::Buffer buffer;
geometry_msgs::TransformStamped transform;

void detectionCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg)
{
  ROS_INFO_STREAM("Following the Objects ID detected: ");
  for (int i = 0; i < msg->detections.size(); ++i)
    ROS_INFO_STREAM("Obj ID: " << msg->detections.at(i).id[0]);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ex4_1_A_node");
  ros::NodeHandle nh;
  ROS_INFO_STREAM("ex4_1_A_node starts");

  ros::Subscriber tag_subscriber = nh.subscribe("tag_detections", 1000, detectionCallback);

  ros::spin();
  return 0;
}