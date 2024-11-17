#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <apriltag_ros/AprilTagDetectionArray.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <tf/transform_listener.h>


void detectionCallbackTF(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  std::string target_frame = "base_link";
  std::string source_frame = msg->header.frame_id;

  while(!listener.canTransform(target_frame, source_frame, ros::Time(0)))
    ros::Duration(0.5).sleep();

  //Transform available
  geometry_msgs::PoseStamped pos_in;
  geometry_msgs::PoseStamped pos_out;

  for(int i = 0; i < msg->detections.size(); ++i)
  {
    pos_in.header.frame_id = msg->detections.at(i).pose.header.frame_id;
    pos_in.pose.position.x = msg->detections.at(i).pose.pose.pose.position.x;
    pos_in.pose.position.y = msg->detections.at(i).pose.pose.pose.position.y;
    pos_in.pose.position.z = msg->detections.at(i).pose.pose.pose.position.z;
    pos_in.pose.orientation.x = msg->detections.at(i).pose.pose.pose.orientation.x;
    pos_in.pose.orientation.y = msg->detections.at(i).pose.pose.pose.orientation.y;
    pos_in.pose.orientation.z = msg->detections.at(i).pose.pose.pose.orientation.z;
    pos_in.pose.orientation.w = msg->detections.at(i).pose.pose.pose.orientation.w;
  
    listener.transformPose(target_frame, pos_in, pos_out);

    ROS_INFO_STREAM("Obj with ID: " << msg->detections.at(i).id[0]);
    ROS_INFO_STREAM("Original pose\n" << pos_in);
    ROS_INFO_STREAM("Transformed pose\n" << pos_out);
  }
}

void detectionCallbackTF2(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg)
{
  std::string target_frame = "base_link";
  std::string source_frame = msg->header.frame_id;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  while(!tfBuffer.canTransform(target_frame, source_frame, ros::Time(0)))
    ros::Duration(0.5).sleep();

  geometry_msgs::TransformStamped transformed = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
  
  //Transform available
  geometry_msgs::PoseStamped pos_in;
  geometry_msgs::PoseStamped pos_out;

  for(int i = 0; i < msg->detections.size(); ++i)
  {
    pos_in.header.frame_id = msg->detections.at(i).pose.header.frame_id;
    pos_in.pose.position.x = msg->detections.at(i).pose.pose.pose.position.x;
    pos_in.pose.position.y = msg->detections.at(i).pose.pose.pose.position.y;
    pos_in.pose.position.z = msg->detections.at(i).pose.pose.pose.position.z;
    pos_in.pose.orientation.x = msg->detections.at(i).pose.pose.pose.orientation.x;
    pos_in.pose.orientation.y = msg->detections.at(i).pose.pose.pose.orientation.y;
    pos_in.pose.orientation.z = msg->detections.at(i).pose.pose.pose.orientation.z;
    pos_in.pose.orientation.w = msg->detections.at(i).pose.pose.pose.orientation.w;
  
    tf2::doTransform(pos_in, pos_out, transformed);

    ROS_INFO_STREAM("Obj with ID: " << msg->detections.at(i).id[0]);
    ROS_INFO_STREAM("Original pose\n" << pos_in);
    ROS_INFO_STREAM("Transformed pose\n" << pos_out);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ex4_1_B_node");
  ros::NodeHandle nh;
  ROS_INFO_STREAM("ex4_1_B_node starts");

  ros::Subscriber tag_subscriber;
  
  char cmd;
  ROS_INFO_STREAM("Press a/A to use tf; b/B to use tf2");
  std::cin >> cmd;
  
  cmd = tolower(cmd);
  if(cmd == 'a')
    tag_subscriber = nh.subscribe("tag_detections", 1000, detectionCallbackTF);
  else if(cmd == 'b')
    tag_subscriber = nh.subscribe("tag_detections", 1000, detectionCallbackTF2);
  else
    ROS_INFO_STREAM("ERROR: try again");

  ros::spin();
  return 0;
}