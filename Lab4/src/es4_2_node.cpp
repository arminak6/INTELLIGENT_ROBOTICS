#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf/tf.h>

cv::Mat K;
cv::Mat D;
int counter = 0;

float pose_x = 0.0;
float pose_y = 0.0;
float pose_z = 0.0;

void detectionCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg)
{
    counter++;
    ROS_INFO_STREAM("Counter: " << counter);

    pose_x += msg->detections.at(0).pose.pose.pose.position.x;
    pose_y += msg->detections.at(0).pose.pose.pose.position.y;
    pose_z += msg->detections.at(0).pose.pose.pose.position.z;

    float mean_pose_x = pose_x / (1.0 * counter);
    float mean_pose_y = pose_y / (1.0 * counter);
    float mean_pose_z = pose_z / (1.0 * counter);

    std::vector<cv::Point2f> projected_points;

    std::vector<cv::Point3f> object_points;
    object_points.push_back(cv::Point3f(mean_pose_x, mean_pose_y, mean_pose_z));

    cv::Mat r_vec, t_vec;
    r_vec = (cv::Mat_<float>(3,1) << 0.0, 0.0, 0.0);
    t_vec = (cv::Mat_<float>(3,1) << mean_pose_x, mean_pose_x, mean_pose_x);

    cv::projectPoints(object_points, r_vec, t_vec, K, D, projected_points);
    ROS_INFO_STREAM("Point(u, v): " << projected_points[0].x << "," << projected_points[0].y);

    cv::Mat display = imread("/home/alberto/ROS_ws/student_ws/src/IntelligentRobotics2223/exercise_4_1/config/image_final.png", cv::IMREAD_COLOR);
    circle(display, projected_points[0], 10, cv::Scalar(255, 255, 255), -1);
    cv::resize(display, display, cv::Size(), 0.5, 0.5);

    cv::imshow("Display", display);
    cv::waitKey(50);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "es4_2_node");
    auto nh = std::make_shared<ros::NodeHandle>("~");

    boost::shared_ptr<const sensor_msgs::CameraInfo> camera_info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("kinect/rgb/camera_info", ros::Duration(5.0));

    std::vector<float> tmp_K;
    for (int i = 0; i < camera_info_ptr->K.size(); ++i)
      tmp_K.push_back(camera_info_ptr->K[i]);
    K = cv::Mat(3, 3, CV_32F, tmp_K.data()).clone();

    std::vector<float> tmp_D;
    for (int i = 0; i < camera_info_ptr->D.size(); ++i)
      tmp_D.push_back(camera_info_ptr->D[i]);
    D = cv::Mat(1, 8, CV_32F, tmp_D.data()).clone();

    auto tag_subscriber = nh->subscribe("/tag_detections", 1000, detectionCallback);

    ros::spin();
    return 0;

}