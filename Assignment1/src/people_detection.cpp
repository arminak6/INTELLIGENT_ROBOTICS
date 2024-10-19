#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <cmath>

// Structure to represent a 2D point
struct Point {
    float x;
    float y;
};

// Structure to represent a person (the centroid of a cluster)
struct Person {
    float x;
    float y;
};

// Function to calculate Euclidean distance between two points
float distance(const Point& p1, const Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

// Callback function to process laser scan data
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::vector<float> ranges = scan->ranges;
    float angle_min = scan->angle_min;
    float angle_increment = scan->angle_increment;
    float range_min = scan->range_min;
    float range_max = scan->range_max;

    std::vector<Point> points;

    // Convert LaserScan polar coordinates to Cartesian points
    for (int i = 0; i < ranges.size(); i++) {
        float range = ranges[i];
        if (range < range_min || range > range_max) {
            continue;  // Ignore invalid ranges
        }
        float angle = angle_min + angle_increment * i;
        Point point;
        point.x = range * std::cos(angle);
        point.y = range * std::sin(angle);
        points.push_back(point);
    }

    std::vector<std::vector<Point>> clusters;  // Store clusters of points
    float cluster_threshold = 0.5;  // Threshold distance for clustering points

    // Group points into clusters (representing people)
    for (const auto& point : points) {
        bool found_cluster = false;

        // Check if the point can be added to any existing cluster
        for (auto& cluster : clusters) {
            if (distance(cluster.back(), point) < cluster_threshold) {
                cluster.push_back(point);
                found_cluster = true;
                break;
            }
        }

        // If no suitable cluster is found, create a new cluster
        if (!found_cluster) {
            clusters.push_back({point});
        }
    }

    std::vector<Person> people;

    // For each cluster, calculate the centroid (average position)
    for (const auto& cluster : clusters) {
        if (cluster.size() < 3) {
            // Ignore small clusters, as they may be noise
            continue;
        }

        float sum_x = 0;
        float sum_y = 0;
        for (const auto& point : cluster) {
            sum_x += point.x;
            sum_y += point.y;
        }

        // Calculate the centroid of the cluster
        Person person;
        person.x = sum_x / cluster.size();
        person.y = sum_y / cluster.size();
        people.push_back(person);
    }

    // Output the detected people's positions
    for (size_t i = 0; i < people.size(); ++i) {
        ROS_INFO("Person %lu: (%.2f, %.2f)", i + 1, people[i].x, people[i].y);
    }

    // Print the number of people detected
    ROS_INFO("Total people detected: %lu", people.size());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "people_detector");
    ros::NodeHandle n;

    // Subscribe to the /scan topic
    ros::Subscriber sub = n.subscribe("/scan", 1000, scanCallback);

    ROS_INFO("Ready to detect people from laser scan data...");

    // Spin and process callbacks
    ros::spin();

    return 0;
}
