#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
#include <cmath>

// define global variable
ros::Publisher g_cmd_vel_pub;
geometry_msgs::Twist g_twist;

double g_safety_distance = 0.5;  // gobal detection distance 
double g_forward_speed = 0.2;   // forward speed
double g_turn_speed = 0.5;      // turn speed
double g_back_speed = -0.1;     // back speed
bool g_obstacle_detected = false; // obstacle detection flag

// laser scan call back function
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    // forward detection distance
    double front_distance = std::min(
        static_cast<double>(*std::min_element(scan->ranges.begin(), scan->ranges.begin() + 10)),
        static_cast<double>(*std::min_element(scan->ranges.end() - 10, scan->ranges.end()))
    );

    // if detection obstacle, flag = true.
    if (front_distance < g_safety_distance) {
        g_obstacle_detected = true;
    }
}

// back forward
void moveBackward() {
    ros::Rate rate(10); // 10 Hz
    g_twist.linear.x = g_back_speed;
    g_twist.angular.z = 0.0;

    // 1 sec
    for (int i = 0; i < 10; ++i) {
        g_cmd_vel_pub.publish(g_twist);
        rate.sleep();
    }
    // stop backward
    g_twist.linear.x = 0.0;
    g_cmd_vel_pub.publish(g_twist);
}

// Rotate 90 degree
void rotate90Degrees() {
    ros::Rate rate(10); // 10 Hz
    g_twist.linear.x = 0.0;
    g_twist.angular.z = g_turn_speed;

    // Rotate 2 sec
    for (int i = 0; i < 20; ++i) {
        g_cmd_vel_pub.publish(g_twist);
        rate.sleep();
    }

    // stop rotate
    g_twist.angular.z = 0.0;
    g_cmd_vel_pub.publish(g_twist);
    g_obstacle_detected = false; // reset obstacle flag
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_follower");
    ros::NodeHandle nh;
    ros::Subscriber laser_sub = nh.subscribe("/scan", 10, laserCallback);
    g_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Rate rate(10); // 10 Hz
    while (ros::ok()) {
        ros::spinOnce();

        if (g_obstacle_detected) {
            // control rule back->rotate
            moveBackward();
            rotate90Degrees();
        } else {
            // control move forward
            g_twist.linear.x = g_forward_speed;
            g_twist.angular.z = 0.0;
            g_cmd_vel_pub.publish(g_twist);
        }

        rate.sleep();
    }

    return 0;
}

