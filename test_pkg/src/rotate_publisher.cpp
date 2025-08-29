#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "rotate_publisher");
    ros::NodeHandle nh;

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Rate loop_rate(10);

    //Define msg parameters 
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = 1.0;   // 1m/s
    twist_msg.linear.y = 0.0;   // 
    twist_msg.linear.z = 0.0;   // 
    twist_msg.angular.x = 0.0;  // 
    twist_msg.angular.y = 0.0;  // 
    twist_msg.angular.z = 1.0;  // 1 rad/s (inverse)

    ROS_INFO("Rotate Publisher Started. Publishing angular velocity...");

    
    while (ros::ok()) {
        cmd_vel_pub.publish(twist_msg);

        ROS_INFO("Publishing: linear.x = %.2f, angular.z = %.2f", twist_msg.linear.x, twist_msg.angular.z);
        loop_rate.sleep();
    }

    return 0;
}

