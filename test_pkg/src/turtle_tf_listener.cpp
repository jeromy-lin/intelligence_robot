#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtle_tf_listener");
    ros::NodeHandle nh;

    tf::TransformListener listener;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    ros::Rate rate(10.0);
    while (nh.ok()) {
        tf::StampedTransform transform;
        try {
            listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
        } catch (tf::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.1).sleep();
            continue;
        }

        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
        vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
        cmd_vel_pub.publish(vel_msg);

        rate.sleep();
    }
    return 0;
}

