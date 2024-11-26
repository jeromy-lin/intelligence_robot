#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtlebot3_tf_monitor");
    ros::NodeHandle nh;

    // TransformListener mointor TF msg
    tf::TransformListener listener;

    ros::Rate rate(10.0);  //Process rate 10 Hz 

    while (ros::ok()) {
        try {
            // lookup "Frame A" to "Frame B", Here is base_link to base_scan
            tf::StampedTransform transform;
            listener.lookupTransform("odom", "base_footprint", ros::Time(0), transform);

            ROS_INFO("TurtleBot3 'odom' position in 'base_footprint' frame: x = %f, y = %f, z = %f",
                     transform.getOrigin().x(),
                     transform.getOrigin().y(),
                     transform.getOrigin().z());
        }
        catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

