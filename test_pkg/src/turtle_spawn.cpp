#include <ros/ros.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtle_spawner");
    ros::NodeHandle nh;

    ros::service::waitForService("/spawn");
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("/spawn");

    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = 4.0;
    spawn_srv.request.y = 2.0;
    spawn_srv.request.theta = 0.0;
    spawn_srv.request.name = "turtle2";

    if (spawn_client.call(spawn_srv)) {
        ROS_INFO("Spawned turtle2 at (4, 2) with orientation 0");
    } else {
        ROS_ERROR("Failed to spawn turtle2");
    }

    return 0;
}

