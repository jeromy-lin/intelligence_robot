#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// define MoveBaseAction action client
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Navigation to Gaol
bool navigateToGoal(MoveBaseClient &ac, double x, double y, double w) {
    move_base_msgs::MoveBaseGoal goal;

    // Define target pose (Map Frame)
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = w;

    ROS_INFO("Sending goal: x=%f, y=%f, w=%f", x, y, w);
    ac.sendGoal(goal);

    // action client
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Goal reached successfully!");
        return true;
    } else {
        ROS_WARN("Failed to reach the goal.");
        return false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_goal_navigation");

    // Create Action, and client connect to move_base
    MoveBaseClient ac("move_base", true);

    ROS_INFO("Waiting for move_base action server...");
    ac.waitForServer();
    ROS_INFO("Connected to move_base server");

    // 請同學依照topic 指定四個目標點並將pose進行更改
    std::vector<std::tuple<double, double, double>> goals = {
	    {1.35, -0.58, 0.113},     // 目標點 1
	    {1.61, 1.2, 0.825},       // 目標點 2
	    {-1.67, 1.43, -0.878},    // 目標點 3
	    {-1.91, -0.832, -0.491}  //  目標點 4
    };

    // 依序導航至各目標點
    for (size_t i = 0; i < goals.size(); ++i) {
        double x = std::get<0>(goals[i]);
        double y = std::get<1>(goals[i]);
        double w = std::get<2>(goals[i]);

        ROS_INFO("Navigating to goal %lu/%lu", i + 1, goals.size());
        if (!navigateToGoal(ac, x, y, w)) {
            ROS_WARN("Skipping to the next goal...");
        }
    }

    ROS_INFO("Multi-goal navigation completed!");
    return 0;
}

