#include "drivers_ax12/ax12_server.h"

void Ax12Server::execute_goal_cb(GoalHandle goal_handle) {
    if(!goal_handle.isValid()) {
        ROS_ERROR("Ax12 received invalid goal !");
        goal_handle.setRejected();
        return;
    }

    goal_handle.setAccepted();

    current_goals.insert(std::make_pair(goal_handle.getGoalID().id.c_str(), goal_handle));


    ROS_INFO("%s", goal_handle.getGoalID().id.c_str());
}
