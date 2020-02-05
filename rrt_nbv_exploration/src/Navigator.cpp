#include "rrt_nbv_exploration/Navigator.h"

Navigator::Navigator()  :
     _move_base_client("move_base", true)
{
    while(!_move_base_client.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    _current_goal_reached = true;
}

void Navigator::publish_nav_goal(rrt_nbv_exploration_msgs::Node &new_goal)
{
    if(!_current_goal_reached)
    {
        cancel_nav_goal();
    }
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = new_goal.position.x;
    goal.target_pose.pose.position.y = new_goal.position.y;
    goal.target_pose.pose.position.z = new_goal.position.z;
    goal.target_pose.pose.orientation.w = 1.0;
    _move_base_client.sendGoal(goal);
    _current_goal_reached = false;
}

void Navigator::cancel_nav_goal()
{
    _move_base_client.cancelGoal();
}

void Navigator::check_current_goal_status(rrt_nbv_exploration_msgs::Node &node)
{
    if(!_current_goal_reached)
    {
        //ROS_INFO("Goal status: %s", _move_base_client.getState().toString().c_str());
        switch (_move_base_client.getState().state_) {
        case actionlib::SimpleClientGoalState::PENDING:
            node.status = rrt_nbv_exploration_msgs::Node::ACTIVE;
            break;
        case actionlib::SimpleClientGoalState::ACTIVE:
            node.status = rrt_nbv_exploration_msgs::Node::ACTIVE;
            break;
        case actionlib::SimpleClientGoalState::SUCCEEDED:
            node.status = rrt_nbv_exploration_msgs::Node::EXPLORED;
            _current_goal_reached = true;
            break;
        case actionlib::SimpleClientGoalState::ABORTED:
            node.status = rrt_nbv_exploration_msgs::Node::FAILED;
            _current_goal_reached = true;
            break;
        default:
            node.status = rrt_nbv_exploration_msgs::Node::ABORTED;
            _current_goal_reached = true;
            break;
        }
    }
}
