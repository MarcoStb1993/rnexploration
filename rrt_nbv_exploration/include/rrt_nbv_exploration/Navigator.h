#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rrt_nbv_exploration_msgs/Node.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

class Navigator
{
public:
    /**
     * @brief Constructor that starts a SimpleActionClient for a MoveBaseAction
     */
    Navigator();
    /**
     * @brief Publishes the node's position as a new goal to the navigation stack
     * @param A new navigation goal is at the node's position
     */
    void publish_nav_goal(rrt_nbv_exploration_msgs::Node &new_goal);
    /**
     * @brief Cancel ongoing navigation goal
     */
    void cancel_nav_goal();
    /**
     * @brief Checks if the current goal was reached or aborted or is still active and alters the node's status accordingly
     * @param The current navigation goal node
     */
    void check_current_goal_status(rrt_nbv_exploration_msgs::Node &goal);
private:
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    MoveBaseClient _move_base_client;
    /**
     * @brief States if the current goal was already reached or aborted (true) or if it is still active (false)
     */
    bool _current_goal_reached;
};
