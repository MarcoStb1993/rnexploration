#include "rrt_nbv_exploration/GainCalculator.h"

namespace rrt_nbv_exploration
{

GainCalculator::GainCalculator()
{
    ros::NodeHandle private_nh("~");
    private_nh.param("sensor_range", _sensor_range, 5.0);
    private_nh.param("visualize_gain_calculation", _visualize_gain_calculation, false);
    ros::NodeHandle nh("rne");
    _raycast_visualization = nh.advertise<visualization_msgs::Marker>("raycast_visualization", 1000);
}

void GainCalculator::calculate_gain(rrt_nbv_exploration_msgs::Node &node, boost::shared_ptr<octomap::OcTree> octree)
{
    visualization_msgs::Marker _node_points;
    _node_points.header.frame_id = "/map";
    _node_points.ns = "raycast_visualization";
    _node_points.id = 0;
    _node_points.action = visualization_msgs::Marker::ADD;
    _node_points.pose.orientation.w = 1.0;
    _node_points.type = visualization_msgs::Marker::SPHERE_LIST;
    _node_points.scale.x = 0.1;
    _node_points.scale.y = 0.1;
    _node_points.scale.z = 0.1;
    _node_points.color.a = 1.0f;
    _node_points.header.stamp = ros::Time::now();

    node.gain = 0;
    octomap::point3d start_point(node.position.x, node.position.y, node.position.z);
    std::vector<octomap::point3d> end_points;
    end_points.push_back(octomap::point3d(node.position.x + _sensor_range, node.position.y, node.position.z));
    end_points.push_back(octomap::point3d(node.position.x - _sensor_range, node.position.y, node.position.z));
    end_points.push_back(octomap::point3d(node.position.x, node.position.y + _sensor_range, node.position.z));
    end_points.push_back(octomap::point3d(node.position.x, node.position.y - _sensor_range, node.position.z));

    bool obstacle = false;
    for(auto it : end_points)
    {
        octomap::KeyRay keyray;
        if(octree->computeRayKeys(start_point, it, keyray))
        {
            int raygain = 0;
            for(auto iterator : keyray)
            {
                octomap::point3d coords = octree->keyToCoord(iterator);
                octomap::OcTreeNode* ocnode = octree->search(iterator);
                geometry_msgs::Point point;
                point.x = coords.x();
                point.y = coords.y();
                point.z = coords.z();
                std_msgs::ColorRGBA color;
                color.r = 0.0f;
                color.g = 0.0f;
                color.b = 1.0f;
                color.a = 1.0f;
                _node_points.points.push_back(point);
                if(ocnode!=NULL){
                    bool occupied = octree->isNodeOccupied(ocnode);
                    if(occupied) {
                      color.r = 1.0f;
                      color.b = 0.0f;
                      obstacle = true;
                    }
                    else {
                        color.g = 1.0f;
                        color.b = 0.0f;
                    }
                }
                else {
                    raygain++;
                }
                _node_points.colors.push_back(color);

            }
            node.gain += (float) raygain;
        }
        else {
            ROS_INFO("out of bounds");
        }
    }
    ROS_INFO("Gain: %i", (int)node.gain);
    if(_visualize_gain_calculation)
    {
    _raycast_visualization.publish(_node_points);
    }
}
}
