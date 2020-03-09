#include "rrt_nbv_exploration/TreeSearcher.h"

TreeSearcher::TreeSearcher()    :
    _kd_tree_index(3, _rrt_kd_adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(20))
{
}

void TreeSearcher::initialize(rrt_nbv_exploration_msgs::Tree &rrt)
{
    _rrt_kd_adaptor.rrt = rrt;
    _kd_tree_index.buildIndex();
}

void TreeSearcher::rebuildIndex(rrt_nbv_exploration_msgs::Tree &rrt)
{
    initialize(rrt);
}

void TreeSearcher::find_nearest_neighbour(geometry_msgs::Point rand_sample, double &min_distance, int &nearest_node)
{
	//ROS_INFO_STREAM("Find nearest neighbor");
    const size_t num_results = 1;
    size_t ret_index;
    nanoflann::KNNResultSet<double> resultSet(num_results);
    resultSet.init(&ret_index, &min_distance );
    double rand_sample_point[3] = {rand_sample.x, rand_sample.y, rand_sample.z};
    _kd_tree_index.findNeighbors(resultSet, &rand_sample_point[0], nanoflann::SearchParams(10));
    nearest_node = ret_index;
    //ROS_INFO_STREAM("Find nearest neighbor finished");
}

std::vector<int> TreeSearcher::search_in_radius(geometry_msgs::Point node_position, double radius)
{
	//ROS_INFO_STREAM("search in radius");
    const double search_radius = static_cast<double>(radius);
    std::vector<std::pair<size_t,double> >   ret_matches;
    nanoflann::SearchParams params;
    double node_position_point[3] = {node_position.x, node_position.y, node_position.z};
    const size_t nMatches = _kd_tree_index.radiusSearch(&node_position_point[0], search_radius, ret_matches, params);
    std::vector<int> nodes_in_radius;
    for(auto it : ret_matches)
    {
        nodes_in_radius.push_back(it.first);
    }
    //nodes_in_radius.erase(nodes_in_radius.begin()); //delete first node from list because it is the starting position for the radius search
    //ROS_INFO("Matches found: %lu", nMatches);
    //ROS_INFO("Nodes in radius %3.3f: ", sqrt(search_radius));
    for(auto it : nodes_in_radius)
    {
        //ROS_INFO("Node %i", it);
    }
    return nodes_in_radius;
}
