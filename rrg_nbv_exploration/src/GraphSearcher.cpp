#include <rrg_nbv_exploration/GraphSearcher.h>

GraphSearcher::GraphSearcher()    :
    _kd_tree_index(3, _rrt_kd_adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(20))
{
}

void GraphSearcher::initialize(rrg_nbv_exploration_msgs::Graph &prm)
{
    _rrt_kd_adaptor.prm = prm;
    _kd_tree_index.buildIndex();
}

void GraphSearcher::rebuildIndex(rrg_nbv_exploration_msgs::Graph &prm)
{
    initialize(prm);
}

void GraphSearcher::findNearestNeighbour(geometry_msgs::Point rand_sample, double &min_distance, int &nearest_node)
{
    const size_t num_results = 1;
    size_t ret_index;
    nanoflann::KNNResultSet<double> resultSet(num_results);
    resultSet.init(&ret_index, &min_distance );
    double rand_sample_point[3] = {rand_sample.x, rand_sample.y, 0};//rand_sample.z};
    _kd_tree_index.findNeighbors(resultSet, &rand_sample_point[0], nanoflann::SearchParams(10));
    nearest_node = ret_index;
}

std::vector<std::pair<int,double>> GraphSearcher::searchInRadius(geometry_msgs::Point node_position, double radius)
{
    const double search_radius = static_cast<double>(radius);
    std::vector<std::pair<size_t,double> >   ret_matches;
    nanoflann::SearchParams params;
    double node_position_point[3] = {node_position.x, node_position.y, 0};//node_position.z};
    const size_t nMatches = _kd_tree_index.radiusSearch(&node_position_point[0], search_radius, ret_matches, params);
    std::vector<std::pair<int,double>> nodes_in_radius;
    for(auto it : ret_matches)
    {
        nodes_in_radius.push_back(std::make_pair((int)it.first,it.second));
    }
    //nodes_in_radius.erase(nodes_in_radius.begin()); //delete first node from list because it is the starting position for the radius search
    //ROS_INFO("Matches found: %lu", nMatches);
    //ROS_INFO("Nodes in radius %3.3f: ", sqrt(search_radius));
//    for(auto it : nodes_in_radius)
//    {
//        //ROS_INFO("Node %i", it);
//    }
    return nodes_in_radius;
}
