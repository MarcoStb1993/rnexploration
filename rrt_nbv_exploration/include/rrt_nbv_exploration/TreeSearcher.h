#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rrt_nbv_exploration_msgs/Tree.h"
#include "rrt_nbv_exploration_msgs/Node.h"
#include "nanoflann.hpp"

/**
 * @brief The TreeSearcher class provides functionality for constructing a KD tree and executing nearest neighbour search as well as radius search.
 */
class TreeSearcher {
public:
    /**
     * @brief Constructor that initializes the KDTreeSingleIndexAdaptor
     */
    TreeSearcher();
    /**
     * @brief Initializes the kd-tree index and update the tree
     * @param RRT with nodes that act as nodes for the kd tree
     */
    void initialize(rrt_nbv_exploration_msgs::Tree &rrt);
    /**
     * @brief Rebuilds the kd-tree if necessary
     * @param RRT with nodes that act as nodes for the kd tree
     */
    void rebuildIndex(rrt_nbv_exploration_msgs::Tree &rrt);
    /**
     * @brief Find the nearest neighbour in the current kd-tree to the randomly sampled point given as parameter
     * @param Randomly sampled 3D point
     * @param Link to minimum distance calculated between a node in the tree and the randomly sampled point
     * @param Link to the nearest node in the tree from the randomly sampled point
     */
    void find_nearest_neighbour(geometry_msgs::Point rand_sample, double &min_distance, int &nearest_node);
    /**
     * @brief Finds all nodes in the current kd-tree in the given (squared) radius around a provided point, radius is squared because Euclidean distance is squared as well
     * @param 3D Point
     * @param Search radius (needs to be squared)
     * @return List of the indices of all nodes in this radius
     */
    std::vector<int> search_in_radius(geometry_msgs::Point node_position, double radius);

private:
    /**
     * @brief The rrt_adaptor struct serves as an adaptor for nanoflann, using the nodes in rrt
     */
    struct rrt_adaptor {
        rrt_nbv_exploration_msgs::Tree rrt;
        inline size_t kdtree_get_point_count() const { return rrt.node_counter; }
        inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
            if(dim==0) return rrt.nodes[idx].position.x;
            else if(dim==1) return rrt.nodes[idx].position.y;
            else return rrt.nodes[idx].position.z;
        }
        template <class BBOX>
        bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
    };
    typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, rrt_adaptor>,rrt_adaptor, 3> kd_tree_adaptor;
    rrt_adaptor _rrt_kd_adaptor;
    kd_tree_adaptor _kd_tree_index;
};
