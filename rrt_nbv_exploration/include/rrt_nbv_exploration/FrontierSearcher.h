#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rrt_nbv_exploration_msgs/Tree.h"
#include "rrt_nbv_exploration_msgs/Node.h"
#include "nanoflann.hpp"

/**
 * @brief The FrontierSearcher class provides functionality for constructing a KD tree and executing nearest neighbour
 * search as well as radius search. It always treats all node points as if they were on the same height (z=0)
 */
class FrontierSearcher {
public:
	/**
	 * @brief Constructor that initializes the KDTreeSingleIndexAdaptor
	 */
	FrontierSearcher();
	/**
	 * @brief Initializes the kd-tree index and update the tree
	 * @param RRT with frontiers that act as nodes for the kd tree
	 */
	void initialize(rrt_nbv_exploration_msgs::Tree &rrt);
	/**
	 * @brief Rebuilds the kd-tree if necessary
	 * @param RRT with frontiers that act as nodes for the kd tree
	 */
	void rebuildIndex(rrt_nbv_exploration_msgs::Tree &rrt);
	/**
	 * @brief Find the nearest neighbour in the current kd-tree to the randomly sampled point given as parameter
	 * @param Randomly sampled 3D point
	 * @param Link to minimum squared distance calculated between a node in the tree and the randomly sampled point
	 * @param Link to the nearest node in the frontiers from the randomly sampled point
	 */
	void findNearestNeighbour(geometry_msgs::Point rand_sample,
			double &min_distance, int &nearest_node);
	/**
	 * @brief Finds all frontiers in the current kd-tree in the given (squared) radius around a provided point, radius is squared because Euclidean distance is squared as well
	 * @param 3D Point
	 * @param Search radius (needs to be squared)
	 * @return List of the indices of all nodes in this radius
	 */
	std::vector<int> searchInRadius(geometry_msgs::Point node_position,
			double radius);

private:
	/**
	 * @brief The rrt_adaptor struct serves as an adaptor for nanoflann, using the frontiers in rrt
	 */
	struct frontier_adaptor {
		rrt_nbv_exploration_msgs::Tree rrt;
		inline size_t kdtree_get_point_count() const {
			return rrt.frontier_counter;
		}
		inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
			if (dim == 0)
				return rrt.frontiers[idx].position.x;
			else if (dim == 1)
				return rrt.frontiers[idx].position.y;
			else
				return 0; //rrt.nodes[idx].position.z;
		}
		template<class BBOX>
		bool kdtree_get_bbox(BBOX& /* bb */) const {
			return false;
		}
	};
	typedef nanoflann::KDTreeSingleIndexAdaptor<
			nanoflann::L2_Simple_Adaptor<double, frontier_adaptor>, frontier_adaptor, 3> kd_tree_adaptor;
	frontier_adaptor _frontier_kd_adaptor;
	kd_tree_adaptor _kd_frontier_index;
};
