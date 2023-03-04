#include "ros/ros.h"
#include "rsb_nbv_exploration_msgs/GlobalGraph.h"

#include "../../../rsb_nbv_exploration/include/rsb_nbv_exploration/nanoflann.hpp"

/**
 * @brief The GlobalGraphSearcher class provides functionality for constructing a KD tree and executing
 * nearest neighbor search as well as radius search in the global graph. It always treats all targets
 * points as if they were on the same height (z=0)
 */
class GlobalGraphSearcher {
public:
	/**
	 * @brief Constructor that initializes the KDTreeSingleIndexAdaptor
	 */
	GlobalGraphSearcher();
	/**
	 * @brief Initializes the kd-tree index and update the tree
	 * @param GG with targets that act as nodes for the kd tree
	 */
	void initialize(rsb_nbv_exploration_msgs::GlobalGraph &gg);
	/**
	 * @brief Rebuilds the kd-tree if necessary
	 * @param GG with targets that act as nodes for the kd tree
	 */
	void rebuildIndex(rsb_nbv_exploration_msgs::GlobalGraph &gg);
	/**
	 * @brief Find the nearest neighbor in the current kd-tree to the point given as parameter
	 * @param 3D point
	 * @param Reference to minimum squared distance calculated between a target in the tree and the point
	 * @param Reference to the nearest target in the tree from the given point
	 */
	void findNearestNeighbour(geometry_msgs::Point point, double &min_distance,
			int &nearest_node);
	/**
	 * @brief Finds all targets in the current kd-tree in the given (squared) radius around a provided
	 * point, radius is squared because Euclidean distance is squared as well
	 * @param 3D Point
	 * @param Search radius (needs to be squared)
	 * @return List of the indices of all targets in this radius and their respective squared distance
	 * sorted ascending by squared distance
	 */
	std::vector<std::pair<int, double>> searchInRadius(
			geometry_msgs::Point position, double radius);

private:
	/**
	 * @brief The gg_adaptor struct serves as an adaptor for nanoflann, using the global targets in the GG
	 */
	struct gg_adaptor {
		rsb_nbv_exploration_msgs::GlobalGraph gg;
		inline size_t kdtree_get_point_count() const {
			return gg.targets_counter;
		}
		inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
			if (dim == 0)
				return gg.targets[idx].viewpoint.x;
			else if (dim == 1)
				return gg.targets[idx].viewpoint.y;
			else
				return 0; //gg.targets[idx].viewpoint.z;
		}
		template<class BBOX>
		bool kdtree_get_bbox(BBOX& /* bb */) const {
			return false;
		}
	};
	typedef nanoflann::KDTreeSingleIndexAdaptor<
			nanoflann::L2_Simple_Adaptor<double, gg_adaptor>, gg_adaptor, 3> gg_kd_tree_adaptor;
	gg_adaptor _gg_kd_adaptor;
	gg_kd_tree_adaptor _gg_kd_tree_index;
};
