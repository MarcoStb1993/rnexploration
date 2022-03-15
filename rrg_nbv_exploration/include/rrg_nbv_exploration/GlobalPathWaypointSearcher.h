#include "ros/ros.h"
#include "rrg_nbv_exploration_msgs/GlobalPath.h"

#include <rrg_nbv_exploration/nanoflann.hpp>

/**
 * @brief The GlobalPathWaypointSearcher class provides functionality for constructing a KD tree and executing
 * nearest neighbor search as well as radius search in the waypoints of a global path. It always treats
 * all waypoints as if they were on the same height (z=0)
 */
class GlobalPathWaypointSearcher {
public:
	/**
	 * @brief Constructor that initializes the KDTreeSingleIndexAdaptor
	 */
	GlobalPathWaypointSearcher();
	/**
	 * @brief Initializes the kd-tree index and update the tree
	 * @param Global path with waypoints that act as nodes for the kd tree
	 */
	void initialize(rrg_nbv_exploration_msgs::GlobalPath &path);
	/**
	 * @brief Rebuilds the kd-tree if necessary
	 * @param Global path with waypoints that act as nodes for the kd tree
	 */
	void rebuildIndex(rrg_nbv_exploration_msgs::GlobalPath &path);
	/**
	 * @brief Find the nearest neighbor in the current kd-tree to the point given as parameter
	 * @param 3D point
	 * @param Reference to minimum squared distance calculated between a waypoint in the tree and the point
	 * @param Reference to the nearest waypoint index in the tree from the given point
	 */
	void findNearestNeighbour(geometry_msgs::Point point, double &min_distance,
			int &nearest_node);
	/**
	 * @brief Finds all waypoints in the current kd-tree in the given (squared) radius around a provided
	 * point, radius is squared because Euclidean distance is squared as well
	 * @param 3D Point
	 * @param Search radius (needs to be squared)
	 * @return List of the indices of all waypoints in this radius and their respective squared distance
	 * sorted ascending by squared distance
	 */
	std::vector<std::pair<int, double>> searchInRadius(
			geometry_msgs::Point position, double radius);

private:
	/**
	 * @brief The path_adaptor struct serves as an adaptor for nanoflann, using the waypoints in a global path
	 */
	struct path_adaptor {
		rrg_nbv_exploration_msgs::GlobalPath path;
		inline size_t kdtree_get_point_count() const {
			return path.waypoints.size();
		}
		inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
			if (dim == 0)
				return path.waypoints[idx].x;
			else if (dim == 1)
				return path.waypoints[idx].y;
			else
				return 0; //path.waypoints[idx].z;
		}
		template<class BBOX>
		bool kdtree_get_bbox(BBOX& /* bb */) const {
			return false;
		}
	};
	typedef nanoflann::KDTreeSingleIndexAdaptor<
			nanoflann::L2_Simple_Adaptor<double, path_adaptor>, path_adaptor, 3> path_kd_tree_adaptor;
	path_adaptor _path_kd_adaptor;
	path_kd_tree_adaptor _path_kd_tree_index;
};
