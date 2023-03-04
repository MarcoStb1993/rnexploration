#include "ros/ros.h"
#include "rsb_nbv_exploration_msgs/GlobalConnection.h"

#include "../../../rsb_nbv_exploration/include/rsb_nbv_exploration/nanoflann.hpp"

/**
 * @brief The GlobalGraphWaypointSearcher class provides functionality for constructing a KD tree and executing
 * nearest neighbor search as well as radius search in the waypoints of a global connection. It always treats
 * all waypoints as if they were on the same height (z=0)
 */
class GlobalGraphWaypointSearcher {
public:
	/**
	 * @brief Constructor that initializes the KDTreeSingleIndexAdaptor
	 */
	GlobalGraphWaypointSearcher();
	/**
	 * @brief Initializes the kd-tree index and update the tree
	 * @param Global connection with waypoints that act as nodes for the kd tree
	 */
	void initialize(rsb_nbv_exploration_msgs::GlobalConnection &connection);
	/**
	 * @brief Rebuilds the kd-tree if necessary
	 * @param Global connection with waypoints that act as nodes for the kd tree
	 */
	void rebuildIndex(rsb_nbv_exploration_msgs::GlobalConnection &connection);
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
	 * @brief The connection_adaptor struct serves as an adaptor for nanoflann, using the waypoints in a global connection
	 */
	struct connection_adaptor {
		rsb_nbv_exploration_msgs::GlobalConnection connection;
		inline size_t kdtree_get_point_count() const {
			return connection.waypoints.size();
		}
		inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
			if (dim == 0)
				return connection.waypoints[idx].x;
			else if (dim == 1)
				return connection.waypoints[idx].y;
			else
				return 0; //connection.waypoints[idx].z;
		}
		template<class BBOX>
		bool kdtree_get_bbox(BBOX& /* bb */) const {
			return false;
		}
	};
	typedef nanoflann::KDTreeSingleIndexAdaptor<
			nanoflann::L2_Simple_Adaptor<double, connection_adaptor>, connection_adaptor, 3> connection_kd_tree_adaptor;
	connection_adaptor _connection_kd_adaptor;
	connection_kd_tree_adaptor _connection_kd_tree_index;
};
