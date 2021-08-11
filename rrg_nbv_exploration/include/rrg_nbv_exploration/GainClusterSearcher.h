#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rrg_nbv_exploration_msgs/Graph.h"
#include "rrg_nbv_exploration_msgs/GainCluster.h"

#include <rrg_nbv_exploration/nanoflann.hpp>

/**
 * @brief The GainClusterSearcher class provides functionality for constructing a KD tree and executing nearest neighbour
 * search as well as radius search on gain clusters.
 */
class GainClusterSearcher {
public:
	/**
	 * @brief Constructor that initializes the KDTreeSingleIndexAdaptor
	 */
	GainClusterSearcher();
	/**
	 * @brief Initializes the kd-tree index and update the tree
	 * @param RRG with nodes that have gain clusters for the kd tree
	 */
	void initialize(rrg_nbv_exploration_msgs::Graph &rrg);
	/**
	 * @brief Rebuilds the kd-tree if necessary
	 * @param RRG with nodes that have gain clusters for the kd tree
	 */
	void rebuildIndex(rrg_nbv_exploration_msgs::Graph &rrg);

	/**
	 * @brief Finds all nodes in the current kd-tree in the given (squared) radius around a provided point, radius is squared because Euclidean distance is squared as well
	 * @param 3D Point
	 * @param Search radius (needs to be squared)
	 * @return List of the indices of all gain clusters in this radius
	 */
	std::vector<int> searchInRadius(geometry_msgs::Point cluster_position,
			double squared_radius);

private:
	/**
	 * @brief The rrg_adaptor struct serves as an adaptor for nanoflann, using the gain Cluster in RRG
	 */
	struct gain_cluster_adaptor {
		rrg_nbv_exploration_msgs::Graph rrg;
		inline size_t kdtree_get_point_count() const {
			return rrg.gain_cluster.size();
		}
		inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
			if (dim == 0)
				return rrg.gain_cluster[idx].position.x;
			else if (dim == 1)
				return rrg.gain_cluster[idx].position.y;
			else
				return rrg.gain_cluster[idx].position.z;
		}
		template<class BBOX>
		bool kdtree_get_bbox(BBOX& /* bb */) const {
			return false;
		}
	};
	typedef nanoflann::KDTreeSingleIndexAdaptor<
			nanoflann::L2_Simple_Adaptor<double, gain_cluster_adaptor>,
			gain_cluster_adaptor, 3> kd_tree_adaptor;
	gain_cluster_adaptor _gc_kd_adaptor;
	kd_tree_adaptor _kd_tree_index;
};
