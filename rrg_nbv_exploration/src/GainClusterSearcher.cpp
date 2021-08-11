#include <rrg_nbv_exploration/GainClusterSearcher.h>

GainClusterSearcher::GainClusterSearcher() :
		_kd_tree_index(3, _gc_kd_adaptor,
				nanoflann::KDTreeSingleIndexAdaptorParams(20)) {
}

void GainClusterSearcher::initialize(rrg_nbv_exploration_msgs::Graph &rrg) {
	_gc_kd_adaptor.rrg = rrg;
	_kd_tree_index.buildIndex();
}

void GainClusterSearcher::rebuildIndex(rrg_nbv_exploration_msgs::Graph &rrg) {
	initialize(rrg);
}

std::vector<int> GainClusterSearcher::searchInRadius(
		geometry_msgs::Point node_position, double radius) {
	const double search_radius = static_cast<double>(radius);
	std::vector<std::pair<size_t, double> > ret_matches;
	nanoflann::SearchParams params;
	double node_position_point[3] = { node_position.x, node_position.y,
			node_position.z };
	const size_t nMatches = _kd_tree_index.radiusSearch(&node_position_point[0],
			search_radius, ret_matches, params);
	std::vector<int> nodes_in_radius;
	for (auto it : ret_matches) {
		nodes_in_radius.push_back((int) it.first);
	}
	return nodes_in_radius;
}
