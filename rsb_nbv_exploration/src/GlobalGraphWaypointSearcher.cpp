#include <rsb_nbv_exploration/GlobalGraphWaypointSearcher.h>

GlobalGraphWaypointSearcher::GlobalGraphWaypointSearcher() :
		_connection_kd_tree_index(3, _connection_kd_adaptor,
				nanoflann::KDTreeSingleIndexAdaptorParams(20)) {
}

void GlobalGraphWaypointSearcher::initialize(
		rsb_nbv_exploration_msgs::GlobalConnection &connection) {
	_connection_kd_adaptor.connection = connection;
	_connection_kd_tree_index.buildIndex();
}

void GlobalGraphWaypointSearcher::rebuildIndex(
		rsb_nbv_exploration_msgs::GlobalConnection &connection) {
	initialize(connection);
}

void GlobalGraphWaypointSearcher::findNearestNeighbour(
		geometry_msgs::Point point, double &min_distance, int &nearest_node) {
	const size_t num_results = 1;
	size_t ret_index;
	nanoflann::KNNResultSet<double> resultSet(num_results);
	resultSet.init(&ret_index, &min_distance);
	double point_array[3] = { point.x, point.y, 0 }; //point.z};
	_connection_kd_tree_index.findNeighbors(resultSet, &point_array[0],
			nanoflann::SearchParams(10));
	nearest_node = ret_index;
}

std::vector<std::pair<int, double>> GlobalGraphWaypointSearcher::searchInRadius(
		geometry_msgs::Point position, double radius) {
	const double search_radius = static_cast<double>(radius);
	std::vector<std::pair<size_t, double> > ret_matches;
	nanoflann::SearchParams params;
	double waypoint_position_point[3] = { position.x, position.y, 0 }; //node_position.z};
	const size_t nMatches = _connection_kd_tree_index.radiusSearch(
			&waypoint_position_point[0], search_radius, ret_matches, params);
	std::vector<std::pair<int, double>> nodes_in_radius;
	for (auto it : ret_matches) {
		nodes_in_radius.push_back(std::make_pair((int) it.first, it.second));
	}
	return nodes_in_radius;
}
