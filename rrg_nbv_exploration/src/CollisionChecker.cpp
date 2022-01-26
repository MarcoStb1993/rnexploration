#include <rrg_nbv_exploration/CollisionChecker.h>

namespace rrg_nbv_exploration {

CollisionChecker::CollisionChecker() {
	ros::NodeHandle private_nh("~");
	private_nh.param("robot_radius", _robot_radius, 1.0);
	private_nh.param("robot_width", _robot_width, 1.0);
	private_nh.param("sensor_max_range", _sensor_range, 5.0);
	private_nh.param("check_init_position", _check_init_position, false);
	private_nh.param("grid_map_resolution", _grid_map_resolution, 0.05);
	std::string occupancy_grid_topic;
	private_nh.param<std::string>("occupancy_grid_topic", occupancy_grid_topic,
			"map");
	private_nh.param("grid_map_occupied", _grid_map_occupied, 100);
	private_nh.param("grid_map_unknown", _grid_map_unknown, -1);
	private_nh.param("inflation_active", _inflation_active, true);
	private_nh.param("move_nodes", _move_nodes, true);
	double min_edge_distance;
	private_nh.param("min_edge_distance", min_edge_distance, 1.0);
	_min_edge_distance_squared = pow(min_edge_distance, 2);
	private_nh.param("max_edge_distance", _max_edge_distance, 1.0);
	_max_edge_distance_squared = pow(_max_edge_distance, 2);
	ros::NodeHandle nh("rne");
	_occupancy_grid_sub = _nh.subscribe(occupancy_grid_topic, 1,
			&CollisionChecker::occupancyGridCallback, this);
	_occupancy_grid_updates_sub = _nh.subscribe(
			occupancy_grid_topic + "_updates", 1,
			&CollisionChecker::occupancyGridUpdatesCallback, this);
	_visualization_pub = nh.advertise<nav_msgs::OccupancyGrid>(
			"rrg_collision_map", 1);
	_rrt_collision_visualization_pub = nh.advertise<
			visualization_msgs::MarkerArray>("rrg_collision_vis", 1000);
	_path_box_distance_thres = 2
			* sqrt(pow(_robot_radius, 2) - pow(_robot_width / 2, 2));
	_init_vis_map = false;
	precalculateCircleLinesOffset();
}

void CollisionChecker::initialize(rrg_nbv_exploration_msgs::Graph &rrg,
		std::shared_ptr<GraphSearcher> graph_searcher,
		std::shared_ptr<GraphPathCalculator> graph_path_calculator) {
	_graph_searcher = std::move(graph_searcher);
	_graph_path_calculator = std::move(graph_path_calculator);
	nav_msgs::OccupancyGrid map = *_occupancy_grid;
	_vis_map.header.stamp = ros::Time::now();
	_vis_map.info.map_load_time = ros::Time::now();
	initVisMap(map);
	if (!_node_edges.markers.empty()) { //remove all visualized markers
		visualization_msgs::Marker node_point;
		node_point.header.frame_id = "/map";
		node_point.header.stamp = ros::Time();
		node_point.ns = "rrt_collision_vis";
		node_point.id = 0;
		node_point.action = visualization_msgs::Marker::DELETEALL;
		_node_points.markers.push_back(node_point);
		_rrt_collision_visualization_pub.publish(_node_points);
	}
	_node_edges.markers.clear();
	_node_points.markers.clear();
	_marker_id = 0;

	initRootNodeAndGraph(map, rrg);
}

void CollisionChecker::initVisMap(const nav_msgs::OccupancyGrid &map) {
	_vis_map.header.frame_id = "map";
	_vis_map.info.resolution = map.info.resolution;
	_vis_map.info.width = map.info.width;
	_vis_map.info.height = map.info.height;
	_vis_map.info.origin = map.info.origin;
	_vis_map.data = std::vector<int8_t>(map.info.width * map.info.height, -1);
	_init_vis_map = true;
}

void CollisionChecker::initRootNodeAndGraph(nav_msgs::OccupancyGrid &map,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	std::vector<int8_t> tmp_vis_map_data = _vis_map.data;
	int node_cost = 0, node_tiles = 0;
	if (_inflation_active
			&& !isCircleInCollision(rrg.nodes[0].position.x,
					rrg.nodes[0].position.y, map, tmp_vis_map_data, node_cost,
					node_tiles)) {
		rrg.nodes[0].radius = inflateCircle(rrg.nodes[0].position.x,
				rrg.nodes[0].position.y, Directions::none, false, map,
				tmp_vis_map_data, node_cost, node_tiles);
		rrg.nodes[0].squared_radius = pow(rrg.nodes[0].radius, 2);
		rrg.nodes[0].traversability_cost = node_cost;
		rrg.nodes[0].traversability_weight = node_tiles;
		rrg.nodes[0].traversability_cost_to_robot =
				rrg.nodes[0].traversability_cost;
		rrg.nodes[0].traversability_weight_to_robot =
				rrg.nodes[0].traversability_weight;
		rrg.largest_node_radius = std::max(rrg.nodes[0].radius,
				rrg.largest_node_radius);
		rrg.highest_traversability_cost_to_robot =
				(double) rrg.nodes[0].traversability_cost
						/ (double) rrg.nodes[0].traversability_weight;
		if (_rrt_collision_visualization_pub.getNumSubscribers() > 0) {
			visualizeNode(rrg.nodes[0]);
			_rrt_collision_visualization_pub.publish(_node_points);
		}
	} else { // no inflation active, do not check collision for root
		rrg.nodes[0].radius = _robot_radius;
		rrg.nodes[0].squared_radius = pow(_robot_radius, 2);
		rrg.largest_node_radius = _robot_radius;
		rrg.nodes[0].traversability_cost = 0;
		rrg.nodes[0].traversability_weight = 0;
		rrg.nodes[0].traversability_cost_to_robot = 0;
		rrg.nodes[0].traversability_weight_to_robot = 0;
		rrg.highest_traversability_cost_to_robot = 0;
	}
	rrg.nodes[0].distance_to_robot = 0;
	rrg.nodes[0].path_to_robot.push_back(0);
	rrg.nodes[0].heading_in = 0;
	rrg.nodes[0].heading_change_to_robot = 0;
	rrg.nodes[0].heading_change_to_robot_best_view = 0;
	rrg.longest_distance_to_robot = 0;
	rrg.largest_heading_change_to_robot_best_view = 0;
	rrg.highest_node_gain = 0;

	_vis_map.data = tmp_vis_map_data;
	_visualization_pub.publish(_vis_map);
}

std::vector<CircleLine> CollisionChecker::calculateCircleLinesOffset(
		double radius) {
	std::vector<CircleLine> circle_lines_offset;
	double half_resolution = _grid_map_resolution / 2;
	double dist = radius / _grid_map_resolution;
	double dist_rounded = round(dist);
	double mx = half_resolution; //circle center (mx,my) in positive quadrant
	double my = half_resolution;
	double sy = my; //iterator (sx,sy) starting at my and mx + grid aligned radius
	double sx = mx + dist_rounded * _grid_map_resolution;
	unsigned int x_offset = (unsigned int) round(
			(sx - mx) / _grid_map_resolution);
	unsigned int y_offset = (unsigned int) round(
			(sy - my) / _grid_map_resolution);
	circle_lines_offset.push_back(CircleLine(x_offset, y_offset));
	double radius_squared = pow(radius, 2);
	while (sx >= mx) {
		sy += _grid_map_resolution;
		while (sx > mx
				&& radius_squared
						< (pow(sy - my - half_resolution, 2)
								+ pow(sx - mx - half_resolution, 2))) {
			//check if iterator is above circle center and lower right grid tile corner is outside circle's radius
			sx -= _grid_map_resolution;
		}
		if (sx <= mx && radius < (sy - my - half_resolution)) {
			return circle_lines_offset; // no cells to add for this last line
		}
		unsigned int x_offset = (unsigned int) round(
				(sx - mx) / _grid_map_resolution);
		unsigned int y_offset = (unsigned int) round(
				(sy - my) / _grid_map_resolution); //round necessary because of double->int conversion inaccuracies
		circle_lines_offset.push_back(CircleLine(x_offset, y_offset));
	}
	return circle_lines_offset;
}

void CollisionChecker::precalculateCircleLinesOffset() {
	_circle_lines_offset = calculateCircleLinesOffset(_robot_radius);
}

bool CollisionChecker::steer(rrg_nbv_exploration_msgs::Graph &rrg,
		rrg_nbv_exploration_msgs::Node &new_node,
		geometry_msgs::Point rand_sample) {
	int nearest_node;
	if (!(_inflation_active ?
			checkEngulfing(rand_sample, nearest_node, rrg) :
			checkDistance(rand_sample, nearest_node, rrg))) {
		return false; //node is engulfed by other node's radius (inflation active) or too close (inactive)
	}
	alignPointToGridMap(rand_sample);
	nav_msgs::OccupancyGrid map = *_occupancy_grid;
	_vis_map.header.stamp = ros::Time::now();
	_vis_map.info.map_load_time = ros::Time::now();
	std::vector<int8_t> tmp_vis_map_data = _vis_map.data;
	//check circle for collisions and inflate it up to max sensor range if none are found
	int node_cost = 0, node_tiles = 0;
	if (!isCircleInCollision(rand_sample.x, rand_sample.y, map,
			tmp_vis_map_data, node_cost, node_tiles)) {
		double yaw = atan2(rand_sample.y - rrg.nodes[nearest_node].position.y,
				rand_sample.x - rrg.nodes[nearest_node].position.x);
		if (_inflation_active) {
			double direction = yaw * 180.0 / M_PI;
			if (direction < 0)
				direction += 360;
			int direction_from_nearest_node = ((int) round(direction / 45))
					* 45;
			if (direction_from_nearest_node == 360)
				direction_from_nearest_node = 0;
			new_node.radius = inflateCircle(rand_sample.x, rand_sample.y,
					direction_from_nearest_node, _move_nodes, map,
					tmp_vis_map_data, node_cost, node_tiles);
			new_node.squared_radius = pow(new_node.radius, 2);
			rrg.largest_node_radius = std::max(new_node.radius,
					rrg.largest_node_radius);
		} else {
			new_node.radius = _robot_radius;
			new_node.squared_radius = pow(_robot_radius, 2);
		}
		new_node.position.x = rand_sample.x;
		new_node.position.y = rand_sample.y;
		new_node.position.z = rand_sample.z;
		new_node.status = rrg_nbv_exploration_msgs::Node::INITIAL;
		//check if new node can be connected to other nodes
		std::vector<std::pair<int, double>> nodes =
				_graph_searcher->searchInRadius(new_node.position,
						_inflation_active ?
								pow(_sensor_range + rrg.largest_node_radius,
										2) :
								_max_edge_distance_squared); //search for neighbors in largest existing radius + max radius possible (which could be reached by this node) if inflation is active
		bool connected = false;
		double height = 0;
		std::vector<rrg_nbv_exploration_msgs::Edge> new_edges;
		std::vector<visualization_msgs::Marker> new_edge_markers;
		for (auto it : nodes) {
			double distance = sqrt(it.second);
			double edge_yaw = atan2(
					new_node.position.y - rrg.nodes[it.first].position.y,
					new_node.position.x - rrg.nodes[it.first].position.x);
			geometry_msgs::Point edge_center;
			double edge_length;
			int edge_cost = 0, edge_tiles = 0;
			bool check_required = calculateEdge(it.first, edge_length,
					distance, edge_center, new_node, rrg);
			if (edge_length > _max_edge_distance) //path box only allowed up to a given length
				continue;
			else if (_inflation_active && edge_length <= 0
					&& it.second < rrg.nodes[it.first].squared_radius) {
				return false; //new node position engulfed by this node's radius
			}
			bool edge_free =
					check_required ?
							fmod(edge_yaw, M_PI / 2) == 0 ?
									!isAlignedRectangleInCollision(
											edge_center.x, edge_center.y,
											edge_yaw, edge_length / 2,
											_robot_width / 2, map,
											tmp_vis_map_data, edge_cost,
											edge_tiles) :
									!isRectangleInCollision(edge_center.x,
											edge_center.y, edge_yaw,
											edge_length / 2, _robot_width / 2,
											map, tmp_vis_map_data, edge_cost,
											edge_tiles)
							: true;
			if (edge_free) {
				rrg_nbv_exploration_msgs::Edge new_edge;
				connected = true;
				height += rrg.nodes[it.first].position.z;
				new_edge.yaw = (int) (edge_yaw * 180.0 / M_PI);
				if (new_edge.yaw < 0)
					new_edge.yaw += 360;
				new_edge.traversability_cost =
						edge_length > 0 ? edge_cost : 0.0;
				new_edge.traversability_weight =
						edge_length > 0 ? edge_tiles : 0.0;
				new_edge.first_node = it.first;
				new_edge.second_node = rrg.node_counter;
				new_edge.length = distance;
				new_edges.push_back(new_edge);
				visualizeEdge(edge_length, edge_center, edge_yaw,
						new_edge_markers);

			}
		}
		if (connected) {
			double shortest_distance = std::numeric_limits<double>::infinity();
			int node_with_shortest_distance = -1;
			int edge_to_shortest_distance = -1;
			for (auto &edge : new_edges) {
				edge.index = rrg.edge_counter++;
				new_node.edges.push_back(edge.index);
				new_node.edge_counter++;
				rrg.edges.push_back(edge);
				rrg.nodes[edge.first_node].edges.push_back(edge.index);
				rrg.nodes[edge.first_node].edge_counter++;
				//find shortest distance to new node
				double distance_to_robot =
						rrg.nodes[edge.first_node].distance_to_robot
								+ edge.length;
				if (distance_to_robot < shortest_distance) {
					shortest_distance = distance_to_robot;
					node_with_shortest_distance = edge.first_node;
					edge_to_shortest_distance = edge.index;
				}
			}

			new_node.status = rrg_nbv_exploration_msgs::Node::INITIAL;
			new_node.gain = -1;
			new_node.reward_function = 0;
			new_node.index = rrg.node_counter++;
			new_node.position.z = height / nodes.size();
			new_node.distance_to_robot = shortest_distance;
			rrg.longest_distance_to_robot = std::max(
					rrg.longest_distance_to_robot, shortest_distance);
			new_node.path_to_robot =
					rrg.nodes[node_with_shortest_distance].path_to_robot;
			new_node.path_to_robot.push_back(new_node.index);
			int heading_out = rrg.edges[edge_to_shortest_distance].yaw; //first node is always root
			new_node.heading_in = heading_out;
			new_node.heading_change_to_robot =
					rrg.nodes[node_with_shortest_distance].heading_change_to_robot
							+ _graph_path_calculator->getAbsoluteAngleDiff(
									heading_out,
									rrg.nodes[node_with_shortest_distance].heading_in); //calculate heading cost from current to neighbor node
			new_node.heading_change_to_robot_best_view = 0.0;
			new_node.traversability_cost = node_cost;
			new_node.traversability_weight = node_tiles;
			new_node.traversability_cost_to_robot =
					rrg.nodes[node_with_shortest_distance].traversability_cost_to_robot
							+ new_node.traversability_cost
							+ rrg.edges[edge_to_shortest_distance].traversability_cost;
			new_node.traversability_weight_to_robot =
					rrg.nodes[node_with_shortest_distance].traversability_weight_to_robot
							+ new_node.traversability_weight
							+ rrg.edges[edge_to_shortest_distance].traversability_weight;
			rrg.highest_traversability_cost_to_robot = std::max(
					rrg.highest_traversability_cost_to_robot,
					(double) new_node.traversability_cost_to_robot
							/ (double) new_node.traversability_weight_to_robot);
			rrg.nodes.push_back(new_node);
			if (_rrt_collision_visualization_pub.getNumSubscribers() > 0) {
				visualizeNode(new_node);
				_rrt_collision_visualization_pub.publish(_node_points);
				for (auto edge_marker : new_edge_markers) {
					_node_edges.markers.push_back(edge_marker);
				}
				_rrt_collision_visualization_pub.publish(_node_edges);
			}
			_vis_map.data = tmp_vis_map_data;
			_visualization_pub.publish(_vis_map);
			return true;
		}
	}
	return false;
}

void CollisionChecker::calculateNextInflatedCircleLinesOffset() {
	double new_radius = -1;
	std::vector<CircleLine> prevOffsetSet;
	if (_inflated_ring_lines_offsets.empty()) { //first inflation ring offset
		new_radius = ceil(_robot_radius / _grid_map_resolution)
				* _grid_map_resolution + _grid_map_resolution; //round up radius to next full grid map tile
		prevOffsetSet = _circle_lines_offset;
	} else if (_inflated_ring_lines_offsets.back().first + _grid_map_resolution
			> _sensor_range) {
		return; //abort, all inflation ring offsets are calculated
	} else { //new inflation ring
		new_radius = _inflated_ring_lines_offsets.back().first
				+ _grid_map_resolution;
		prevOffsetSet = _inflated_ring_lines_offsets.back().second;
	}
	std::vector<CircleLine> newOffsetSet = calculateCircleLinesOffset(
			new_radius);
	for (int i = 0; i < prevOffsetSet.size(); i++) { //prev offset set size must be smaller equals new offset set
		newOffsetSet[i].x_start = prevOffsetSet[i].x_offset + 1;
	}
	_inflated_ring_lines_offsets.push_back(
			std::make_pair(new_radius, newOffsetSet));

}

int CollisionChecker::isSetInCollision(double center_x, double center_y,
		nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map,
		std::vector<CircleLine> offsets, int &cost, int &tiles,
		bool recalculate) {
	unsigned int map_x, map_y;
	if (!worldToMap(center_x, center_y, map_x, map_y, map))
		return Directions::none;
	int new_cost = cost;
	int new_tiles = tiles;
	std::vector<int8_t> tmp_vis_map = vis_map;
	int direction = Directions::center;
	bool fixed_direction = false; //if a direction becomes fixed, it cannot be merged anymore
	for (auto it : offsets) { //always starts at highest x offset and ends at highest y offset
		if (recalculate || it.x_start == 0) { //circle
			if (isLineInCollision(map_x - it.x_offset, map_x + it.x_offset,
					map_y + it.y_offset, map, tmp_vis_map, new_cost,
					new_tiles)) {
				return Directions::none;
			}
			if (it.y_offset != 0) { //y offset is only 0 in the center slice, otherwise symmetrical
				if (isLineInCollision(map_x - it.x_offset, map_x + it.x_offset,
						map_y - it.y_offset, map, tmp_vis_map, new_cost,
						new_tiles)) {
					return Directions::none;
				}
			}
		} else { //ring
			const int max_x_offset = offsets.begin()->x_offset;
			const int max_y_offset = offsets.end()->x_offset;
			if (isLineInCollision(map_x + it.x_start, map_x + it.x_offset,
					map_y + it.y_offset, map, tmp_vis_map, new_cost, new_tiles)) //northwest quadrant
				if (!checkDirection(direction, fixed_direction,
						it.x_offset == max_x_offset ? Directions::south :
						it.y_offset == max_y_offset ?
								Directions::east : Directions::southeast))
					return Directions::none;
			if (isLineInCollision(map_x - it.x_offset, map_x - it.x_start,
					map_y + it.y_offset, map, tmp_vis_map, new_cost, new_tiles)) //southwest quadrant
				if (!checkDirection(direction, fixed_direction,
						it.x_offset == max_x_offset ? Directions::north :
						it.y_offset == max_y_offset ?
								Directions::east : Directions::northeast))
					return Directions::none;
			if (it.y_offset != 0) { //y offset is only 0 in the center slice, otherwise symmetrical
				if (isLineInCollision(map_x + it.x_start, map_x + it.x_offset,
						map_y - it.y_offset, map, tmp_vis_map, new_cost,
						new_tiles)) //northeast quadrant
					if (!checkDirection(direction, fixed_direction,
							it.x_offset == max_x_offset ? Directions::south :
							it.y_offset == max_y_offset ?
									Directions::west : Directions::southwest))
						return Directions::none;
				if (isLineInCollision(map_x - it.x_offset, map_x - it.x_start,
						map_y - it.y_offset, map, tmp_vis_map, new_cost,
						new_tiles)) //southeast quadrant
					if (!checkDirection(direction, fixed_direction,
							it.x_offset == max_x_offset ? Directions::north :
							it.y_offset == max_y_offset ?
									Directions::west : Directions::northwest))
						return Directions::none;
			}
		}
	}
	if (direction == Directions::center) { //only store cost, tiles and visualization when no collision was detected
		cost = new_cost;
		tiles = new_tiles;
		vis_map = tmp_vis_map;
	}
	return direction;
}

bool CollisionChecker::isCircleInCollision(double center_x, double center_y,
		nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map, int &cost,
		int &tiles) {
	return isSetInCollision(center_x, center_y, map, vis_map,
			_circle_lines_offset, cost, tiles) != Directions::center;
}

bool CollisionChecker::checkDirection(int &current_direction, bool &fixed,
		int new_direction) {
	if (current_direction == Directions::center) { //first direction (initialization)
		current_direction = new_direction;
		return true;
	}
	const int difference = getAbsoluteAngleDiff(current_direction,
			new_direction);
	if (difference == 0) {
		return true; //same directions
	} else if ((fixed && difference >= 90) || difference >= 135) {
		return false;
	} else if (difference <= 90) { //merge directions
		int keep_direction = current_direction;
		if (fixed || difference == 45) { //keep diagonal direction
			current_direction =
					(current_direction == 0 || (current_direction % 90) == 0) ?
							new_direction : current_direction;
		} else if (current_direction <= west && new_direction >= east) { //jump around north
			current_direction = (current_direction - difference / 2) % 360;
		} else if (new_direction <= west && current_direction >= east) {
			current_direction = (new_direction - difference / 2) % 360;
		} else if (current_direction > new_direction) { //merge
			current_direction = current_direction - difference / 2;
		} else {
			current_direction = new_direction - difference / 2;
		}
		fixed = true;
	}
	return true;
}

double CollisionChecker::inflateCircle(double &x, double &y,
		int direction_from_nearest_node, bool move_node,
		nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map, int &cost,
		int &tiles) {
	std::vector<int8_t> tmp_vis_map = vis_map;
	if (_inflated_ring_lines_offsets.empty())
		calculateNextInflatedCircleLinesOffset();
	double current_radius = _inflated_ring_lines_offsets.front().first;
	auto it = _inflated_ring_lines_offsets.begin();
	int index = 0;
	std::vector<std::pair<double, double>> previous_positions;
	previous_positions.push_back(std::make_pair(x, y));
	while (it != _inflated_ring_lines_offsets.end()) {
		int direction = isSetInCollision(x, y, map, tmp_vis_map, it->second,
				cost, tiles);
		if (direction == Directions::none) {
			return current_radius;
		} else {
			if (move_node) {
				if (direction == Directions::center) {
					moveCircle(x, y, direction_from_nearest_node, index + 1,
							previous_positions, map, tmp_vis_map, cost, tiles);	//move circle with same inflation away from nearest node
					current_radius = it->first;
					if (it == --_inflated_ring_lines_offsets.end()) { //calculate next inflation ring if not reached sensor range
						calculateNextInflatedCircleLinesOffset();
					}
					index++;
					it = _inflated_ring_lines_offsets.begin() + index; //increasing vector size invalidates iterator
				} else {
					if (!moveCircle(x, y, direction, index, previous_positions,
							map, tmp_vis_map, cost, tiles)) { //don't increment iterator, repeat this inflation for new position next iteration
						return current_radius;
					}
				}
			} else {
				if (direction == Directions::center) {
					current_radius = it->first;
					if (it == --_inflated_ring_lines_offsets.end()) { //calculate next inflation ring if not reached sensor range
						calculateNextInflatedCircleLinesOffset();
					}
					index++;
					it = _inflated_ring_lines_offsets.begin() + index; //increasing vector size invalidates iterator
				} else {
					return current_radius;
				}
			}
		}
		vis_map = tmp_vis_map;
	}
	return current_radius;
}

bool CollisionChecker::moveCircle(double &x, double &y, int direction, int ring,
		std::vector<std::pair<double, double>> &previous_positions,
		nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map, int &cost,
		int &tiles) {
	geometry_msgs::Point new_point;
	new_point.x = x;
	new_point.y = y;
	if (direction <= Directions::northwest || direction >= northeast) {
		new_point.x += _grid_map_resolution;
	} else if (direction <= southeast && direction >= southwest) {
		new_point.x -= _grid_map_resolution;
	}
	if (direction <= Directions::northeast && direction >= southeast) {
		new_point.y -= _grid_map_resolution;
	} else if (direction <= Directions::southwest && direction >= northwest) {
		new_point.y += _grid_map_resolution;
	}
//check distance to other nodes
//TODO: check if went too far away (regard edge length) and regard radius in min distance
	double min_distance;
	int nearest_node;
	_graph_searcher->findNearestNeighbour(new_point, min_distance,
			nearest_node);
	if (min_distance < _min_edge_distance_squared) {
//		ROS_INFO_STREAM(
//				"Point ("<< new_point.x << ", " << new_point.y<<") too close to node " << nearest_node << " (" << sqrt(min_distance) << "m)");
		return false;
	}
//check previous positions
	for (auto it = previous_positions.rbegin(); it != previous_positions.rend();
			++it) {
		if (it->first == new_point.x && it->second == new_point.y) {
//			ROS_INFO_STREAM(
//					"Point ("<< new_point.x << ", " << new_point.y<<") was already tried");
			return false;
		}
	}
//	ROS_INFO_STREAM(
//			"Inflated rings size " << _inflated_ring_lines_offsets.size() << " access ring " << (ring - 1));
	if (isSetInCollision(new_point.x, new_point.y, map, vis_map,
			ring == 0 ?
					_circle_lines_offset :
					_inflated_ring_lines_offsets.at(ring - 1).second, cost,
			tiles, true) == Directions::center) {
//		ROS_INFO_STREAM("Moved to position: " << x << ", " << y);
		x = new_point.x;
		y = new_point.y;
		previous_positions.push_back(std::make_pair(x, y));
		return true;
	} else {
//		ROS_INFO_STREAM("Moving failed due to collision");
		return false;
	}
}

bool CollisionChecker::isRectangleInCollision(double x, double y, double yaw,
		double half_height, double half_width, nav_msgs::OccupancyGrid &map,
		std::vector<int8_t> &vis_map, int &cost, int &tiles) {
	std::array<MapPoint, 4> map_corners;
	std::array<GridPoint, 4> grid_corners;
	double cos_yaw, sin_yaw;
	if (yaw < -PI_HALF) {
		yaw += M_PI; //	combine mirrored case (-PI < yaw < -PI/2 => 0 < yaw < PI/2)
	} else if (yaw > PI_HALF) {
		yaw -= M_PI; //	combine mirrored case (PI > yaw > PI/2 => 0 > yaw > -PI/2)
	}
	cos_yaw = cos(yaw);
	sin_yaw = sin(yaw);
	double width_cos = half_width * cos_yaw;
	double width_sin = half_width * sin_yaw;
	double height_cos = half_height * cos_yaw;
	double height_sin = half_height * sin_yaw;
	if (yaw > 0 && yaw < PI_HALF) { //height from top left to bottom right
		map_corners[0] = { x + height_cos - width_sin, y + width_cos
				+ height_sin }; //left corner
		map_corners[1] = { x - height_cos - width_sin, y + width_cos
				- height_sin }; //bottom corner
		map_corners[2] = { x + height_cos + width_sin, y - width_cos
				+ height_sin }; //top corner
		map_corners[3] = { x - height_cos + width_sin, y - width_cos
				- height_sin }; //right corner
	} else { // yaw >  -PI_HALF && yaw < 0 (height from bottom left to top right)
		map_corners[0] = { x - height_cos - width_sin, y + width_cos
				- height_sin }; //left corner
		map_corners[1] = { x - height_cos + width_sin, y - width_cos
				- height_sin }; //bottom corner
		map_corners[2] = { x + height_cos - width_sin, y + width_cos
				+ height_sin }; //top corner
		map_corners[3] = { x + height_cos + width_sin, y - width_cos
				+ height_sin }; //right corner
	}
	double gradient_up = (map_corners[2].x - map_corners[3].x)
			/ (map_corners[2].y - map_corners[3].y);
	double gradient_down = -1 / gradient_up;
	if (!worldToMap(map_corners[0].x, map_corners[0].y, grid_corners[0].x,
			grid_corners[0].y, map)
			|| !worldToMap(map_corners[1].x, map_corners[1].y,
					grid_corners[1].x, grid_corners[1].y, map)
			|| !worldToMap(map_corners[2].x, map_corners[2].y,
					grid_corners[2].x, grid_corners[2].y, map)
			|| !worldToMap(map_corners[3].x, map_corners[3].y,
					grid_corners[3].x, grid_corners[3].y, map))
		return true;
	unsigned int grid_x = grid_corners[3].x;
	unsigned int grid_y = grid_corners[3].y;
	double gradient_top = gradient_up;
	double gradient_bot = gradient_down;
	double iterator_y = (floor(map_corners[3].y / _grid_map_resolution) + 1.0)
			* _grid_map_resolution;
	double iterator_x = (floor(map_corners[3].x / _grid_map_resolution) + 0.5)
			* _grid_map_resolution;
	double iterator_x_top = map_corners[3].x
			+ gradient_top * (iterator_y - map_corners[3].y);
	double offset_x_top = (iterator_x_top - iterator_x) / _grid_map_resolution;
	double iterator_x_bot = map_corners[3].x
			+ gradient_bot * (iterator_y - map_corners[3].y);
	double offset_x_bot = (iterator_x_bot - iterator_x) / _grid_map_resolution;
	bool top_corner_reached = false;
	bool bot_corner_reached = false;
	while (iterator_y < map_corners[0].y) {
		int x_start, x_end;
		if (!top_corner_reached && iterator_y >= map_corners[2].y) { // top corner reached, change gradient
			gradient_top = gradient_down;
			iterator_x_top = map_corners[2].x
					+ gradient_top
							* (iterator_y - map_corners[2].y
									- _grid_map_resolution);
			offset_x_top = (iterator_x_top - iterator_x) / _grid_map_resolution;
			top_corner_reached = true;
			x_end = grid_corners[2].x;
		} else
			x_end = grid_x + (int) round(offset_x_top);
		if (!bot_corner_reached && iterator_y >= map_corners[1].y) { // bot corner reached, change gradient
			gradient_bot = gradient_up;
			iterator_x_bot = map_corners[1].x
					+ gradient_bot
							* (iterator_y - map_corners[1].y
									- _grid_map_resolution);
			offset_x_bot = (iterator_x_bot - iterator_x) / _grid_map_resolution;
			bot_corner_reached = true;
			x_start = grid_corners[1].x;
		} else
			x_start = grid_x + (int) round(offset_x_bot);
		if (isLineInCollision(x_start, x_end, grid_y, map, vis_map, cost,
				tiles))
			return true;
		if (iterator_y < map_corners[0].y) {
			offset_x_top += gradient_top;
			offset_x_bot += gradient_bot;
		}
		grid_y += 1;
		iterator_y += _grid_map_resolution;
	}
	if (isLineInCollision(grid_x + (int) round(offset_x_bot),
			grid_x + (int) round(offset_x_top), grid_y, map, vis_map, cost,
			tiles))
		return true;
	return false;
}

bool CollisionChecker::isAlignedRectangleInCollision(double x, double y,
		double yaw, double half_height, double half_width,
		nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map, int &cost,
		int &tiles) {
	std::array<MapPoint, 2> map_corners;
	std::array<GridPoint, 2> grid_corners;
	if (yaw == -M_PI / 2 || yaw == M_PI / 2) { //height in y direction
		map_corners[0] = { x - half_width, y + half_height }; //bottom left corner
		map_corners[1] = { x + half_width, y - half_height }; //top right corner
	} else { //yaw == -M_PI || 0 || M_PI (height in x direction)
		map_corners[0] = { x - half_height, y + half_width }; //bottom left corner
		map_corners[1] = { x + half_height, y - half_width }; //top right corner
	}
	if (!worldToMap(map_corners[0].x, map_corners[0].y, grid_corners[0].x,
			grid_corners[0].y, map)
			|| !worldToMap(map_corners[1].x, map_corners[1].y,
					grid_corners[1].x, grid_corners[1].y, map))
		return true;
	for (unsigned int i = grid_corners[1].y; i <= grid_corners[0].y; i++) {
		if (isLineInCollision(grid_corners[0].x, grid_corners[1].x, i, map,
				vis_map, cost, tiles))
			return true;
	}
	return false;
}

bool CollisionChecker::isLineInCollision(int x_start, int x_end, int y,
		nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map, int &cost,
		int &tiles) {
	if (x_start < 0 || x_end > map.info.width || y < 0 || y > map.info.height) {
		return true;
	}
	for (int x = y * map.info.width + x_start; x <= y * map.info.width + x_end;
			x++) {
		if (map.data[x] == _grid_map_unknown
				|| map.data[x] >= _grid_map_occupied) {
			return true;
		} else {
			cost += map.data[x];
			tiles += 1;
			vis_map[x] = map.data[x];
		}
	}
	return false;
}

void CollisionChecker::alignPointToGridMap(geometry_msgs::Point &rand_sample) {
	rand_sample.x = (round(rand_sample.x / _grid_map_resolution) + 0.5)
			* _grid_map_resolution;
	rand_sample.y = (round(rand_sample.y / _grid_map_resolution) + 0.5)
			* _grid_map_resolution;
}

bool CollisionChecker::calculateEdge(int nearest_node, double &edge_length,
		double distance, geometry_msgs::Point &edge_center,
		rrg_nbv_exploration_msgs::Node &new_node,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	if (_inflation_active) {
		double new_node_path_box_distance = sqrt(
				new_node.squared_radius - pow(_robot_width / 2, 2));
		double nearest_node_path_box_distance = sqrt(
				rrg.nodes[nearest_node].squared_radius
						- pow(_robot_width / 2, 2));
		edge_length = distance - new_node_path_box_distance
				- nearest_node_path_box_distance;
		if (edge_length <= 0) {
			edge_length = 0;
			return false;
		}
		double center_factor =
				(nearest_node_path_box_distance + edge_length / 2) / distance;
		edge_center.x = rrg.nodes[nearest_node].position.x
				+ center_factor
						* (new_node.position.x
								- rrg.nodes[nearest_node].position.x);
		edge_center.y = rrg.nodes[nearest_node].position.y
				+ center_factor
						* (new_node.position.y
								- rrg.nodes[nearest_node].position.y);
	} else {
		edge_length = distance - _path_box_distance_thres;
		if (edge_length <= 0) {
			return false;
		}
		edge_center.x = (rrg.nodes[nearest_node].position.x
				+ new_node.position.x) / 2;
		edge_center.y = (rrg.nodes[nearest_node].position.y
				+ new_node.position.y) / 2;
	}
	return true;
}

bool CollisionChecker::checkEngulfing(geometry_msgs::Point &point,
		int &nearest_node, rrg_nbv_exploration_msgs::Graph &rrg) {
	double min_distance = -1;
	nearest_node = -1;
	std::vector<std::pair<int, double>> nearby_nodes =
			_graph_searcher->searchInRadius(point,
					pow(rrg.largest_node_radius, 2));
	for (auto it : nearby_nodes) {
		if (rrg.nodes[it.first].squared_radius > it.second) {
			return false;
		} else {
			double distance = sqrt(it.second) - rrg.nodes[it.first].radius;
			if (distance < min_distance) {
				nearest_node = it.first;
				min_distance = distance;
			}
		}
	}
	if (min_distance > _max_edge_distance) { //move sample point closer to nearest node to enable connection
		point.x = rrg.nodes[nearest_node].position.x
				- ((_max_edge_distance + rrg.nodes[nearest_node].radius)
						* (rrg.nodes[nearest_node].position.x - point.x)
						/ (min_distance + rrg.nodes[nearest_node].radius));
		point.y = rrg.nodes[nearest_node].position.y
				- ((_max_edge_distance + rrg.nodes[nearest_node].radius)
						* (rrg.nodes[nearest_node].position.y - point.y)
						/ (min_distance + rrg.nodes[nearest_node].radius));
	}
	return true;
}

bool CollisionChecker::checkDistance(geometry_msgs::Point &point,
		int &nearest_node, rrg_nbv_exploration_msgs::Graph &rrg) {
	double min_distance = -1;
	nearest_node = -1;
	_graph_searcher->findNearestNeighbour(point, min_distance, nearest_node);
	if (min_distance >= _min_edge_distance_squared) {
		if (min_distance >= _max_edge_distance_squared) {
			// if random sample is further away than max edge distance, replace it at max distance to the nearest node on a line with the sample
			double distance = sqrt(min_distance);
			point.x = rrg.nodes[nearest_node].position.x
					- (_max_edge_distance
							* (rrg.nodes[nearest_node].position.x - point.x)
							/ distance);
			point.y = rrg.nodes[nearest_node].position.y
					- (_max_edge_distance
							* (rrg.nodes[nearest_node].position.y - point.y)
							/ distance);
		}
		return true;
	}
	return false;
}

void CollisionChecker::occupancyGridCallback(
		const nav_msgs::OccupancyGrid::ConstPtr &map_msg) {
	_occupancy_grid = std::make_shared<nav_msgs::OccupancyGrid>(*map_msg);
}

void CollisionChecker::occupancyGridUpdatesCallback(
		const map_msgs::OccupancyGridUpdate::ConstPtr &map_msg) {
	int index = 0;
	for (int y = map_msg->y; y < map_msg->y + map_msg->height; y++) {
		for (int x = map_msg->x; x < map_msg->x + map_msg->width; x++) {
			_occupancy_grid->data[getIndex(x, y)] = map_msg->data[index++];
		}
	}
}

int CollisionChecker::getIndex(int x, int y) {
	int sx = _occupancy_grid->info.width;
	return y * sx + x;
}

int CollisionChecker::getAbsoluteAngleDiff(int x, int y) {
	return 180 - abs(abs(x - y) - 180);
}

bool CollisionChecker::worldToMap(double wx, double wy, unsigned int &mx,
		unsigned int &my, nav_msgs::OccupancyGrid &map) {
	if (wx < map.info.origin.position.x || wy < map.info.origin.position.y)
		return false;
	mx = (int) ((wx - map.info.origin.position.x) / map.info.resolution);
	my = (int) ((wy - map.info.origin.position.y) / map.info.resolution);
	if (mx < map.info.height && my < map.info.width)
		return true;
	return false;
}

void CollisionChecker::visualizeNode(rrg_nbv_exploration_msgs::Node &node) {
	visualization_msgs::Marker node_point;
	node_point.header.frame_id = "/map";
	node_point.header.stamp = ros::Time();
	node_point.ns = "rrt_collision_vis";
	node_point.id = _marker_id++;
	node_point.action = visualization_msgs::Marker::ADD;
	node_point.pose.orientation.w = 1.0;
	node_point.type = visualization_msgs::Marker::CYLINDER;
	node_point.scale.x = 2 * node.radius;
	node_point.scale.y = 2 * node.radius;
	node_point.scale.z = 0.01;
	node_point.color.g = 1.0f;
	node_point.color.a = 0.5f;
	node_point.pose.position.x = node.position.x;
	node_point.pose.position.y = node.position.y;
	node_point.pose.position.z = 0.005;
	_node_points.markers.push_back(node_point);
}

void CollisionChecker::visualizeEdge(double edge_length,
		const geometry_msgs::Point &edge_center, double edge_yaw,
		std::vector<visualization_msgs::Marker> &new_edge_markers) {
	if (_rrt_collision_visualization_pub.getNumSubscribers() > 0) {
		visualization_msgs::Marker node_edge;
		if (edge_length > 0) {
			node_edge.header.frame_id = "/map";
			node_edge.header.stamp = ros::Time();
			node_edge.ns = "rrt_collision_vis";
			node_edge.id = _marker_id++;
			node_edge.action = visualization_msgs::Marker::ADD;
			node_edge.type = visualization_msgs::Marker::CUBE;
			node_edge.scale.x = edge_length;
			node_edge.scale.y = _robot_width;
			node_edge.scale.z = 0.01;
			node_edge.color.g = 1.0f;
			node_edge.color.a = 0.5f;
			node_edge.pose.position.x = edge_center.x;
			node_edge.pose.position.y = edge_center.y;
			node_edge.pose.position.z = 0.005;
			tf2::Quaternion quaternion;
			quaternion.setRPY(0, 0, edge_yaw);
			quaternion.normalize();
			node_edge.pose.orientation = tf2::toMsg(quaternion);
			new_edge_markers.push_back(node_edge);
		}
	}
}

}
