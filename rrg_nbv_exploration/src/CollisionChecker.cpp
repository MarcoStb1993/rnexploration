#include <rrg_nbv_exploration/CollisionChecker.h>

namespace rrg_nbv_exploration {

CollisionChecker::CollisionChecker() {
	ros::NodeHandle private_nh("~");
	private_nh.param("robot_radius", _robot_radius, 1.0);
	private_nh.param("robot_width", _robot_width, 1.0);
	private_nh.param("sensor_max_range", _sensor_range, 5.0);
	private_nh.param("local_graph_radius", _local_graph_radius, 5.0);
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
	_available_nodes.clear();
	_available_edges.clear();
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
	int robot_yaw = _graph_path_calculator->getRobotYaw(
			_graph_path_calculator->getRobotPose()); //get current robot orientation (yaw) for heading change calculation
	std::vector<int8_t> tmp_vis_map_data = _vis_map.data;
	int node_cost = 0, node_tiles = 0, node_collision = Collisions::empty;
	if (_inflation_active
			&& !isCircleInCollision(rrg.nodes[0].position.x,
					rrg.nodes[0].position.y, map, tmp_vis_map_data, node_cost,
					node_tiles, node_collision)) {
		rrg.nodes[0].radius = inflateCircle(rrg.nodes[0].position.x,
				rrg.nodes[0].position.y, false, rrg.nodes[0], map,
				tmp_vis_map_data, node_cost, node_tiles, node_collision,
				_robot_radius, _sensor_range);
		rrg.nodes[0].squared_radius = pow(rrg.nodes[0].radius, 2);
		rrg.nodes[0].traversability_cost = node_cost;
		rrg.nodes[0].traversability_weight = node_tiles > 0 ? node_tiles : 1;
		rrg.nodes[0].traversability_cost_to_robot =
				rrg.nodes[0].traversability_cost;
		rrg.nodes[0].traversability_weight_to_robot =
				rrg.nodes[0].traversability_weight;
		rrg.largest_node_radius = std::max(rrg.nodes[0].radius,
				rrg.largest_node_radius);
		rrg.nodes[0].radii_to_robot = rrg.nodes[0].radius;
		if (_rrt_collision_visualization_pub.getNumSubscribers() > 0) {
			visualizeNode(rrg.nodes[0]);
			_rrt_collision_visualization_pub.publish(_node_points);
		}
	} else { // no inflation active, do not check collision for root
		rrg.nodes[0].radius = _robot_radius;
		rrg.nodes[0].squared_radius = pow(_robot_radius, 2);
		rrg.largest_node_radius = _robot_radius;
		rrg.nodes[0].traversability_cost = 0;
		rrg.nodes[0].traversability_weight = 1.0;
		rrg.nodes[0].traversability_cost_to_robot = 0;
		rrg.nodes[0].traversability_weight_to_robot = 1.0;
		rrg.nodes[0].radii_to_robot = _robot_radius;
	}
	rrg.nodes[0].retry_inflation = node_collision == Collisions::unknown;
	rrg.nodes[0].distance_to_robot = 0;
	rrg.nodes[0].path_to_robot.push_back(0);
	rrg.nodes[0].heading_in = robot_yaw;
	rrg.nodes[0].heading_change_to_robot = 0;
	rrg.nodes[0].heading_change_to_robot_best_view = 0;
	rrg.nodes[0].cost_function = _graph_path_calculator->calculateCostFunction(
			rrg.nodes[0]);

	_vis_map.data = tmp_vis_map_data;
	_visualization_pub.publish(_vis_map);
}

std::vector<CollisionChecker::CircleLine> CollisionChecker::calculateCircleLinesOffset(
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
		geometry_msgs::Point rand_sample, geometry_msgs::Pose &robot_pos,
		bool unmovable_point) {
	int nearest_node;

	new_node.position.x = rand_sample.x;
	new_node.position.y = rand_sample.y;
	new_node.position.z = rand_sample.z;
	new_node.radius = 0.05;

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
	int node_cost = 0, node_tiles = 0, node_collision = Collisions::empty;
	if (!isCircleInCollision(rand_sample.x, rand_sample.y, map,
			tmp_vis_map_data, node_cost, node_tiles, node_collision)) {
		if (_inflation_active) {
			new_node.radius = inflateCircle(rand_sample.x, rand_sample.y,
					unmovable_point ? false : _move_nodes,
					rrg.nodes[nearest_node], map, tmp_vis_map_data, node_cost,
					node_tiles, node_collision, _robot_radius, _sensor_range);
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
		new_node.index = getAvailableNodeIndex(rrg);
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
		std::vector<rrg_nbv_exploration_msgs::Edge> new_retriable_edges;
		std::vector<visualization_msgs::Marker> new_edge_markers;
		for (auto it : nodes) {
			if (rrg.nodes[it.first].status
					== rrg_nbv_exploration_msgs::Node::INACTIVE) //skip connections to inactive nodes
				continue;
			double distance = sqrt(it.second);
			double edge_yaw = atan2(
					new_node.position.y - rrg.nodes[it.first].position.y,
					new_node.position.x - rrg.nodes[it.first].position.x);
			geometry_msgs::Point edge_center;
			double edge_length;
			int edge_cost = 0, edge_tiles = 0, edge_collision =
					Collisions::empty;
			bool check_required = calculateEdge(it.first, edge_length, distance,
					edge_center, new_node, rrg);
			if (edge_length > _max_edge_distance) //path box only allowed up to a given length
				continue;
			else if (_inflation_active && edge_length > 0) { //only connect nodes with enough radius overlap
				continue;
			} else if (_inflation_active
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
											edge_tiles, edge_collision) :
									!isRectangleInCollision(edge_center.x,
											edge_center.y, edge_yaw,
											edge_length / 2, _robot_width / 2,
											map, tmp_vis_map_data, edge_cost,
											edge_tiles, edge_collision)
							: true;
			if (edge_free) {
				connected = true;
				height += rrg.nodes[it.first].position.z;
				rrg_nbv_exploration_msgs::Edge new_edge = constructEdge(
						edge_yaw, edge_length, edge_cost, edge_tiles, it.first,
						new_node.index, distance, rrg);
				new_edges.push_back(new_edge);
				visualizeEdge(edge_length, edge_center, edge_yaw,
						new_edge_markers);
			} else if (edge_collision == Collisions::unknown) { //store failed edge for later reassessment
				rrg_nbv_exploration_msgs::Edge new_edge = constructEdge(
						edge_yaw, edge_length, edge_cost, edge_tiles, it.first,
						new_node.index, distance, rrg);
				new_retriable_edges.push_back(new_edge);
			}

		}
		if (connected) {
			new_node.traversability_cost = node_cost;
			new_node.traversability_weight = node_tiles > 0 ? node_tiles : 1;
			new_node.status = rrg_nbv_exploration_msgs::Node::INITIAL;
			new_node.gain = -1;
			new_node.reward_function = 0;
			new_node.position.z = height / nodes.size();
			new_node.retry_inflation = node_collision == Collisions::unknown;
			new_node.cost_function = std::numeric_limits<double>::infinity();
			findBestConnectionForNode(rrg, new_node, robot_pos, true,
					new_edges);
			insertNodeInRrg(new_node, rrg);
			for (auto edge : new_retriable_edges) { //add edges which failed because of unknown space
				_retriable_edges.push_back(edge);
			}
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

std::vector<int> CollisionChecker::inflateExistingNode(rrg_nbv_exploration_msgs::Graph &rrg,
		int node, geometry_msgs::Pose robot_pos,
		std::list<int> &nodes_to_update, bool &added_node_to_update) {
	std::vector<int> engulfed_nodes;
	_vis_map.header.stamp = ros::Time::now();
	_vis_map.info.map_load_time = ros::Time::now();
	std::vector<int8_t> tmp_vis_map_data = _vis_map.data;
	nav_msgs::OccupancyGrid map = *_occupancy_grid;
	int node_cost = 0, node_tiles = 0, node_collision = Collisions::empty;
	double new_radius = inflateCircle(rrg.nodes[node].position.x,
			rrg.nodes[node].position.y, false, rrg.nodes[node], map,
			tmp_vis_map_data, node_cost, node_tiles, node_collision,
			rrg.nodes[node].radius, _sensor_range);
	if (new_radius > rrg.nodes[node].radius) {
		rrg.nodes[node].retry_inflation = node_collision == Collisions::unknown;
		rrg.nodes[node].radius = new_radius;
		rrg.nodes[node].squared_radius = pow(new_radius, 2);
		rrg.largest_node_radius = std::max(new_radius, rrg.largest_node_radius);
		rrg.nodes[node].traversability_cost = node_cost;
		rrg.nodes[node].traversability_weight = node_tiles > 0 ? node_tiles : 1;
		rrg.nodes[node].cost_function =
				_graph_path_calculator->calculateCostFunction(rrg.nodes[node]);
		std::vector<std::pair<int, double>> nodes =
				_graph_searcher->searchInRadius(rrg.nodes[node].position,
						pow(new_radius + rrg.largest_node_radius, 2));
		for (auto it : nodes) {
			double distance = sqrt(it.second);
			if(rrg.nodes.at(it.first).gain == 0 && new_radius > distance + rrg.nodes.at(it.first).radius){ //node is completely engulfed by inflated node's new radius
				engulfed_nodes.push_back(it.first);
			}
			if (it.first == node
					|| _graph_path_calculator->findExistingEdge(rrg, node,
							it.first) != -1
					|| rrg.nodes[it.first].status
							== rrg_nbv_exploration_msgs::Node::INACTIVE)
				continue; //skip node itself, existing edge or inactive node
			geometry_msgs::Point edge_center;
			double edge_length;
			int edge_cost = 0, edge_tiles = 0;
			if (!calculateEdge(it.first, edge_length, distance, edge_center,
					rrg.nodes[node], rrg)) {
				double edge_yaw = atan2(
						rrg.nodes[node].position.y
								- rrg.nodes[it.first].position.y,
						rrg.nodes[node].position.x
								- rrg.nodes[it.first].position.x);
				rrg_nbv_exploration_msgs::Edge new_edge;
				new_edge.yaw = (int) (edge_yaw * 180.0 / M_PI);
				if (new_edge.yaw < 0)
					new_edge.yaw += 360;
				if (node < it.first)
					new_edge.yaw += new_edge.yaw > 180 ? (-180) : 180;
				new_edge.traversability_cost = 0.0;
				new_edge.traversability_weight = 0.0;
				new_edge.first_node = std::min(it.first, node);
				new_edge.second_node = std::max(it.first, node);
				new_edge.length = distance;
				insertEdgeInRrg(new_edge, rrg);
				rrg.nodes[node].edges.push_back(new_edge.index);
				rrg.nodes[node].edge_counter++;
				rrg.nodes[it.first].edges.push_back(new_edge.index);
				rrg.nodes[it.first].edge_counter++;
				checkNewEdgePathLength(robot_pos, rrg, new_edge,
						nodes_to_update, added_node_to_update);
			}
		}
		if (_rrt_collision_visualization_pub.getNumSubscribers() > 0) {
			visualizeNode(rrg.nodes[node]);
			_rrt_collision_visualization_pub.publish(_node_points);
		}
		_vis_map.data = tmp_vis_map_data;
		_visualization_pub.publish(_vis_map);
	}
	return engulfed_nodes;
}

int CollisionChecker::collisionCheckForFailedNode(
		rrg_nbv_exploration_msgs::Graph &rrg, int node) {
	nav_msgs::OccupancyGrid map = *_occupancy_grid;
	std::vector<int8_t> tmp_vis_map_data = _vis_map.data;
	int node_cost = 0, node_tiles = 0, node_collision = Collisions::empty;
	if (!isCircleInCollision(rrg.nodes[node].position.x,
			rrg.nodes[node].position.y, map, tmp_vis_map_data, node_cost,
			node_tiles, node_collision))
		if (_inflation_active && rrg.nodes[node].radius > _robot_radius)
			inflateCircle(rrg.nodes[node].position.x,
					rrg.nodes[node].position.y, false, rrg.nodes[node], map,
					tmp_vis_map_data, node_cost, node_tiles, node_collision,
					_robot_radius, rrg.nodes[node].radius);
	return node_collision;
}

void CollisionChecker::checkNewEdgePathLength(
		const geometry_msgs::Pose &robot_pos,
		rrg_nbv_exploration_msgs::Graph &rrg,
		rrg_nbv_exploration_msgs::Edge &edge, std::list<int> &nodes_to_update,
		bool &added_node_to_update) {
	if (rrg.nodes[edge.first_node].distance_to_robot + edge.length
			< rrg.nodes[edge.second_node].distance_to_robot) {
		_graph_path_calculator->updatePathsToRobot(edge.first_node, rrg,
				robot_pos, false, nodes_to_update, added_node_to_update);
	} else if (rrg.nodes[edge.second_node].distance_to_robot + edge.length
			< rrg.nodes[edge.first_node].distance_to_robot) {
		_graph_path_calculator->updatePathsToRobot(edge.second_node, rrg,
				robot_pos, false, nodes_to_update, added_node_to_update);
	}
}

void CollisionChecker::retryEdges(rrg_nbv_exploration_msgs::Graph &rrg,
		geometry_msgs::Pose robot_pos, std::list<int> &nodes_to_update,
		bool &added_node_to_update) {
	nav_msgs::OccupancyGrid map = *_occupancy_grid;
	_vis_map.header.stamp = ros::Time::now();
	_vis_map.info.map_load_time = ros::Time::now();
	std::vector<int8_t> tmp_vis_map_data = _vis_map.data;
	bool edge_added = false;
	std::vector<visualization_msgs::Marker> new_edge_markers;
	_retriable_edges.erase(
			std::remove_if(_retriable_edges.begin(), _retriable_edges.end(),
					[this, &rrg, &robot_pos, &map, &tmp_vis_map_data,
							&edge_added, &new_edge_markers, &nodes_to_update,
							&added_node_to_update](
							rrg_nbv_exploration_msgs::Edge &edge) {
						int edge_cost = 0, edge_tiles = 0, edge_collision =
								Collisions::empty;
						geometry_msgs::Point edge_center;
						edge_center.x = (rrg.nodes[edge.first_node].position.x
								+ rrg.nodes[edge.second_node].position.x) / 2;
						edge_center.y = (rrg.nodes[edge.first_node].position.y
								+ rrg.nodes[edge.second_node].position.y) / 2;
						double edge_length = edge.length
								- _path_box_distance_thres;
						fmod(edge.yaw, M_PI / 2) == 0 ?
								isAlignedRectangleInCollision(edge_center.x,
										edge_center.y, edge.yaw, edge_length,
										_robot_width / 2, map, tmp_vis_map_data,
										edge_cost, edge_tiles, edge_collision) :
								isRectangleInCollision(edge_center.x,
										edge_center.y, edge.yaw, edge_length,
										_robot_width / 2, map, tmp_vis_map_data,
										edge_cost, edge_tiles, edge_collision);
						if (edge_collision == Collisions::empty) { //add edge to RRG
							insertEdgeInRrg(edge, rrg);
							rrg.nodes[edge.first_node].edges.push_back(
									edge.index);
							rrg.nodes[edge.first_node].edge_counter++;
							rrg.nodes[edge.second_node].edges.push_back(
									edge.index);
							rrg.nodes[edge.second_node].edge_counter++;
							edge_added = true;
							visualizeEdge(edge_length, edge_center, edge.yaw,
									new_edge_markers);
							checkNewEdgePathLength(robot_pos, rrg, edge,
									nodes_to_update, added_node_to_update);
							return true;
						} else if (edge_collision == Collisions::occupied) { //discard edge
							return true;
						}
						return false; //edge stays in list if still unknown
					}),_retriable_edges.end());
	if (edge_added) {
		if (_rrt_collision_visualization_pub.getNumSubscribers() > 0) {
			for (auto edge_marker : new_edge_markers) {
				_node_edges.markers.push_back(edge_marker);
			}
			_rrt_collision_visualization_pub.publish(_node_edges);
		}
		_vis_map.data = tmp_vis_map_data;
		_visualization_pub.publish(_vis_map);
	}
}

void CollisionChecker::calculateNextInflatedCircleLinesOffset() {
	double new_radius = -1;
	std::vector<CircleLine> prevOffsetSet;
	if (_inflated_ring_lines_offsets.empty()) { //first inflation ring offset
		new_radius = ceil(_robot_radius / _grid_map_resolution)
				* _grid_map_resolution + _grid_map_resolution; //round up radius to next full grid map tile
		prevOffsetSet = _circle_lines_offset;
	} else if (_inflated_ring_lines_offsets.back().first + _grid_map_resolution
			> _local_graph_radius) {
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
	_path_box_distance_thresholds.push_back(
			std::make_pair(new_radius,
					sqrt(pow(new_radius, 2) - pow(_robot_width / 2, 2))));
}

int CollisionChecker::isSetInCollision(double center_x, double center_y,
		bool &fixed_direction, nav_msgs::OccupancyGrid &map,
		std::vector<int8_t> &vis_map, std::vector<CircleLine> offsets,
		int &cost, int &tiles, int &collision, bool recalculate) {
	unsigned int map_x, map_y;
	if (!worldToMap(center_x, center_y, map_x, map_y, map))
		return Directions::none;
	int new_cost = cost;
	int new_tiles = tiles;
	std::vector<int8_t> tmp_vis_map = vis_map;
	int direction = Directions::center;
	for (auto it : offsets) { //always starts at highest x offset and ends at highest y offset
		if (recalculate || it.x_start == 0) { //circle
			if (isLineInCollision(map_x - it.x_offset, map_x + it.x_offset,
					map_y + it.y_offset, map, tmp_vis_map, new_cost, new_tiles,
					collision)) {
				return Directions::none;
			}
			if (it.y_offset != 0) { //y offset is only 0 in the center slice, otherwise symmetrical
				if (isLineInCollision(map_x - it.x_offset, map_x + it.x_offset,
						map_y - it.y_offset, map, tmp_vis_map, new_cost,
						new_tiles, collision)) {
					return Directions::none;
				}
			}
		} else { //ring
			const int max_x_offset = offsets.begin()->x_offset;
			const int max_y_offset = offsets.end()->x_offset;
			if (isLineInCollision(map_x + it.x_start, map_x + it.x_offset,
					map_y + it.y_offset, map, tmp_vis_map, new_cost, new_tiles,
					collision)) //northwest quadrant
				if (!checkDirection(direction, fixed_direction,
						it.x_offset == max_x_offset ? Directions::south :
						it.y_offset == max_y_offset ?
								Directions::east : Directions::southeast))
					return Directions::none;
			if (isLineInCollision(map_x - it.x_offset, map_x - it.x_start,
					map_y + it.y_offset, map, tmp_vis_map, new_cost, new_tiles,
					collision)) //southwest quadrant
				if (!checkDirection(direction, fixed_direction,
						it.x_offset == max_x_offset ? Directions::north :
						it.y_offset == max_y_offset ?
								Directions::east : Directions::northeast))
					return Directions::none;
			if (it.y_offset != 0) { //y offset is only 0 in the center slice, otherwise symmetrical
				if (isLineInCollision(map_x + it.x_start, map_x + it.x_offset,
						map_y - it.y_offset, map, tmp_vis_map, new_cost,
						new_tiles, collision)) //northeast quadrant
					if (!checkDirection(direction, fixed_direction,
							it.x_offset == max_x_offset ? Directions::south :
							it.y_offset == max_y_offset ?
									Directions::west : Directions::southwest))
						return Directions::none;
				if (isLineInCollision(map_x - it.x_offset, map_x - it.x_start,
						map_y - it.y_offset, map, tmp_vis_map, new_cost,
						new_tiles, collision)) //southeast quadrant
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
		int &tiles, int &collision) {
	bool fixed_direction = false;
	return isSetInCollision(center_x, center_y, fixed_direction, map, vis_map,
			_circle_lines_offset, cost, tiles, collision) != Directions::center;
}

bool CollisionChecker::checkDirection(int &current_direction, bool &fixed,
		int new_direction) {
	if (current_direction == Directions::center) { //first direction (initialization)
		current_direction = new_direction;
		return true;
	}
	const int difference = _graph_path_calculator->getAbsoluteAngleDiff(
			current_direction, new_direction);
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

bool CollisionChecker::checkMovementWithNearestNode(double &x, double &y,
		int &direction, rrg_nbv_exploration_msgs::Node &nearest_node,
		bool &fixed_direction) {
	geometry_msgs::Point new_point = movePoint(direction, x, y);
	double distance_squared = sqrt(
			pow(new_point.x - nearest_node.position.x, 2)
					+ pow(new_point.y - nearest_node.position.y, 2));
	double distance = sqrt(distance_squared);
	double distance_to_radius = distance - nearest_node.radius;
	bool move_towards_node = false;
	bool move_from_node = false;
	double distance_threshold = 0;
	if (distance_to_radius < 0) {
		move_from_node = true;
	} else {
		distance_threshold = _path_box_distance_thres / 2
				+ _path_box_distance_thresholds.at(
						(nearest_node.radius - _robot_radius)
								/ _grid_map_resolution).second;
		if (distance > distance_threshold) {
			move_towards_node = true;
		}
	}
	if (move_from_node || move_towards_node) {
		double yaw = atan2(y - nearest_node.position.y,
				x - nearest_node.position.x);
		double node_direction = yaw * 180.0 / M_PI;
		if (node_direction < 0)
			node_direction += 360;
		int direction_from_nearest_node = ((int) round(node_direction / 45))
				* 45;
		if (direction_from_nearest_node == 360)
			direction_from_nearest_node = 0;
		if (move_towards_node) { //opposite direction
			direction_from_nearest_node =
					direction_from_nearest_node + direction_from_nearest_node
							> 180 ? (-180) : 180;
		}
		if (!checkDirection(direction, fixed_direction,
				direction_from_nearest_node)) { //merge directions, if it fails, abort inflation
			return false;
		} else { //check if merged direction is acceptable
			geometry_msgs::Point merged_point = movePoint(direction, x, y);
			double distance_merged_squared = sqrt(
					pow(merged_point.x - nearest_node.position.x, 2)
							+ pow(merged_point.y - nearest_node.position.y, 2));
			double distance_merged = sqrt(distance_merged_squared);
			double distance_merged_to_radius = distance_merged
					- nearest_node.radius;
			if (distance_merged_to_radius < 0
					|| distance_merged > distance_threshold) {
				return false;
			}
		}
	}
	return true;
}

double CollisionChecker::inflateCircle(double &x, double &y, bool move_node,
		rrg_nbv_exploration_msgs::Node &nearest_node,
		nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map, int &cost,
		int &tiles, int &collision, double current_radius, double max_radius) {
	std::vector<int8_t> tmp_vis_map = vis_map;
	if (_inflated_ring_lines_offsets.empty())
		calculateNextInflatedCircleLinesOffset();
	int index = 0;
	if (current_radius > _robot_radius) {
		index = (current_radius - _robot_radius) / _grid_map_resolution;
	}
	auto it = std::next(_inflated_ring_lines_offsets.begin(), index);
	std::vector<std::pair<double, double>> previous_positions;
	previous_positions.push_back(std::make_pair(x, y));
	while (it != _inflated_ring_lines_offsets.end()) {
		if (it->first >= max_radius) {
			return current_radius;
		}
		bool fixed_direction = false;
		int direction = isSetInCollision(x, y, fixed_direction, map,
				tmp_vis_map, it->second, cost, tiles, collision);
		if (direction == Directions::none) {
			return current_radius;
		} else {
			if (move_node) {
				if (direction == Directions::center) {
					current_radius = it->first;
					if (it == --_inflated_ring_lines_offsets.end()) { //calculate next inflation ring if not reached sensor range
						calculateNextInflatedCircleLinesOffset();
					}
					index++;
					it = _inflated_ring_lines_offsets.begin() + index; //increasing vector size invalidates iterator
				} else {
					if (!checkMovementWithNearestNode(x, y, direction,
							nearest_node, fixed_direction)
							|| !moveCircle(x, y, direction, index,
									previous_positions, map, tmp_vis_map, cost,
									tiles, collision)) { //don't increment iterator, repeat this inflation for new position next iteration
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

geometry_msgs::Point CollisionChecker::movePoint(int direction, double &x,
		double &y) {
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

	return new_point;
}

bool CollisionChecker::moveCircle(double &x, double &y, int direction, int ring,
		std::vector<std::pair<double, double>> &previous_positions,
		nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map, int &cost,
		int &tiles, int &collision) {
	geometry_msgs::Point new_point = movePoint(direction, x, y);
	for (auto it = previous_positions.rbegin(); it != previous_positions.rend();
			++it) { //check previous positions
		if (it->first == new_point.x && it->second == new_point.y) {
			return false;
		}
	}
	bool fixed_direction = false;
	if (isSetInCollision(new_point.x, new_point.y, fixed_direction, map,
			vis_map,
			ring == 0 ?
					_circle_lines_offset :
					_inflated_ring_lines_offsets.at(ring - 1).second, cost,
			tiles, collision, true) == Directions::center) {
		x = new_point.x;
		y = new_point.y;
		previous_positions.push_back(std::make_pair(x, y));
		return true;
	} else {
		return false;
	}
}

bool CollisionChecker::isRectangleInCollision(double x, double y, double yaw,
		double half_height, double half_width, nav_msgs::OccupancyGrid &map,
		std::vector<int8_t> &vis_map, int &cost, int &tiles, int &collision) {
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
		if (isLineInCollision(x_start, x_end, grid_y, map, vis_map, cost, tiles,
				collision))
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
			tiles, collision))
		return true;
	return false;
}

bool CollisionChecker::isAlignedRectangleInCollision(double x, double y,
		double yaw, double half_height, double half_width,
		nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map, int &cost,
		int &tiles, int &collision) {
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
				vis_map, cost, tiles, collision))
			return true;
	}
	return false;
}

bool CollisionChecker::isLineInCollision(int x_start, int x_end, int y,
		nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map, int &cost,
		int &tiles, int &collision) {
	if (x_start < 0 || x_end > map.info.width || y < 0 || y > map.info.height) {
		return true;
	}
	for (int x = y * map.info.width + x_start; x <= y * map.info.width + x_end;
			x++) {
		if (map.data[x] == _grid_map_unknown) {
			collision = Collisions::unknown;
			return true;
		} else if (map.data[x] >= _grid_map_occupied) {
			collision = Collisions::occupied;
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

rrg_nbv_exploration_msgs::Edge CollisionChecker::constructEdge(double edge_yaw,
		double edge_length, int edge_cost, int edge_tiles, int neighbor_node,
		int new_node, double distance, rrg_nbv_exploration_msgs::Graph &rrg) {
	rrg_nbv_exploration_msgs::Edge new_edge;
	new_edge.yaw = (int) ((edge_yaw * 180.0 / M_PI));
	if (new_edge.yaw < 0)
		new_edge.yaw += 360;
	if (new_node < neighbor_node) //reverse edge yaw (always from node with smaller index to larger index
		new_edge.yaw += new_edge.yaw > 180 ? (-180) : 180;
	new_edge.traversability_cost = edge_length > 0 ? edge_cost : 0.0;
	new_edge.traversability_weight = edge_length > 0 ? edge_tiles : 0.0;
	new_edge.first_node = std::min(new_node, neighbor_node);
	new_edge.second_node = std::max(new_node, neighbor_node);
	new_edge.length = distance;
	new_edge.inactive = false;
	return new_edge;
}

bool CollisionChecker::checkEngulfing(geometry_msgs::Point &point,
		int &nearest_node, rrg_nbv_exploration_msgs::Graph &rrg) {
	double min_distance = -1;
	double min_distance_to_radius = -1;
	nearest_node = -1;
	std::vector<std::pair<int, double>> nearby_nodes =
			_graph_searcher->searchInRadius(point,
					pow(rrg.largest_node_radius, 2));
	for (auto it : nearby_nodes) {
		if (rrg.nodes[it.first].squared_radius > it.second) {
			return false;
		} else {
			double distance = sqrt(it.second) - rrg.nodes[it.first].radius;
			if (distance < min_distance_to_radius) {
				nearest_node = it.first;
				min_distance_to_radius = distance;
				min_distance = sqrt(it.second);
			}
		}
	}
	if (nearest_node == -1) { //if no nearby nodes found, find nearest neighbor to move sample closer
		_graph_searcher->findNearestNeighbour(point, min_distance,
				nearest_node);
		min_distance = sqrt(min_distance);
	}
	int index = ((rrg.nodes[nearest_node].radius - _robot_radius)
			/ _grid_map_resolution) - 1;
	double distance_threshold;
	distance_threshold =
			index >= 0 ?
					_path_box_distance_thres / 2
							+ _path_box_distance_thresholds.at(index).second :
					_path_box_distance_thres;
	if (min_distance > distance_threshold) { //move sample point closer to nearest node to enable connection
		point.x = rrg.nodes[nearest_node].position.x
				- (distance_threshold
						* (rrg.nodes[nearest_node].position.x - point.x)
						/ min_distance);
		point.y = rrg.nodes[nearest_node].position.y
				- (distance_threshold
						* (rrg.nodes[nearest_node].position.y - point.y)
						/ min_distance);
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

void CollisionChecker::findBestConnectionForNode(
		rrg_nbv_exploration_msgs::Graph &rrg,
		rrg_nbv_exploration_msgs::Node &node, geometry_msgs::Pose &robot_pos,
		bool new_node, std::vector<rrg_nbv_exploration_msgs::Edge> edges) {
	std::vector<rrg_nbv_exploration_msgs::Edge> node_edges;
	if (!new_node) { //get edges from given existing node
		for (auto i : node.edges) {
			node_edges.push_back(rrg.edges[i]);
		}
	} else { //use given edges for new node
		node_edges = edges;
	}
	for (auto &edge : node_edges) {
		int neighbor_node_index =
				edge.first_node == node.index ?
						edge.second_node : edge.first_node;
		if (new_node) { //store edges in RRG and references in the particular nodes
			insertEdgeInRrg(edge, rrg);
			node.edges.push_back(edge.index);
			node.edge_counter++;
			rrg.nodes[neighbor_node_index].edges.push_back(edge.index);
			rrg.nodes[neighbor_node_index].edge_counter++;
		}
		if (rrg.nodes[neighbor_node_index].status
				!= rrg_nbv_exploration_msgs::Node::FAILED
				&& !std::isinf(rrg.nodes[neighbor_node_index].cost_function)) {
			//only evaluate edge if it is not to a failed or unreachable node
			rrg_nbv_exploration_msgs::Node updated_node; //calculate potential cost if connected via this edge
			updated_node.distance_to_robot =
					rrg.nodes[neighbor_node_index].distance_to_robot
							+ edge.length;
			updated_node.path_to_robot =
					rrg.nodes[neighbor_node_index].path_to_robot;
			updated_node.path_to_robot.push_back(node.index);
			updated_node.radii_to_robot =
					rrg.nodes[neighbor_node_index].radii_to_robot + node.radius;
			if (neighbor_node_index == rrg.nearest_node) {
				//calculate heading to new node directly from robot if edge to nearest node
				int yaw_to_start_node =
						(int) (((atan2(node.position.y - robot_pos.position.y,
								node.position.x - robot_pos.position.x) * 180.0
								/ M_PI)));
				if (yaw_to_start_node < 0)
					yaw_to_start_node += 360;

				updated_node.heading_in = yaw_to_start_node;
			} else
				updated_node.heading_in = edge.yaw;

			updated_node.heading_change_to_robot =
					rrg.nodes[neighbor_node_index].heading_change_to_robot
							+ _graph_path_calculator->getAbsoluteAngleDiff(
									rrg.nodes[neighbor_node_index].heading_in,
									updated_node.heading_in); //calculate heading cost from current to neighbor node
			updated_node.heading_change_to_robot_best_view =
					_graph_path_calculator->calculateHeadingChangeToBestView(
							updated_node);
			updated_node.traversability_cost_to_robot =
					rrg.nodes[neighbor_node_index].traversability_cost_to_robot
							+ node.traversability_cost
							+ edge.traversability_cost;
			updated_node.traversability_weight_to_robot =
					rrg.nodes[neighbor_node_index].traversability_weight_to_robot
							+ node.traversability_weight
							+ edge.traversability_weight;
			updated_node.cost_function =
					_graph_path_calculator->calculateCostFunction(updated_node);
			if (updated_node.cost_function < node.cost_function) {
				node.distance_to_robot = updated_node.distance_to_robot;
				node.path_to_robot = updated_node.path_to_robot;
				node.heading_in = updated_node.heading_in;
				node.heading_change_to_robot =
						updated_node.heading_change_to_robot;
				node.heading_change_to_robot_best_view =
						updated_node.heading_change_to_robot_best_view;
				node.radii_to_robot = updated_node.radii_to_robot;
				node.traversability_cost_to_robot =
						updated_node.traversability_cost_to_robot;
				node.traversability_weight_to_robot =
						updated_node.traversability_weight_to_robot;
				node.cost_function = updated_node.cost_function;
			}
		}
	}
}

void CollisionChecker::addAvailableNode(int node) {
	_available_nodes.insert(node);
}

void CollisionChecker::addAvailableEdge(int edge) {
	_available_edges.insert(edge);
}

int CollisionChecker::getAvailableNodeIndex(
		rrg_nbv_exploration_msgs::Graph &rrg) {
	if (!_available_nodes.empty()) {
		ROS_INFO_STREAM("Available node index " << *_available_nodes.begin());
		return *_available_nodes.begin();
	} else {
		ROS_INFO_STREAM("Node index at end " << rrg.node_counter);
		return rrg.node_counter;
	}
}

void CollisionChecker::insertNodeInRrg(rrg_nbv_exploration_msgs::Node &node,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	if (!_available_nodes.empty()) {
		ROS_INFO_STREAM("Insert node at " << *_available_nodes.begin());
		rrg.nodes.at(*_available_nodes.begin()) = node;
		_available_nodes.erase(_available_nodes.begin());
	} else {
		ROS_INFO_STREAM("Push node at " << rrg.node_counter);
		rrg.nodes.push_back(node);
		rrg.node_counter++;
	}
}

void CollisionChecker::insertEdgeInRrg(rrg_nbv_exploration_msgs::Edge &edge,
		rrg_nbv_exploration_msgs::Graph &rrg) {
	if (!_available_edges.empty()) {
		ROS_INFO_STREAM("Insert edge at " << *_available_edges.begin());
		edge.index = *_available_edges.begin();
		edge.inactive = false;
		rrg.edges.at(*_available_edges.begin()) = edge;
		_available_edges.erase(_available_edges.begin());
	} else {
		ROS_INFO_STREAM("Push edge at " << rrg.edge_counter);
		edge.index = rrg.edge_counter++;
		rrg.edges.push_back(edge);
	}
}

void CollisionChecker::removeDeletedAvailableNodes(int node_counter) {
	int removals = 0;
	for (auto it = _available_nodes.begin(); it != _available_nodes.end();) {
		if (*it >= node_counter) {
			it = _available_nodes.erase(it);
			removals++;
		} else {
			++it;
		}
	}
	ROS_WARN_STREAM(
			"Removed " << removals << " available nodes above equal index "<< node_counter);
}

void CollisionChecker::removeDeletedAvailableEdges(int edge_counter) {
	int removals = 0;
	for (auto it = _available_edges.begin(); it != _available_edges.end();) {
		if (*it >= edge_counter) {
			it = _available_edges.erase(it);
			removals++;
		} else {
			++it;
		}
	}
	ROS_WARN_STREAM(
			"Removed " << removals << " available edges above equal index "<< edge_counter);
}

void CollisionChecker::removeRetriableEdgesForNode(int node) {
	_retriable_edges.erase(
			std::remove_if(_retriable_edges.begin(), _retriable_edges.end(),
					[node](rrg_nbv_exploration_msgs::Edge &edge) {
						return (edge.first_node == node
								|| edge.second_node == node); //remove retriable edge if connecting to the newly inactive node
					}),_retriable_edges.end());
}

bool CollisionChecker::checkConnectionToFrontier(
		rrg_nbv_exploration_msgs::Graph &rrg, int node,
		geometry_msgs::Point frontier, double distance) {
	nav_msgs::OccupancyGrid map = *_occupancy_grid;
	std::vector<int8_t> tmp_vis_map_data = _vis_map.data;
	std::vector<visualization_msgs::Marker> new_edge_markers;
	int edge_cost = 0, edge_tiles = 0, edge_collision = Collisions::empty;
	geometry_msgs::Point edge_center;
	edge_center.x = (rrg.nodes[node].position.x + frontier.x) / 2;
	edge_center.y = (rrg.nodes[node].position.y + frontier.y) / 2;
	double edge_length = distance - _path_box_distance_thres;
	double edge_yaw = atan2(frontier.y - rrg.nodes[node].position.y,
			frontier.x - rrg.nodes[node].position.x);
	fmod(edge_yaw, M_PI / 2) == 0 ?
			isAlignedRectangleInCollision(edge_center.x, edge_center.y,
					edge_yaw, edge_length, _robot_width / 2, map,
					tmp_vis_map_data, edge_cost, edge_tiles, edge_collision) :
			isRectangleInCollision(edge_center.x, edge_center.y, edge_yaw,
					edge_length, _robot_width / 2, map, tmp_vis_map_data,
					edge_cost, edge_tiles, edge_collision);
	if (edge_collision == Collisions::empty) {
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

void CollisionChecker::visualizeNode(rrg_nbv_exploration_msgs::Node &node,
		int failed) {
	visualization_msgs::Marker node_point;
	node_point.header.frame_id = "/map";
	node_point.header.stamp = ros::Time();
	node_point.ns = "rrt_collision_vis";
	node_point.id = _inflation_active ? node.index : _marker_id++;
	node_point.action = visualization_msgs::Marker::ADD;
	node_point.pose.orientation.w = 1.0;
	node_point.type = visualization_msgs::Marker::CYLINDER;
	node_point.scale.x = 2 * node.radius;
	node_point.scale.y = 2 * node.radius;
	node_point.scale.z = 0.01;
	if (failed > 0)
		node_point.color.r = 1.0f;
	if (failed == 0 || failed == 3)
		node_point.color.g = 1.0f;
	if (failed == 2)
		node_point.color.g = 0.5f;
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
