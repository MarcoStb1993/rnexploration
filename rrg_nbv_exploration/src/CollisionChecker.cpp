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

bool CollisionChecker::isSetInCollision(double center_x, double center_y,
		nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map,
		std::vector<CircleLine> offsets, int &cost, int &tiles) {
	unsigned int map_x, map_y;
	if (!worldToMap(center_x, center_y, map_x, map_y, map))
		return true;
	for (auto it : offsets) {
		if (it.x_start == 0) {
			if (isLineInCollision(map_x - it.x_offset, map_x + it.x_offset,
					map_y + it.y_offset, map, vis_map, cost, tiles))
				return true;
			if (it.y_offset != 0)
				if (isLineInCollision(map_x - it.x_offset, map_x + it.x_offset,
						map_y - it.y_offset, map, vis_map, cost, tiles))
					return true;
		} else {
			if (isLineInCollision(map_x + it.x_start, map_x + it.x_offset,
					map_y + it.y_offset, map, vis_map, cost, tiles))
				return true;
			if (isLineInCollision(map_x - it.x_offset, map_x - it.x_start,
					map_y + it.y_offset, map, vis_map, cost, tiles))
				return true;
			if (it.y_offset != 0) {
				if (isLineInCollision(map_x + it.x_start, map_x + it.x_offset,
						map_y - it.y_offset, map, vis_map, cost, tiles))
					return true;
				if (isLineInCollision(map_x - it.x_offset, map_x - it.x_start,
						map_y - it.y_offset, map, vis_map, cost, tiles))
					return true;
			}
		}
	}
	return false;
}

bool CollisionChecker::isCircleInCollision(double center_x, double center_y,
		nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map, int &cost,
		int &tiles) {
	return isSetInCollision(center_x, center_y, map, vis_map,
			_circle_lines_offset, cost, tiles);
}

double CollisionChecker::inflateCircle(double x, double y,
		nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map, int &cost,
		int &tiles) {
	//TODO: move away from obstacles on collision and repeat inflation
	double current_radius = ceil(_robot_radius / _grid_map_resolution)
			* _grid_map_resolution; //round up radius to next full grid map tile
	int prevOffsetIndex = -1;
	std::vector<int8_t> tmp_vis_map = vis_map;
	for (auto it : _inflated_ring_lines_offsets) { //iterate over already existing offsets first
		if (isSetInCollision(x, y, map, tmp_vis_map, it.second, cost, tiles)) {
			return current_radius;
		}
		vis_map = tmp_vis_map;
		current_radius = it.first;
		prevOffsetIndex++;
	}
	current_radius += _grid_map_resolution;
	for (double radius = current_radius; radius < _sensor_range; radius +=
			_grid_map_resolution) { //check in rings with increasing radius and save the offsets
		std::vector<CircleLine> newOffsetSet = calculateCircleLinesOffset(
				radius);
		std::vector<CircleLine> prevOffsetSet =
				prevOffsetIndex >= 0 ?
						_inflated_ring_lines_offsets[prevOffsetIndex].second :
						_circle_lines_offset;
		for (int i = 0; i < prevOffsetSet.size(); i++) { //prev offset set size must be smaller equals new offset set
			newOffsetSet[i].x_start = prevOffsetSet[i].x_offset + 1;
		}
		if (isSetInCollision(x, y, map, tmp_vis_map, newOffsetSet, cost,
				tiles)) {
			return current_radius;
		}
		vis_map = tmp_vis_map;
		current_radius = radius;
		_inflated_ring_lines_offsets.push_back(
				std::make_pair(current_radius, newOffsetSet));
		prevOffsetIndex++;
	}
	return current_radius;
}

bool CollisionChecker::isRectangleInCollision(double x, double y, double yaw,
		double half_height, double half_width, nav_msgs::OccupancyGrid &map,
		std::vector<int8_t> &vis_map, int &cost, int &tiles) {
	std::array<MapPoint, 4> map_corners;
	std::array<GridPoint, 4> grid_corners;
	double cos_yaw, sin_yaw;
	if (yaw < -PI_HALF) {
		yaw += M_PI; //	combine mirrored case (-PI < yaw < -PI/2 => 0 < yaw < PI/2)
//		ROS_INFO_STREAM("Add PI to yaw, new yaw= " << yaw);
	} else if (yaw > PI_HALF) {
		yaw -= M_PI; //	combine mirrored case (PI > yaw > PI/2 => 0 > yaw > -PI/2)
//		ROS_INFO_STREAM("Subtract PI from yaw, new yaw= " << yaw);
	}
	cos_yaw = cos(yaw);
	sin_yaw = sin(yaw);
	double width_cos = half_width * cos_yaw;
	double width_sin = half_width * sin_yaw;
	double height_cos = half_height * cos_yaw;
	double height_sin = half_height * sin_yaw;
	if (yaw > 0 && yaw < PI_HALF) { //height from top left to bottom right
//		ROS_INFO_STREAM("height from top left to bottom right");
		map_corners[0] = { x + height_cos - width_sin, y + width_cos
				+ height_sin }; //left corner
		map_corners[1] = { x - height_cos - width_sin, y + width_cos
				- height_sin }; //bottom corner
		map_corners[2] = { x + height_cos + width_sin, y - width_cos
				+ height_sin }; //top corner
		map_corners[3] = { x - height_cos + width_sin, y - width_cos
				- height_sin }; //right corner
	} else { // yaw >  -PI_HALF && yaw < 0 (height from bottom left to top right)
//		ROS_INFO_STREAM("height from bottom left to top right");
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
//	ROS_INFO_STREAM(
//			"Rectangle (map) : c0=(" << map_corners[0].x << ", " << map_corners[0].y << "), c1=(" << map_corners[1].x << ", " << map_corners[1].y << "), c2=(" << map_corners[2].x << ", " << map_corners[2].y << "), c3=(" << map_corners[3].x << ", " << map_corners[3].y << ")");

	if (!worldToMap(map_corners[0].x, map_corners[0].y, grid_corners[0].x,
			grid_corners[0].y, map)
			|| !worldToMap(map_corners[1].x, map_corners[1].y,
					grid_corners[1].x, grid_corners[1].y, map)
			|| !worldToMap(map_corners[2].x, map_corners[2].y,
					grid_corners[2].x, grid_corners[2].y, map)
			|| !worldToMap(map_corners[3].x, map_corners[3].y,
					grid_corners[3].x, grid_corners[3].y, map))
		return true;
//	ROS_INFO_STREAM(
//			"Rectangle (grid): c0=(" << grid_corners[0].x << ", " << grid_corners[0].y << "), c1=(" << grid_corners[1].x << ", " << grid_corners[1].y << "), c2=(" << grid_corners[2].x << ", " << grid_corners[2].y << "), c3=(" << grid_corners[3].x << ", " << grid_corners[3].y << ")");
//	ROS_INFO_STREAM("M down: " << gradient_down << ", M up: " << gradient_up);
	unsigned int grid_x = grid_corners[3].x;
	unsigned int grid_y = grid_corners[3].y;
	double gradient_top = gradient_up;
	double gradient_bot = gradient_down;
	double iterator_y = (floor(map_corners[3].y / _grid_map_resolution) + 1.0)
			* _grid_map_resolution;
	double iterator_x = (floor(map_corners[3].x / _grid_map_resolution) + 0.5)
			* _grid_map_resolution;
//	ROS_INFO_STREAM("s=(" << iterator_x << ", " << iterator_y << ")");
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
//			ROS_INFO_STREAM(
//					"Top corner reached sy="<< iterator_y << " top y=" << map_corners[2].y);
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
//			ROS_INFO_STREAM(
//					"Bot corner reached sy="<< iterator_y << " bot y=" << map_corners[1].y);
			x_start = grid_corners[1].x;
		} else
			x_start = grid_x + (int) round(offset_x_bot);
//		ROS_INFO_STREAM(
//				"xtop=(" << iterator_x_top << ","<< iterator_y << "), offset: " << offset_x_top);
//		ROS_INFO_STREAM(
//				"xbot=(" << iterator_x_bot << ","<< iterator_y << "), offset: " << offset_x_bot);
//		ROS_INFO_STREAM(
//				"Line start=(" << x_start << ","<<grid_y <<") line end=(" << x_end << "," << grid_y << ")");
		if (isLineInCollision(x_start, x_end, grid_y, map, vis_map, cost,
				tiles))
			return true;
		if (iterator_y < map_corners[0].y) {
			offset_x_top += gradient_top;
			offset_x_bot += gradient_bot;
		}
		grid_y += 1;
		iterator_y += _grid_map_resolution;
//		ROS_INFO_STREAM("map y: " << iterator_y << ", grid y: " << grid_y);
	}
//	ROS_INFO_STREAM(
//			"Left corner reached sy="<< iterator_y << " left y=" << map_corners[0].y);
//	ROS_INFO_STREAM(
//			"Line start=(" << grid_x + (int) round(offset_x_bot) << ","<<grid_y <<") line end=(" << grid_x + (int) round(offset_x_top) << "," << grid_y << ")");
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
//	ROS_INFO_STREAM(
//			"Aligned Rectangle (map) : c0=(" << map_corners[0].x << ", " << map_corners[0].y << ", c1=(" << map_corners[1].x << ", " << map_corners[1].y << ", c2=(" << map_corners[2].x << ", " << map_corners[2].y << ", c3=(" << map_corners[3].x << ", " << map_corners[3].y << ")");

	if (!worldToMap(map_corners[0].x, map_corners[0].y, grid_corners[0].x,
			grid_corners[0].y, map)
			|| !worldToMap(map_corners[1].x, map_corners[1].y,
					grid_corners[1].x, grid_corners[1].y, map))
		return true;
//	ROS_INFO_STREAM(
//			"Aligned Rectangle (grid): c0=(" << grid_corners[0].x << ", " << grid_corners[0].y << ", c1=(" << grid_corners[1].x << ", " << grid_corners[1].y << ", c2=(" << grid_corners[2].x << ", " << grid_corners[2].y << ", c3=(" << grid_corners[3].x << ", " << grid_corners[3].y << ")");

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
			vis_map[x] = 0;
		}
	}
	return false;
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

bool CollisionChecker::initialize(geometry_msgs::Point position) {
	nav_msgs::OccupancyGrid map = _occupancy_grid;
	_vis_map.header.stamp = ros::Time::now();
	_vis_map.info.map_load_time = ros::Time::now();
	initVisMap(map);
	precalculateCircleLinesOffset();
	_node_edges.markers.clear();
	_node_points.markers.clear();
	if (_check_init_position) {
		std::vector<int8_t> tmp_vis_map_data = _vis_map.data;
		int node_cost = 0, node_tiles = 0;
		if (!isCircleInCollision(position.x, position.y, map, tmp_vis_map_data,
				node_cost, node_tiles)) {
			_vis_map.data = tmp_vis_map_data;
			_visualization_pub.publish(_vis_map);
			return true;
		}
		return false;
	} else
		return true;
}

bool CollisionChecker::steer(rrg_nbv_exploration_msgs::Node &new_node,
		rrg_nbv_exploration_msgs::Edge &new_edge,
		rrg_nbv_exploration_msgs::Node &nearest_node,
		geometry_msgs::Point rand_sample, double distance, double check_node) {
	nav_msgs::OccupancyGrid map = _occupancy_grid;
	bool no_collision = false;
	_vis_map.header.stamp = ros::Time::now();
	_vis_map.info.map_load_time = ros::Time::now();
	std::vector<int8_t> tmp_vis_map_data = _vis_map.data;
	double yaw = atan2(rand_sample.y - nearest_node.position.y,
			rand_sample.x - nearest_node.position.x);
	int edge_cost = 0, edge_tiles = 0;
	bool edge_free = (
			distance > _path_box_distance_thres ?
					fmod(yaw, M_PI / 2) == 0 ?
							!isAlignedRectangleInCollision(
									(nearest_node.position.x + rand_sample.x)
											/ 2,
									(nearest_node.position.y + rand_sample.y)
											/ 2, yaw,
									(distance - _path_box_distance_thres) / 2,
									_robot_width / 2, map, tmp_vis_map_data,
									edge_cost, edge_tiles) :
							!isRectangleInCollision(
									(nearest_node.position.x + rand_sample.x)
											/ 2,
									(nearest_node.position.y + rand_sample.y)
											/ 2, yaw,
									(distance - _path_box_distance_thres) / 2,
									_robot_width / 2, map, tmp_vis_map_data,
									edge_cost, edge_tiles)
					:
					true);
	int node_cost = 0, node_tiles = 0;
	bool node_free =
			check_node ?
					!isCircleInCollision(rand_sample.x, rand_sample.y, map,
							tmp_vis_map_data, node_cost, node_tiles) :
					true;
	if (edge_free && node_free) {
		if (check_node) {
			if (_inflation_active) {
				new_node.radius = inflateCircle(rand_sample.x, rand_sample.y,
						map, tmp_vis_map_data, node_cost, node_tiles);
				new_node.squared_radius = pow(new_node.radius, 2);
			} else {
				new_node.radius = _robot_radius;
				new_node.squared_radius = pow(_robot_radius, 2);
			}
			new_node.position.x = rand_sample.x;
			new_node.position.y = rand_sample.y;
			new_node.position.z = rand_sample.z;
			new_node.status = rrg_nbv_exploration_msgs::Node::INITIAL;
			new_node.traversability_cost = ((double) node_cost
					/ (double) node_tiles) / (double) (_grid_map_occupied - 1); //average cost/tile normalized to [0,1]
		}
		new_edge.yaw = (int) (yaw * 180.0 / M_PI);
		if (new_edge.yaw < 0)
			new_edge.yaw += 360;
		new_edge.traversability_cost =
				distance > _path_box_distance_thres ?
						((double) edge_cost / (double) edge_tiles)
								/ (double) (_grid_map_occupied - 1) :
						0.0; //average cost/tile normalized to [0,1]
		_vis_map.data = tmp_vis_map_data;
		_visualization_pub.publish(_vis_map);
		if (_rrt_collision_visualization_pub.getNumSubscribers() > 0) {
			visualization_msgs::Marker node_point;

			node_point.header.frame_id = "/map";
			node_point.header.stamp = ros::Time();
			node_point.ns = "rrt_collision_vis";
			node_point.id = 2 * _node_points.markers.size();
			node_point.action = visualization_msgs::Marker::ADD;
			node_point.pose.orientation.w = 1.0;
			node_point.type = visualization_msgs::Marker::CYLINDER;
			node_point.scale.x = 2 * new_node.radius;
			node_point.scale.y = 2 * new_node.radius;
			node_point.scale.z = 0.01;
			node_point.color.g = 1.0f;
			node_point.color.a = 0.5f;
			node_point.pose.position.x = rand_sample.x;
			node_point.pose.position.y = rand_sample.y;
			node_point.pose.position.z = 0.005;
			_node_points.markers.push_back(node_point);
			_rrt_collision_visualization_pub.publish(_node_points);
		}
		return true;
	}
	return false;
}

void CollisionChecker::occupancyGridCallback(
		const nav_msgs::OccupancyGrid::ConstPtr &map_msg) {
	_occupancy_grid = *map_msg;
}

int CollisionChecker::getIndex(int x, int y) {
	int sx = _occupancy_grid.info.width;
	return y * sx + x;
}

void CollisionChecker::occupancyGridUpdatesCallback(
		const map_msgs::OccupancyGridUpdate::ConstPtr &map_msg) {
	int index = 0;
	for (int y = map_msg->y; y < map_msg->y + map_msg->height; y++) {
		for (int x = map_msg->x; x < map_msg->x + map_msg->width; x++) {
			_occupancy_grid.data[getIndex(x, y)] = map_msg->data[index++];
		}
	}
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

}
