#include <rrg_nbv_exploration/CollisionChecker.h>

namespace rrg_nbv_exploration {
CollisionChecker::CollisionChecker() {
	ros::NodeHandle private_nh("~");
	private_nh.param("robot_radius", _robot_radius, 1.0);
	private_nh.param("sensor_max_range", _sensor_range, 5.0);
	private_nh.param("check_init_position", _check_init_position, false);
	private_nh.param("grid_map_resolution", _grid_map_resolution, 0.05);
	std::string occupancy_grid_topic;
	private_nh.param<std::string>("occupancy_grid_topic", occupancy_grid_topic,
			"map");
	ros::NodeHandle nh("rne");
	_occupancy_grid_sub = _nh.subscribe(occupancy_grid_topic, 1,
			&CollisionChecker::occupancyGridCallback, this);
	_visualization_pub = nh.advertise<nav_msgs::OccupancyGrid>(
			"rrg_collision_map", 1);
	_rrt_collision_visualization_pub = nh.advertise<
			visualization_msgs::MarkerArray>("rrg_collision_vis", 1000);
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
		std::vector<CircleLine> offsets) {
	unsigned int map_x, map_y;
	if (!worldToMap(center_x, center_y, map_x, map_y, map))
		return true;
	for (auto it : offsets) {
		if (it.x_start == 0) {
			if (isLineInCollision(map_x - it.x_offset, map_x + it.x_offset,
					map_y + it.y_offset, map, vis_map))
				return true;
			if (it.y_offset != 0)
				if (isLineInCollision(map_x - it.x_offset, map_x + it.x_offset,
						map_y - it.y_offset, map, vis_map))
					return true;
		} else {
			if (isLineInCollision(map_x + it.x_start, map_x + it.x_offset,
					map_y + it.y_offset, map, vis_map))
				return true;
			if (isLineInCollision(map_x - it.x_offset, map_x - it.x_start,
					map_y + it.y_offset, map, vis_map))
				return true;
			if (it.y_offset != 0) {
				if (isLineInCollision(map_x + it.x_start, map_x + it.x_offset,
						map_y - it.y_offset, map, vis_map))
					return true;
				if (isLineInCollision(map_x - it.x_offset, map_x - it.x_start,
						map_y - it.y_offset, map, vis_map))
					return true;
			}
		}
	}
	return false;
}

bool CollisionChecker::isCircleInCollision(double center_x, double center_y,
		nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map) {
	return isSetInCollision(center_x, center_y, map, vis_map,
			_circle_lines_offset);
}

double CollisionChecker::inflateCircle(double x, double y,
		nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map) {
	double current_radius = ceil(_robot_radius / _grid_map_resolution)
			* _grid_map_resolution; //round up radius to next full grid map tile
	int prevOffsetIndex = -1;
	std::vector<int8_t> tmp_vis_map = vis_map;
	for (auto it : _inflated_ring_lines_offsets) { //iterate over already existing offsets first
		if (isSetInCollision(x, y, map, tmp_vis_map, it.second)) {
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
		if (isSetInCollision(x, y, map, tmp_vis_map, newOffsetSet)) {
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

bool CollisionChecker::isLineInCollision(int x_start, int x_end, int y,
		nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map) {
	if (x_start < 0 || x_end > map.info.width || y < 0 || y > map.info.height)
		return true;
	for (int x = y * map.info.width + x_start; x <= y * map.info.width + x_end;
			x++) {
		if (map.data[x] == -1 || map.data[x] >= 100) {
			return true;
		} else {
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
	_node_points.markers.clear();
	if (_check_init_position) {
		std::vector<int8_t> tmp_vis_map_data = _vis_map.data;
		if (!isCircleInCollision(position.x, position.y, map,
				tmp_vis_map_data)) {
			_vis_map.data = tmp_vis_map_data;
			_visualization_pub.publish(_vis_map);
			return true;
		}
		return false;
	} else
		return true;
}

bool CollisionChecker::steer(rrg_nbv_exploration_msgs::Node &new_node,
		rrg_nbv_exploration_msgs::Node &nearest_node,
		geometry_msgs::Point rand_sample) {
	nav_msgs::OccupancyGrid map = _occupancy_grid;
	bool no_collision = false;
	_vis_map.header.stamp = ros::Time::now();
	_vis_map.info.map_load_time = ros::Time::now();
	std::vector<int8_t> tmp_vis_map_data = _vis_map.data;
	double yaw = atan2(rand_sample.y - nearest_node.position.y,
			rand_sample.x - nearest_node.position.x);
	if (!isCircleInCollision(rand_sample.x, rand_sample.y, map,
			tmp_vis_map_data)) {
		new_node.radius = inflateCircle(rand_sample.x, rand_sample.y, map,
				tmp_vis_map_data);
		new_node.squared_radius = pow(new_node.radius, 2);
		new_node.position.x = rand_sample.x;
		new_node.position.y = rand_sample.y;
		new_node.position.z = rand_sample.z;
		new_node.status = rrg_nbv_exploration_msgs::Node::INITIAL;
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
