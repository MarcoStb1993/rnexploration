#include "rrt_nbv_exploration/CollisionChecker.h"

namespace rrt_nbv_exploration {
CollisionChecker::CollisionChecker() {
	ros::NodeHandle private_nh("~");
	private_nh.param("robot_width", _robot_width, 1.0);
	private_nh.param("robot_radius", _robot_radius, 1.0);
	private_nh.param("visualize_collision", _visualize_collision, false);
	std::string occupancy_grid_topic;
	private_nh.param<std::string>("occupancy_grid_topic", occupancy_grid_topic,
			"map");
	ros::NodeHandle nh("rne");
	_occupancy_grid_sub = _nh.subscribe(occupancy_grid_topic, 1,
			&CollisionChecker::occupancyGridCallback, this);
	if (_visualize_collision) {
//		_collision_visualization = nh.advertise<visualization_msgs::Marker>(
//				"steering_visualization", 10);
		_visualization_pub = nh.advertise<nav_msgs::OccupancyGrid>(
				"collision_visualization", 1);
	}
	_path_box_distance_thres = 2
			* sqrt(pow(_robot_radius, 2) - pow(_robot_width / 2, 2));
	_init_vis_map = false;
}

bool CollisionChecker::isCircleInCollision(double center_x, double center_y,
		nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map) {
	unsigned int map_x, map_y;
	if (!worldToMap(center_x, center_y, map_x, map_y, map))
		return true;
	unsigned int radius = (int) (_robot_radius / map.info.resolution);
//	ROS_INFO_STREAM(
//			"Circle with center " << center_x << ", " << center_y << ": " << map_x << ", " << map_y << ", radius: " << radius);
	int x = radius, y = 0, err = 0;
	bool collision = false;
	while (x >= y) {
		if (isLineInCollision(map_x - x, map_x + x, map_y + y, map, vis_map))
			return true; //collision = true;
		if (isLineInCollision(map_x - x, map_x + x, map_y - y, map, vis_map))
			return true; //collision = true; //return true;
		if (isLineInCollision(map_x - y, map_x + y, map_y + x, map, vis_map))
			return true; //collision = true; //return true;
		if (isLineInCollision(map_x - y, map_x + y, map_y - x, map, vis_map))
			return true; //collision = true; //return true;
		if (err <= 0) {
			y += 1;
			err += 2 * y + 1;
		} else {
			x -= 1;
			err -= 2 * x + 1;
		}
	}
	return false; //return collision; //false;
}

bool CollisionChecker::isRectangleInCollision(double x, double y, double yaw,
		double half_height, double half_width, nav_msgs::OccupancyGrid &map,
		std::vector<int8_t> &vis_map) {
	std::array<point, 4> corners;
	double cos_yaw, sin_yaw;
	bool collision = false;
	cos_yaw = cos(yaw);
	sin_yaw = sin(yaw);
//	ROS_INFO_STREAM(
//			std::setprecision(3) << std::fixed << "Square with center " << x << ", " << y << ": yaw: " << yaw* (180.0/3.141592653589793238463) <<", 0:" << x - half_height * cos_yaw + half_width * sin_yaw << ", " << y - half_height * sin_yaw - half_width * cos_yaw << ", 1: " << x + half_height * cos_yaw + half_width * sin_yaw << ", " << y + half_height * sin_yaw - half_width * cos_yaw << ", 2: " << x + half_height * cos_yaw - half_width * sin_yaw << ", " << y + half_height * sin_yaw + half_width * cos_yaw << ", 3: " << x - half_height * cos_yaw - half_width * sin_yaw << ", " << y - half_height * sin_yaw + half_width * cos_yaw);
	if (!worldToMap(x - half_height * cos_yaw - half_width * sin_yaw,
			y - half_height * sin_yaw + half_width * cos_yaw, corners[3].x,
			corners[3].y, map)
			|| !worldToMap(x + half_height * cos_yaw - half_width * sin_yaw,
					y + half_height * sin_yaw + half_width * cos_yaw,
					corners[2].x, corners[2].y, map)
			|| !worldToMap(x + half_height * cos_yaw + half_width * sin_yaw,
					y + half_height * sin_yaw - half_width * cos_yaw,
					corners[1].x, corners[1].y, map)
			|| !worldToMap(x - half_height * cos_yaw + half_width * sin_yaw,
					y - half_height * sin_yaw - half_width * cos_yaw,
					corners[0].x, corners[0].y, map))
		return true;
//	ROS_INFO_STREAM(
//			"Square with center " << x << ", " << y << ": 0:" << corners[0].x << ", " << corners[0].y << ", 1: " << corners[1].x << ", " << corners[1].y << ", 2: " << corners[2].x << ", " << corners[2].y << ", 3: " << corners[3].x << ", " << corners[3].y);
	if ((corners[0].x == corners[1].x && corners[1].y == corners[2].y)
			|| (corners[1].x == corners[2].x && corners[2].y == corners[3].y)) {
//		ROS_INFO("aligned");
//		ROS_INFO_STREAM(
//				"Square with center " << x << ", " << y << ": 0:" << corners[0].x << ", " << corners[0].y << ", 1: " << corners[1].x << ", " << corners[1].y << ", 2: " << corners[2].x << ", " << corners[2].y << ", 3: " << corners[3].x << ", " << corners[3].y);
		bool y0_top = corners[0].y > corners[2].y;
		bool x0_left = corners[0].x < corners[2].x;
//		ROS_INFO_STREAM("Top=x0: " << y0_top << " Left=y0: " << x0_left);
		for (unsigned int i = (y0_top ? corners[2].y : corners[0].y);
				i <= (y0_top ? corners[0].y : corners[2].y); i++) {
//			ROS_INFO_STREAM("line at x: " << i << " from " << (x0_left ? corners[0].y : corners[2].y) << " to " << (x0_left ? corners[2].y : corners[0].y));
			if (isLineInCollision(x0_left ? corners[0].x : corners[2].x,
					x0_left ? corners[2].x : corners[0].x, i, map, vis_map))
				return true; //collision = true; //return true;
		}
	} else {
		//Mid point
		auto it_bot = std::min_element(std::begin(corners), std::end(corners),
				[](const point &p1, const point &p2) {
					return p1.y < p2.y;
				});
		std::size_t bot_corner = std::distance(std::begin(corners), it_bot);
		auto it_left = std::min_element(std::begin(corners), std::end(corners),
				[](const point &p1, const point &p2) {
					return p1.x < p2.x;
				});
		std::size_t left_corner = std::distance(std::begin(corners), it_left);
		std::size_t right_corner = (left_corner + 2) % 4, top_corner =
				(bot_corner + 2) % 4;
//		ROS_INFO_STREAM(
//				"Bot: " << bot_corner << " left: " << left_corner<< " right: " << right_corner << " top: " << top_corner);
		float l_m = ((float) corners[left_corner].x
				- (float) corners[bot_corner].x)
				/ ((float) corners[left_corner].y
						- (float) corners[bot_corner].y);
		float r_m = ((float) corners[right_corner].x
				- (float) corners[bot_corner].x)
				/ ((float) corners[right_corner].y
						- (float) corners[bot_corner].y);
//		ROS_INFO_STREAM("l_m: " << l_m << " r_m: " <<r_m);
		unsigned int lr_y = corners[bot_corner].y;
		float l_x = (float) corners[bot_corner].x, r_x = l_x;
		if (isLineInCollision(l_x, r_x, lr_y, map, vis_map))
			return true; //collision = true; //return true;
		while (lr_y < corners[top_corner].y) {
			if (lr_y == corners[left_corner].y) {
				l_m = ((float) corners[top_corner].x
						- (float) corners[left_corner].x)
						/ ((float) corners[top_corner].y
								- (float) corners[left_corner].y);
				l_x = (float) corners[left_corner].x;
//				ROS_INFO_STREAM("new l_m: " << l_m);
			}
			if (lr_y == corners[right_corner].y) {
				r_m = ((float) corners[top_corner].x
						- (float) corners[right_corner].x)
						/ ((float) corners[top_corner].y
								- (float) corners[right_corner].y);
				r_x = (float) corners[right_corner].x;
//				ROS_INFO_STREAM("new r_m: " <<r_m);
			}
			l_x += l_m;
			r_x += r_m;
			lr_y++;
//			ROS_INFO_STREAM(
//					"line x: " << lr_y << " l_x: " << l_x << " r_x: " << r_x);
//			ROS_INFO_STREAM(
//					"line(rounded) x: " << lr_y << " l_x: " << (unsigned int) round(l_x) << " r_x: " << (unsigned int) round(r_y));
			if (isLineInCollision((unsigned int) round(l_x),
					(unsigned int) round(r_x), lr_y, map, vis_map))
				return true; //collision = true; //return true;
		}

	}
	return false; //return collision;
}

bool CollisionChecker::isLineInCollision(int y_start, int y_end, int x,
		nav_msgs::OccupancyGrid &map, std::vector<int8_t> &vis_map) {
//	ROS_INFO_STREAM(
//			"is line in collision? x: " << x <<" y-start: " << y_start << " y-end: " << y_end);
//	ROS_INFO_STREAM(
//			"map width: " << map.info.width << " map height: " << map.info.width << " map data length: " << map.data.size() << "map at 0 " << (int)map.data[0]);
	if (y_start < 0 || y_end > map.info.width || x < 0 || x > map.info.height)
		return true;
	bool collision = false;
	for (int y = x * map.info.width + y_start; y <= x * map.info.width + y_end;
			y++) {
		if (map.data[y] == -1 || map.data[y] >= 100) {
			return true; //collision = true; //return true;
		} else {
			vis_map[y] = 0;
		}
	}
	return false; //collision; //false;
}

void CollisionChecker::initVisMap(const nav_msgs::OccupancyGrid &map) {
	vis_map.header.frame_id = "map";
	vis_map.info.resolution = map.info.resolution;
	vis_map.info.width = map.info.width;
	vis_map.info.height = map.info.height;
	vis_map.info.origin = map.info.origin;
	vis_map.data = std::vector<int8_t>(map.info.width * map.info.height, -1);
	_init_vis_map = true;
}

bool CollisionChecker::initialize(geometry_msgs::Point position) {
	nav_msgs::OccupancyGrid map = _occupancy_grid;
	if (_visualize_collision) {
		vis_map.header.stamp = ros::Time::now();
		vis_map.info.map_load_time = ros::Time::now();
		initVisMap(map);
	}
	std::vector<int8_t> tmp_vis_map_data = vis_map.data;
	if (!isCircleInCollision(position.x, position.y, map, tmp_vis_map_data)) {
		if (_visualize_collision) {
			vis_map.data = tmp_vis_map_data;
			_visualization_pub.publish(vis_map);
		}
		return true;
	}
	return false;
}

bool CollisionChecker::steer(rrt_nbv_exploration_msgs::Node &new_node,
		rrt_nbv_exploration_msgs::Node &nearest_node,
		geometry_msgs::Point rand_sample, double min_distance) {
	nav_msgs::OccupancyGrid map = _occupancy_grid;
	double distance = sqrt(min_distance);
	//if (distance >= 2 * _robot_radius) {
	bool no_collision = false;
	if (_visualize_collision) {
		vis_map.header.stamp = ros::Time::now();
		vis_map.info.map_load_time = ros::Time::now();
	}
	std::vector<int8_t> tmp_vis_map_data = vis_map.data;
//	ROS_INFO_STREAM(
//			"Dist: "<< distance << " thres:" << _path_box_distance_thres << " rand x: " << rand_sample.x << " rand y: " << rand_sample.y);
//	ROS_INFO_STREAM("yaw: " << atan2(rand_sample.y - nearest_node.position.y,
//									rand_sample.x - nearest_node.position.x) << " rand: x: " << rand_sample.x << " ,y: " << rand_sample.y
//			<< " nearest: x: " << nearest_node.position.x << " ,y: " << nearest_node.position.y);
	bool rectangle = (
			distance > _path_box_distance_thres ?
					!isRectangleInCollision(
							(nearest_node.position.x + rand_sample.x) / 2,
							(nearest_node.position.y + rand_sample.y) / 2,
							atan2(rand_sample.y - nearest_node.position.y,
									rand_sample.x - nearest_node.position.x),
							(distance - _path_box_distance_thres) / 2,
							_robot_width / 2, map, tmp_vis_map_data) :
					true);
	bool circle = !isCircleInCollision(rand_sample.x, rand_sample.y, map,
			tmp_vis_map_data);
	if (rectangle && circle) {
		new_node.position.x = rand_sample.x;
		new_node.position.y = rand_sample.y;
		new_node.position.z = rand_sample.z;
		new_node.children_counter = 0;
		new_node.status = rrt_nbv_exploration_msgs::Node::INITIAL;
		if (_visualize_collision) {
			vis_map.data = tmp_vis_map_data;
			_visualization_pub.publish(vis_map);
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
