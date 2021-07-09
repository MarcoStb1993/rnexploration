#include <rrg_nbv_exploration/GainCalculator.h>

namespace rrg_nbv_exploration {

GainCalculator::GainCalculator() :
		_gain_poll_points(boost::extents[0][0][0]) {
	ros::NodeHandle private_nh("~");
	private_nh.param("sensor_max_range", _sensor_max_range, 5.0);
	private_nh.param("sensor_min_range", _sensor_min_range, 1.0);
	private_nh.param("delta_phi", _delta_phi, 10);
	private_nh.param("delta_theta", _delta_theta, 10);
	private_nh.param("delta_radius", _delta_radius, 0.1);
	private_nh.param("sensor_horizontal_fov", _sensor_horizontal_fov, 360);
	private_nh.param("sensor_vertical_fov_bottom", _sensor_vertical_fov_bottom,
			180);
	private_nh.param("sensor_vertical_fov_top", _sensor_vertical_fov_top, 0);
	private_nh.param("sensor_height", _sensor_height, 0.5);
	private_nh.param("sensor_size", _sensor_size, 0.1);
	private_nh.param("min_view_score", _min_view_score, 0.1);
	private_nh.param("oc_resolution", _octomap_resolution, 0.1);
	private_nh.param("max_node_height_difference", _max_node_height_difference,
			1.0);
	private_nh.param("dbscan_min_points", _dbscan_min_points, 3);
	private_nh.param("dbscan_epsilon", _dbscan_epsilon, 1.0);
	std::string octomap_topic;
	private_nh.param<std::string>("octomap_topic", octomap_topic,
			"octomap_binary");
	ros::NodeHandle nh("rne");
	raysample_visualization = nh.advertise<visualization_msgs::Marker>(
			"raysample_visualization", 1000);
	cluster_visualization = nh.advertise<visualization_msgs::Marker>(
			"cluster_visualization", 1000);
	_updated_node_publisher = nh.advertise<rrg_nbv_exploration_msgs::Node>(
			"updated_node", 1);
	_node_to_update_subscriber = nh.subscribe("node_to_update", 1,
			&GainCalculator::nodeToUpdateCallback, this);
	_octomap_sub = _nh.subscribe(octomap_topic, 1,
			&GainCalculator::convertOctomapMsgToOctree, this);

	_best_gain_per_view = 0;
	_max_gain_points = 0;
	_last_updated_node.index = -1;
	_sensor_min_range_squared = pow(_sensor_min_range, 2);
}

GainCalculator::~GainCalculator() {
	_gain_poll_points.resize(boost::extents[0][0][0]);
}

void GainCalculator::precalculateGainPolls() {
	precalculateGainPollPoints();
}

void GainCalculator::precalculateGainPollPoints() {
	std::vector<StepStruct> theta_steps;
	std::vector<StepStruct> phi_steps;
	for (int theta = 0; theta < 360; theta += _delta_theta) {
		double rad = theta * M_PI / 180;
		StepStruct step = { theta, cos(rad), sin(rad) };
		theta_steps.push_back(step);
	}
	for (int phi = _sensor_vertical_fov_top; phi <= _sensor_vertical_fov_bottom;
			phi += _delta_phi) {
		double rad = phi * M_PI / 180;
		StepStruct step = { phi, cos(rad), sin(rad) };
		phi_steps.push_back(step);
	}
	int radius_steps =
			(int) ((_sensor_max_range - _sensor_size) / _delta_radius);
	_gain_poll_points.resize(
			boost::extents[theta_steps.size()][phi_steps.size()][radius_steps]);
	for (int t = 0; t < theta_steps.size(); t++) {
		StepStruct theta = theta_steps.at(t);
		for (int p = 0; p < phi_steps.size(); p++) {
			StepStruct phi = phi_steps.at(p);
			for (int r = 0; r < radius_steps; r++) {
				double radius = _sensor_size + r * _delta_radius;
				_gain_poll_points[t][p][r] =
						{ radius * theta.cos * phi.sin, radius * theta.sin
								* phi.sin, radius * phi.cos, theta.step,
								phi.step, radius, radius > _sensor_min_range };
			}
		}
	}

	int range_steps = (int) ((_sensor_max_range - _sensor_min_range)
			/ _delta_radius);
	_best_gain_per_view = phi_steps.size() * range_steps
			* (_sensor_horizontal_fov / _delta_theta + 1);
	_max_gain_points = theta_steps.size() * phi_steps.size() * range_steps;
//	ROS_WARN_STREAM(
//			"Ray sampling gain calculation initialized with " << _max_gain_points << " poll points and max reachable gain per view of " << _best_gain_per_view);
}

void GainCalculator::calculateGain(rrg_nbv_exploration_msgs::Node &node) {
	if (!measureNodeHeight(node) && node.distanceToRobot != 0
			&& node.edges.size() < 2) {
		//node.status = rrg_nbv_exploration_msgs::Node::INITIAL;
		node.gain = -1;
		return;
	}
	calculatePointGain(node);
}

void GainCalculator::calculatePointGain(rrg_nbv_exploration_msgs::Node &node) {
	bool publish_visualization = raysample_visualization.getNumSubscribers()
			> 0;
	bool publish_cluster_visualization =
			cluster_visualization.getNumSubscribers() > 0;
	visualization_msgs::Marker node_points_vis;
	visualization_msgs::Marker cluster_points_vis;
	if (publish_visualization) {
		node_points_vis.header.frame_id = "/map";
		node_points_vis.ns = "raysample_visualization";
		node_points_vis.id = 0;
		node_points_vis.action = visualization_msgs::Marker::ADD;
		node_points_vis.pose.orientation.w = 1.0;
		node_points_vis.type = visualization_msgs::Marker::SPHERE_LIST;
		node_points_vis.scale.x = _octomap_resolution * 0.9;
		node_points_vis.scale.y = _octomap_resolution * 0.9;
		node_points_vis.scale.z = _octomap_resolution * 0.9;
		node_points_vis.color.a = 1.0f;
		node_points_vis.header.stamp = ros::Time::now();
	}
	if (publish_cluster_visualization) {
		cluster_points_vis.header.frame_id = "/map";
		cluster_points_vis.ns = "cluster_visualization";
		cluster_points_vis.id = 0;
		cluster_points_vis.action = visualization_msgs::Marker::ADD;
		cluster_points_vis.pose.orientation.w = 1.0;
		cluster_points_vis.type = visualization_msgs::Marker::CUBE_LIST;
		cluster_points_vis.scale.x = _octomap_resolution * 0.3;
		cluster_points_vis.scale.y = _octomap_resolution * 0.3;
		cluster_points_vis.scale.z = _octomap_resolution;
		cluster_points_vis.color.a = 1.0f;
		cluster_points_vis.header.stamp = ros::Time::now();
	}
	node.gain = 0.0;

	cluster_point_array cluster_points(
			boost::extents[_gain_poll_points.shape()[0]][_gain_poll_points.shape()[1]][_gain_poll_points.shape()[2]]);
	std::fill(cluster_points.origin(),
			cluster_points.origin() + cluster_points.num_elements(),
			ClusterLabel::NoPoint);

	std::map<int, int> gain_per_yaw;

	double x = node.position.x;
	double y = node.position.y;
	double z = node.position.z;

	for (multi_array_index theta = 0; theta < _gain_poll_points.shape()[0];
			theta++) {
		for (multi_array_index phi = 0; phi < _gain_poll_points.shape()[1];
				phi++) {
			for (multi_array_index radius = 0;
					radius < _gain_poll_points.shape()[2]; radius++) {
				PollPoint point = _gain_poll_points[theta][phi][radius];
				geometry_msgs::Point vis_point;
				vis_point.x = point.x + x;
				vis_point.y = point.y + y;
				vis_point.z = point.z + z;
				if (publish_visualization) {
					octomap::OcTreeNode *ocnode = _octree->search(vis_point.x,
							vis_point.y, vis_point.z);
					std_msgs::ColorRGBA color;
					color.r = 0.0f;
					color.g = 0.0f;
					color.b = 1.0f;
					color.a = 1.0f;
					node_points_vis.points.push_back(vis_point);
					if (ocnode != NULL) {
						if (_octree->isNodeOccupied(ocnode)) {
							color.r = 1.0f;
							color.b = 0.0f;
							node_points_vis.colors.push_back(color);
							break; //end ray sampling for this ray because of an obstacle in the way
						} else {
							color.g = 1.0f;
							color.b = 0.0f;
						}
					} else {
						if (point.in_range) {
							cluster_points[theta][phi][radius] =
									ClusterLabel::Undefined;
							gain_per_yaw[point.theta]++;
						} else {
							color.r = 0.7f;
							color.g = 0.7f;
							color.b = 0.7f;
						}

					}
					node_points_vis.colors.push_back(color);
				} else {
					octomap::OcTreeNode *ocnode = _octree->search(vis_point.x,
							vis_point.y, vis_point.z);
					if (ocnode != NULL) {
						if (_octree->isNodeOccupied(ocnode))
							break; //end ray sampling for this ray because of an obstacle in the way
					} else if (point.in_range) {
						cluster_points[theta][phi][radius] =
								ClusterLabel::Undefined;
						gain_per_yaw[point.theta]++;
					}
				}
			}
		}
	}

	//DBSCAN
	std::vector<PointCluster> cluster;
	int cluster_counter = 0;
	for (cluster_point_array_index theta = 0; theta < cluster_points.shape()[0];
			theta++) {
		for (cluster_point_array_index phi = 0; phi < cluster_points.shape()[1];
				phi++) {
			for (cluster_point_array_index radius = 0;
					radius < cluster_points.shape()[2]; radius++) {
				if (cluster_points[theta][phi][radius]
						!= ClusterLabel::Undefined) {
					continue;
				}
				ClusterIndex index = { theta, phi, radius };
				std::queue<ClusterIndex> neighbor_keys;
				retrieveClusterPointNeighbors(index, cluster_points,
						neighbor_keys);
				if (neighbor_keys.size() < _dbscan_min_points) {
					cluster_points[theta][phi][radius] = ClusterLabel::Noise;
					continue;
				}
				cluster_points[theta][phi][radius] = ++cluster_counter;
				PointCluster current_cluster = { 1, cluster_counter,
						(double) theta * _delta_theta, (double) phi
								* _delta_phi, (double) radius * _delta_radius };
				if (publish_cluster_visualization) {
					addClusterVisualizationPoint(theta, phi, radius, x, y, z,
							cluster_counter, cluster_points_vis);
				}
				// Update cluster meta data
				current_cluster.size++;
				current_cluster.center_theta += (double) theta * _delta_theta;
				current_cluster.center_phi += (double) phi * _delta_phi;
				current_cluster.center_radius += (double) radius
						* _delta_radius;
				while (!neighbor_keys.empty()) {
					ClusterIndex neighbor = neighbor_keys.front();
					if (cluster_points[neighbor.theta][neighbor.phi][neighbor.radius]
							<= ClusterLabel::Undefined) { // not part of any cluster
						cluster_points[neighbor.theta][neighbor.phi][neighbor.radius] =
								cluster_counter;
						if (publish_cluster_visualization) {
							addClusterVisualizationPoint(neighbor.theta,
									neighbor.phi, neighbor.radius, x, y, z,
									cluster_counter, cluster_points_vis);
						}
						// Update cluster meta data
						current_cluster.size++;
						current_cluster.center_theta += (double) neighbor.theta
								* _delta_theta;
						current_cluster.center_phi += (double) neighbor.phi
								* _delta_phi;
						current_cluster.center_radius +=
								(double) neighbor.radius * _delta_radius;
						// add neighbors to queue if more than min points
						retrieveClusterPointNeighbors(neighbor, cluster_points,
								neighbor_keys);
					}
					neighbor_keys.pop();
				}
				current_cluster.center_theta /= (double) current_cluster.size;
				current_cluster.center_phi /= (double) current_cluster.size;
				current_cluster.center_radius /= (double) current_cluster.size;
				cluster.push_back(current_cluster);
			}
		}
	}

	for (auto i : cluster) {
		ROS_INFO_STREAM(
				"Cluster " << i.number << " size: " << i.size << " Center: " << i.center_theta << ", " << i.center_phi << ", " << i.center_radius);
	}
	//TODO: visualize clusters

	int best_yaw = 0;
	int best_yaw_score = 0;
	int horizontal_fov =
			_sensor_horizontal_fov == 360 ? 270 : _sensor_horizontal_fov; //get a best yaw for 360deg sensors (always 0 otherwise)

	for (int yaw = 0; yaw < 360; yaw++) {
		double yaw_score = 0;
		for (int fov = -horizontal_fov / 2; fov < horizontal_fov / 2; fov++) {
			int theta = yaw + fov;
			if (theta < 0)
				theta += 360;
			if (theta >= 360)
				theta -= 360;
			yaw_score += gain_per_yaw[theta];
		}
		if (best_yaw_score < yaw_score) {
			best_yaw_score = yaw_score;
			best_yaw = yaw;
		}
	}

	double view_score = (double) best_yaw_score / (double) _best_gain_per_view;

	ROS_INFO_STREAM(
			"Best yaw score: " << best_yaw_score << " view score: " << view_score << " best yaw: " << best_yaw);

	if (view_score < _min_view_score
			|| (node.status == rrg_nbv_exploration_msgs::Node::VISITED
					&& node.best_yaw <= best_yaw + 5
					&& node.best_yaw >= best_yaw - 5)) {
		//no use exploring similar yaw again, sensor position approximation flawed in this case
		node.status = rrg_nbv_exploration_msgs::Node::EXPLORED;
		node.gain = 0;
	} else {
		node.gain = best_yaw_score;
		node.best_yaw = best_yaw;
	}

	if (publish_visualization) {
		//Visualize best yaw direction
		geometry_msgs::Point vis_point;
		vis_point.x = x
				+ (_sensor_max_range + _delta_radius)
						* cos(M_PI * best_yaw / 180.0);
		vis_point.y = y
				+ (_sensor_max_range + _delta_radius)
						* sin(M_PI * best_yaw / 180.0);
		vis_point.z = z;
		std_msgs::ColorRGBA color;
		color.r = 1.0f;
		color.g = 1.0f;
		color.b = 0.0f;
		color.a = 1.0f;
		node_points_vis.points.push_back(vis_point);
		node_points_vis.colors.push_back(color);
		raysample_visualization.publish(node_points_vis);
	}
	if (publish_cluster_visualization) {
		//Visualize point clusters
		visualization_msgs::Marker cluster_center_vis;
		cluster_center_vis.header.frame_id = "/map";
		cluster_center_vis.ns = "cluster_visualization";
		cluster_center_vis.id = 1;
		cluster_center_vis.action = visualization_msgs::Marker::ADD;
		cluster_center_vis.pose.orientation.w = 1.0;
		cluster_center_vis.type = visualization_msgs::Marker::CUBE_LIST;
		cluster_center_vis.scale.x = _octomap_resolution * 2;
		cluster_center_vis.scale.y = _octomap_resolution * 2;
		cluster_center_vis.scale.z = _octomap_resolution * 2;
		cluster_center_vis.color.a = 1.0f;
		cluster_center_vis.header.stamp = ros::Time::now();
		for (auto i : cluster) {
			geometry_msgs::Point vis_point;
			vis_point.x = x
					+ i.center_radius * _delta_radius
							* cos(M_PI * i.center_theta / 180.0)
							* sin(
									M_PI
											* (_sensor_vertical_fov_top
													+ i.center_phi) / 180.0);
			vis_point.y = y
					+ i.center_radius * sin(M_PI * i.center_theta / 180.0)
							* sin(
									M_PI
											* (_sensor_vertical_fov_top
													+ i.center_phi) / 180.0);
			vis_point.z = z + i.center_radius * cos(
			M_PI * (_sensor_vertical_fov_top + i.center_phi) / 180.0);
			std_msgs::ColorRGBA color;
			color.r = 0.0f;
			color.g = 0.0f;
			color.b = 0.0f;
			color.a = 0.5f;
			if (i.number <= 2) {
				color.r = i.number * 0.5;
			} else if (i.number > 2 && i.number <= 4) {
				color.g = (i.number - 2) * 0.5;
			} else if (i.number > 4 && i.number <= 6) {
				color.b = (i.number - 4) * 0.5;
			} else if (i.number > 6 && i.number <= 8) {
				color.r = (i.number - 6) * 0.5;
				color.g = (i.number - 6) * 0.5;
			} else if (i.number > 8 && i.number <= 10) {
				color.r = (i.number - 8) * 0.5;
				color.b = (i.number - 8) * 0.5;
			} else {
				color.g = 0.5f;
				color.b = 0.5f;
			}
			cluster_center_vis.points.push_back(vis_point);
			cluster_center_vis.colors.push_back(color);
		}
		cluster_visualization.publish(cluster_center_vis);
		cluster_visualization.publish(cluster_points_vis);
	}

}

void GainCalculator::addClusterVisualizationPoint(
		cluster_point_array_index theta, cluster_point_array_index phi,
		cluster_point_array_index radius, double x, double y, double z,
		int cluster_counter, visualization_msgs::Marker &cluster_points_vis) {
	PollPoint point = _gain_poll_points[theta][phi][radius];
	geometry_msgs::Point vis_point;
	vis_point.x = point.x + x;
	vis_point.y = point.y + y;
	vis_point.z = point.z + z;
	std_msgs::ColorRGBA color;
	color.r = 0.0f;
	color.g = 0.0f;
	color.b = 0.0f;
	color.a = 1.0f;
	if (cluster_counter <= 2) {
		color.r = cluster_counter * 0.5;
	} else if (cluster_counter > 2 && cluster_counter <= 4) {
		color.g = (cluster_counter - 2) * 0.5;
	} else if (cluster_counter > 4 && cluster_counter <= 6) {
		color.b = (cluster_counter - 4) * 0.5;
	} else if (cluster_counter > 6 && cluster_counter <= 8) {
		color.r = (cluster_counter - 6) * 0.5;
		color.g = (cluster_counter - 6) * 0.5;
	} else if (cluster_counter > 8 && cluster_counter <= 10) {
		color.r = (cluster_counter - 8) * 0.5;
		color.b = (cluster_counter - 8) * 0.5;
	} else {
		color.g = 0.5f;
		color.b = 0.5f;
	}

	cluster_points_vis.points.push_back(vis_point);
	cluster_points_vis.colors.push_back(color);
}

void GainCalculator::retrieveClusterPointNeighbors(ClusterIndex point,
		cluster_point_array &cluster_points,
		std::queue<ClusterIndex> &neighbor_keys) {
	std::vector<ClusterIndex> neighbors;
	if (point.theta == 0) {
		checkIfClusterPointExists(
				{ (cluster_point_array_index) cluster_points.shape()[0] - 1,
						point.phi, point.radius }, cluster_points, neighbors);
		checkIfClusterPointExists( { point.theta + 1, point.phi, point.radius },
				cluster_points, neighbors);
	} else if (point.theta == cluster_points.shape()[0] - 1) {
		checkIfClusterPointExists( { point.theta - 1, point.phi, point.radius },
				cluster_points, neighbors);
		checkIfClusterPointExists( { 0, point.phi, point.radius },
				cluster_points, neighbors);
	} else {
		checkIfClusterPointExists( { point.theta - 1, point.phi, point.radius },
				cluster_points, neighbors);
		checkIfClusterPointExists( { point.theta + 1, point.phi, point.radius },
				cluster_points, neighbors);
	}
	if (point.phi == 0) {
		checkIfClusterPointExists( { point.theta, point.phi + 1, point.radius },
				cluster_points, neighbors);
	} else if (point.phi == cluster_points.shape()[1] - 1) {
		checkIfClusterPointExists( { point.theta, point.phi - 1, point.radius },
				cluster_points, neighbors);
	} else {
		checkIfClusterPointExists( { point.theta, point.phi - 1, point.radius },
				cluster_points, neighbors);
		checkIfClusterPointExists( { point.theta, point.phi + 1, point.radius },
				cluster_points, neighbors);
	}
	if (point.radius == 0) {
		checkIfClusterPointExists( { point.theta, point.phi, point.radius + 1 },
				cluster_points, neighbors);
	} else if (point.radius == cluster_points.shape()[2] - 1) {
		checkIfClusterPointExists( { point.theta, point.phi, point.radius - 1 },
				cluster_points, neighbors);
	} else {
		checkIfClusterPointExists( { point.theta, point.phi, point.radius - 1 },
				cluster_points, neighbors);
		checkIfClusterPointExists( { point.theta, point.phi, point.radius + 1 },
				cluster_points, neighbors);
	}
	if (neighbors.size() >= _dbscan_min_points) {
		for (auto neighbor : neighbors) {
			neighbor_keys.push(neighbor);
		}
	}
}

void GainCalculator::checkIfClusterPointExists(ClusterIndex index,
		cluster_point_array &cluster_points,
		std::vector<ClusterIndex> &neighbors) {
	if (cluster_points[index.theta][index.phi][index.radius]
			!= ClusterLabel::NoPoint) {
		neighbors.push_back(index);
	}
}

bool GainCalculator::measureNodeHeight(rrg_nbv_exploration_msgs::Node &node) {
	double min_x, min_y, min_z;
	_octree->getMetricMin(min_x, min_y, min_z);
	octomap::point3d node_point(node.position.x, node.position.y,
			node.position.z);
	octomap::point3d bottom_point(node.position.x, node.position.y,
			min_z - _octomap_resolution); //seems to ignore lowest voxel if using only min_z
	octomap::KeyRay keyray;
// Raytrace from initial node height (parent height) to min z value to find first occupied voxel (assume current z pos is above ground)
	if (_octree->computeRayKeys(node_point, bottom_point, keyray)) {
		for (auto iterator : keyray) {
			octomap::point3d coords = _octree->keyToCoord(iterator);
			octomap::OcTreeNode *ocnode = _octree->search(iterator);
			if (ocnode != NULL) {
				if (_octree->isNodeOccupied(ocnode)) {
					double new_height = coords.z() + _sensor_height;
					if (abs(node.position.z - new_height)
							<= _max_node_height_difference) {
						node.position.z = new_height;
						return true;
					}
				}
			}
		}
	} else {
		ROS_INFO_STREAM(
				"Raytracing for node height measurement to min z out of bounds");
	}
	double max_x, max_y, max_z;
	_octree->getMetricMax(max_x, max_y, max_z);
	octomap::point3d top_point(node.position.x, node.position.y,
			max_z + _octomap_resolution); //seems to ignore lowest voxel if using only min_z, suspect the same for max z
// Raytrace from initial node height (parent height) to max z value to find first free voxel after occupied voxel (assume current z pos is below ground)
	bool ground_detected = false;
	if (_octree->computeRayKeys(node_point, top_point, keyray)) {
		for (auto iterator : keyray) {
			octomap::point3d coords = _octree->keyToCoord(iterator);
			octomap::OcTreeNode *ocnode = _octree->search(iterator);
			if (ocnode != NULL) {
				if (!ground_detected) {	//find first occupied voxel
					if (_octree->isNodeOccupied(ocnode)) {
						ground_detected = true;
					}
				} else {//find first empty voxel above ground, if there are none, it was probably the ceiling
					if (!_octree->isNodeOccupied(ocnode)) {
						double new_height = coords.z() + _sensor_height;
						if (abs(node.position.z - new_height)
								<= _max_node_height_difference) {
							node.position.z = new_height;
							return true;
						}
					}
				}
			}
		}
	} else {
		ROS_INFO_STREAM(
				"Raytracing for node height measurement to max z out of bounds");
	}
	return false;
}

void GainCalculator::convertOctomapMsgToOctree(
		const octomap_msgs::Octomap::ConstPtr &map_msg) {
	_abstract_octree.reset(octomap_msgs::msgToMap(*map_msg));
	_octree = std::dynamic_pointer_cast<octomap::OcTree>(_abstract_octree);
}

void GainCalculator::nodeToUpdateCallback(
		const rrg_nbv_exploration_msgs::NodeToUpdate::ConstPtr &node_to_update) {
	if (_last_updated_node.index != node_to_update->node.index
			|| node_to_update->force_update) {
		rrg_nbv_exploration_msgs::Node node;
		geometry_msgs::Point pos;
		pos.x = node_to_update->node.position.x;
		pos.y = node_to_update->node.position.y;
		pos.z = node_to_update->node.position.z;
		node.position = pos;
		node.status = node_to_update->node.status;
		node.index = node_to_update->node.index;
		node.gain = node_to_update->node.gain;
		node.best_yaw = node_to_update->node.best_yaw;
		calculateGain(node);
		_last_updated_node = node;
		_updated_node_publisher.publish(node);
	} else {
		_updated_node_publisher.publish(_last_updated_node);
	}
}

void GainCalculator::dynamicReconfigureCallback(
		rrg_nbv_exploration::GainCalculatorConfig &config, uint32_t level) {
	_min_view_score = config.min_view_score;
}

}
