#include <rrt_nbv_exploration/costmap_client.h>

#include <functional>
#include <mutex>
#include <string>

namespace rrt_nbv_exploration {
// static translation table to speed things up
std::array<unsigned char, 256> init_translation_table();
static const std::array<unsigned char, 256> cost_translation_table__ =
		init_translation_table();

Costmap2DClient::Costmap2DClient(ros::NodeHandle &param_nh,
		ros::NodeHandle &subscription_nh) {
	std::string costmap_topic;
	std::string footprint_topic;
	std::string costmap_updates_topic;
	param_nh.param("occupancy_grid_topic", costmap_topic, std::string("map"));
	costmap_updates_topic = costmap_topic + "_updates";
	/* initialize costmap */
	costmap_sub_ = subscription_nh.subscribe<nav_msgs::OccupancyGrid>(
			costmap_topic, 1000,
			[this](const nav_msgs::OccupancyGrid::ConstPtr &msg) {
				updateFullMap(msg);
			});
	ROS_INFO("Waiting for costmap to become available, topic: %s",
			costmap_topic.c_str());
	auto costmap_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(
			costmap_topic, subscription_nh);
	updateFullMap(costmap_msg);

	/* subscribe to map updates */
	costmap_updates_sub_ = subscription_nh.subscribe<
			map_msgs::OccupancyGridUpdate>(costmap_updates_topic, 1000,
			[this](const map_msgs::OccupancyGridUpdate::ConstPtr &msg) {
				updatePartialMap(msg);
			});
}

void Costmap2DClient::updateFullMap(
		const nav_msgs::OccupancyGrid::ConstPtr &msg) {
	unsigned int size_in_cells_x = msg->info.width;
	unsigned int size_in_cells_y = msg->info.height;
	double resolution = msg->info.resolution;
	double origin_x = msg->info.origin.position.x;
	double origin_y = msg->info.origin.position.y;

	ROS_DEBUG("received full new map, resizing to: %d, %d", size_in_cells_x,
			size_in_cells_y);
	costmap_.resizeMap(size_in_cells_x, size_in_cells_y, resolution, origin_x,
			origin_y);

	// lock as we are accessing raw underlying map
	auto *mutex = costmap_.getMutex();
	std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*mutex);

	// fill map with data
	unsigned char *costmap_data = costmap_.getCharMap();
	size_t costmap_size = costmap_.getSizeInCellsX()
			* costmap_.getSizeInCellsY();
	ROS_DEBUG("full map update, %lu values", costmap_size);
	for (size_t i = 0; i < costmap_size && i < msg->data.size(); ++i) {
		unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
		costmap_data[i] = cost_translation_table__[cell_cost];
	}
	ROS_DEBUG("map updated, written %lu values", costmap_size);
}

void Costmap2DClient::updatePartialMap(
		const map_msgs::OccupancyGridUpdate::ConstPtr &msg) {
	ROS_DEBUG("received partial map update");

	if (msg->x < 0 || msg->y < 0) {
		ROS_ERROR("negative coordinates, invalid update. x: %d, y: %d", msg->x,
				msg->y);
		return;
	}

	size_t x0 = static_cast<size_t>(msg->x);
	size_t y0 = static_cast<size_t>(msg->y);
	size_t xn = msg->width + x0;
	size_t yn = msg->height + y0;

	// lock as we are accessing raw underlying map
	auto *mutex = costmap_.getMutex();
	std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*mutex);

	size_t costmap_xn = costmap_.getSizeInCellsX();
	size_t costmap_yn = costmap_.getSizeInCellsY();

	if (xn > costmap_xn || x0 > costmap_xn || yn > costmap_yn
			|| y0 > costmap_yn) {
		ROS_WARN("received update doesn't fully fit into existing map, "
				"only part will be copied. received: [%lu, %lu], [%lu, %lu] "
				"map is: [0, %lu], [0, %lu]", x0, xn, y0, yn, costmap_xn,
				costmap_yn);
	}

	// update map with data
	unsigned char *costmap_data = costmap_.getCharMap();
	size_t i = 0;
	for (size_t y = y0; y < yn && y < costmap_yn; ++y) {
		for (size_t x = x0; x < xn && x < costmap_xn; ++x) {
			size_t idx = costmap_.getIndex(x, y);
			unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
			costmap_data[idx] = cost_translation_table__[cell_cost];
			++i;
		}
	}
}

std::array<unsigned char, 256> init_translation_table() {
	std::array<unsigned char, 256> cost_translation_table;

	// lineary mapped from [0..100] to [0..255]
	for (size_t i = 0; i < 256; ++i) {
		cost_translation_table[i] = static_cast<unsigned char>(1
				+ (251 * (i - 1)) / 97);
	}

	// special values:
	cost_translation_table[0] = 0;      // NO obstacle
	cost_translation_table[99] = 253;   // INSCRIBED obstacle
	cost_translation_table[100] = 254;  // LETHAL obstacle
	cost_translation_table[static_cast<unsigned char>(-1)] = 255;  // UNKNOWN

	return cost_translation_table;
}

}  // namespace explore
