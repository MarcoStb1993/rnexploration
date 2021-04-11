/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#ifndef COSTMAP_CLIENT_
#define COSTMAP_CLIENT_

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Pose.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

namespace rrt_nbv_exploration {
class Costmap2DClient {
public:
	/**
	 * @brief Contructs client and start listening
	 * @details Constructor will block until first map update is received and
	 * map is ready to use
	 *
	 * @param param_nh node hadle to retrieve parameters from
	 * @param subscription_nh node hadle where topics will be subscribed
	 */
	Costmap2DClient(ros::NodeHandle &param_nh,
			ros::NodeHandle &subscription_nh);

	/**
	 * @brief Return a pointer to the "master" costmap which receives updates from
	 * all the layers.
	 *
	 * This pointer will stay the same for the lifetime of Costmap2DClient object.
	 */
	costmap_2d::Costmap2D* getCostmap() {
		return &costmap_;
	}

	/**
	 * @brief Return a pointer to the "master" costmap which receives updates from
	 * all the layers.
	 *
	 * This pointer will stay the same for the lifetime of Costmap2DClient object.
	 */
	const costmap_2d::Costmap2D* getCostmap() const {
		return &costmap_;
	}

protected:
	void updateFullMap(const nav_msgs::OccupancyGrid::ConstPtr &msg);
	void updatePartialMap(const map_msgs::OccupancyGridUpdate::ConstPtr &msg);

	costmap_2d::Costmap2D costmap_;

private:
	// will be unsubscribed at destruction
	ros::Subscriber costmap_sub_;
	ros::Subscriber costmap_updates_sub_;
};

}  // namespace explore

#endif
