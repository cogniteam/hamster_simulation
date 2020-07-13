/**
 * Filename: Wandering.h
 *   Author: Igor Makhtes
 *     Date: Nov 25, 2014
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef INCLUDE_WANDERER_WANDERING_H_
#define INCLUDE_WANDERER_WANDERING_H_


#include <time.h>

#include <boost/thread.hpp>
#include <boost/foreach.hpp>

#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>

#include <hupster_wandering/trajectory/simulator/TrajectorySimulator.h>
#include <hupster_wandering/trajectory/matcher/SimpleTrajectoryMatcher.h>
#include <hupster_wandering/trajectory/simulator/models/AckermannModel.h>


using namespace std;


#define foreach BOOST_FOREACH


/*
 * Simple wandering algorithm
 */
class Wandering {

public:

	Wandering(const string& baseFrameId, double maxVelocity, bool enabled);

	virtual ~Wandering();

public:

	void spin();

private:

	TrajectoryMatch::Ptr chooseBestTrajectory(
            const nav_msgs::OccupancyGrid& costMap);

	void createTrajectories(
			double simulationTime, double granularity);

	void stateCallback(const std_msgs::Bool::Ptr& state);

    void costmapCallback(const nav_msgs::OccupancyGrid::Ptr& costmap) {
        costmap_ = costmap;
    }

    void costmapUpdateCallback(const map_msgs::OccupancyGridUpdate::Ptr& map) {

        if (!costmap_) {
            return;
        }

        size_t x0 = static_cast<size_t>(map->x);
        size_t y0 = static_cast<size_t>(map->y);
        size_t xn = map->width + x0;
        size_t yn = map->height + y0;

        // lock as we are accessing raw underlying map

        size_t costmap_xn = costmap_->info.width;
        size_t costmap_yn = costmap_->info.height;

        // update map with data
        vector<int8_t>& costmap_data = costmap_->data;

        size_t i = 0;

        for (size_t y = y0; y < yn && y < costmap_yn; ++y)
        {
            for (size_t x = x0; x < xn && x < costmap_xn; ++x)
            {
                size_t idx =  y * costmap_->info.width + x;
                unsigned char cell_cost = static_cast<unsigned char>(map->data[i]);
                costmap_data[idx] = cell_cost;
                ++i;
            }
        }
    }

    void publishState() {
        std_msgs::Bool state;
        state.data = enabled_;
        statePublisher_.publish(state);
    }

private:

	double maxVelocity_;

	bool enabled_;

	bool publishStop_;

	string baseFrameId_;

	ros::Time preferSideChangeTime_;

    ros::Publisher statePublisher_;

	bool preferRight_;

	SimpleTrajectoryMatcher* trajectoryMatcher_;

	Trajectory::VectorPtr frontTrajectories_;

	Trajectory::Ptr frontTrajectory_;

    nav_msgs::OccupancyGrid::Ptr costmap_;

};

#endif /* INCLUDE_WANDERER_WANDERING_H_ */
