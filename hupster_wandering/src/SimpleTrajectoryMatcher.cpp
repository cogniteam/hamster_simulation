/**
 * Filename: SimpleTrajectoryMatcher.cpp
 *   Author: Igor Makhtes
 *     Date: Nov 29, 2014
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

#include <hupster_wandering/trajectory/matcher/SimpleTrajectoryMatcher.h>


SimpleTrajectoryMatcher::SimpleTrajectoryMatcher() {

}

SimpleTrajectoryMatcher::~SimpleTrajectoryMatcher() {

}

int SimpleTrajectoryMatcher::getCellValue(
        const nav_msgs::OccupancyGrid& costmap, const geometry_msgs::PoseStamped& pose) const {
    
    tf::StampedTransform t;
    auto poseCopy = pose;
    poseCopy.header.stamp = ros::Time(0);
    
    try {

        geometry_msgs::PoseStamped poseOnMap;

        tfListener_.transformPose(costmap.header.frame_id, poseCopy, poseOnMap);

        tf::Vector3 pixel(poseOnMap.pose.position.x, 
                poseOnMap.pose.position.y, 0.0);

        pixel -= tf::Vector3(costmap.info.origin.position.x, costmap.info.origin.position.y, 0.0);
        pixel /= costmap.info.resolution;
        
        uint32_t pixelIndex = (int)pixel.x() + ((int)pixel.y()) * costmap.info.width;

        if (pixelIndex < costmap.data.size()) {
            return costmap.data[pixelIndex];
        } else {
            ROS_ERROR("Pixel out of map");
        }

    } catch (std::exception& e) {
        ROS_ERROR("Wandering: failed to transform trajectory");
        ROS_ERROR("%s", e.what());
        return 100;
    }
    
    return 0;

}

TrajectoryMatch::Ptr SimpleTrajectoryMatcher::match(const nav_msgs::OccupancyGrid& costMap,
		const Trajectory::Ptr& trajectory) const {

	const nav_msgs::Path::Ptr& path = trajectory->getPath();

	/**
	 * Score ranges from -1 (Unknown) to 100 (definitely occupied)
	 */
	const int pointScoreRange = 101;

	/**
	 * Maximum possible trajectory weight
	 */
	const double maxTrajectoryWeight = 1;

	/**
	 * Maximum score, used to normalize the value to range 0..1
	 */
	double maxPathScore = path->poses.size() * pointScoreRange * maxTrajectoryWeight;

	bool fatalPath = false;
	double scoreSum = 0;

	for (int i = 0; i < path->poses.size(); ++i) {
		auto& pose = path->poses[i];

		double pointValue = 1.0 + (double)getCellValue(costMap, pose); 

		/**
		 * Fatal path check
		 */
		if (pointValue >= 100) {
			/**
			 * The path is definitely blocked, mark it as fatal
			 */
			fatalPath = true;
		}

		scoreSum += pointValue;
	}

	/**
	 * Empty path
	 */
	if (maxPathScore == 0)
		return TrajectoryMatch::Ptr(new TrajectoryMatch(trajectory, 0));

	/**
	 * Normalize and invert
	 */
	double finalScore = (1 - scoreSum / maxPathScore) * trajectory->getWeight();

	/**
	 * Fatal path, adjust the score to be in range [-1,0]
	 */
	if (fatalPath)
		finalScore -= 1;

	TrajectoryMatch::Ptr trajectoryMatch(new TrajectoryMatch(trajectory, finalScore));
	return trajectoryMatch;
}
