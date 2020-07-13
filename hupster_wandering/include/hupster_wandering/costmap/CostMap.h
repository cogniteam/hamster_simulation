/**
 * Filename: CostMap.h
 *   Author: Igor Makhtes
 *     Date: Nov 26, 2014
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

#ifndef INCLUDE_WANDERER_COSTMAP_H_
#define INCLUDE_WANDERER_COSTMAP_H_


#include <string>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>

#include <hupster_wandering/costmap/datasource/ICostMapDataSource.h>
#include <hupster_wandering/costmap/parameters/IParametersProvider.h>


using namespace std;


/**
 * Generic cost map representation.
 * Can be used with various data sources
 * @see IDataSource
 */
class CostMap {

public:

	/**
	 * Initializes the cost map with specified parameters
	 * @param dataSource Data source for cost map filling
	 * @param inflationRadius Inflation radius in meters
	 * @param mapWidth Map width in meters
	 * @param mapHeight Map height in meters
	 * @param resolution
	 * @param frameId Robot's frame id, all points from the ICostMapDataSource will be transformed into this frame.
	 * 				  It also will be the frame id of the published map
	 * @warning Currently the origin of the map it the center
	 * @todo add originX, originY
	 */
	CostMap(ICostMapDataSource* dataSource, double inflationRadius, double mapWidth,
			double mapHeight, double resolution, const string& frameId);

	/**
	 * Initializes the cost map with specified parameters
	 * @param dataSource
	 * @param parametersProvider
	 */
	CostMap(ICostMapDataSource* dataSource, IParametersProvider* parametersProvider);

	/**
	 * Deletes data source and parameters provider pointers if needed
	 */
	virtual ~CostMap();

public:

	/**
	 * Return occupancy grid
	 * @return occupancy grid
	 */
	inline nav_msgs::OccupancyGrid::ConstPtr getOccupancyGrid() const {
		return boost::const_pointer_cast<nav_msgs::OccupancyGrid const>(occupancyGrid_);
	}

	/**
	 * Returns cv::Mat of the cost map
	 * @return
	 */
	inline cv::Mat getCvMatrix() const {
		return cvMatrix_;
	}

	/**
	 * Set all cells of the map to free(0)
	 */
	inline void clearMap() {
		memset(occupancyGrid_->data.data(), 0, occupancyGrid_->data.size());
	}

	/**
	 * Returns the value of cost map cell, located in specified position
	 * @param pose Position should be in cost map's frame
	 * @return Cell value: Unknown(-1), Free(0), Occupied(1..100)
	 */
	signed char getCellValue(const geometry_msgs::Pose& pose) const;

protected:

	/**
	 * Data source for the cost map
	 */
	ICostMapDataSource* dataSource_;

	/**
	 * Cost map parameters provider
	 */
	IParametersProvider* parametersProvider_;

	/**
	 * Holds the grid array
	 */
	nav_msgs::OccupancyGrid::Ptr occupancyGrid_;

	/**
	 * Uses the array of occupancy grid
	 */
	cv::Mat cvMatrix_;

	/**
	 * Transform listener
	 */
	tf::TransformListener tfListener_;

	/**
	 * Inflation radius in meters
	 */
	double inflationRadius_;

protected:

	/**
	 * Initializes the cost map with specified parameters
	 * @param dataSource Data source for cost map filling
	 * @param parametersProvider Cost map parameters provider
	 * @param inflationRadius Inflation radius in meters
	 * @param mapWidth Map width in meters
	 * @param mapHeight Map height in meters
	 * @param resolution
	 * @param frameId Robot's frame id, all points from the ICostMapDataSource will be transformed into this frame.
	 * 				  It also will be the frame id of the published map
	 * @warning Currently the origin of the map it the center
	 * @todo add originX, originY
	 */
	void initializeCostMap(ICostMapDataSource* dataSource,
			IParametersProvider* parametersProvider,
			double inflationRadius, double mapWidth,
			double mapHeight, double resolution, string frameId);

	/**
	 * Prints summary to ROS console
	 */
	void printSummary() const;

	/**
	 * Creates the occupancy grid message
	 * @param mapWidth
	 * @param mapHeight
	 * @param resolution
	 * @param frameId
	 */
	void createOccupancyGrid(double mapWidth,
			double mapHeight, double resolution, const string& frameId);

	/**
	 * Clear cost map callback from ICostMapDataSource
	 */
	inline void clearMapCallback() {
		clearMap();
	}

	/**
	 * New point callback from ICostMapDataSource
	 * @param x x coordinate
	 * @param y y coordinate
	 * @param frameId frame id of the coordinates
	 * @param stamp time stamp
	 */
	void addPointCallback(double x, double y, const string& frameId, const ros::Time& stamp);

	/**
	 * Convert local map coordinates to pixels coordinates
	 * @param point
	 * @return
	 */
	inline cv::Point localCoordinatesToPixel(const tf::Vector3& point) const {
		return localCoordinatesToPixel(point.x(), point.y());
	}

	/**
	 * Convert local map coordinates to pixels coordinates
	 * @param x
	 * @param y
	 * @return
	 */
	inline cv::Point localCoordinatesToPixel(double x, double y) const {
		return cv::Point(
				(x - occupancyGrid_->info.origin.position.x) /
					occupancyGrid_->info.resolution,
				(y - occupancyGrid_->info.origin.position.y) /
					occupancyGrid_->info.resolution);
	}

	/**
	 * Checks whether provided pixel within the map bounds
	 * @param pixel
	 * @return If the point within the map bounds - the value returned,
	 * 		   otherwise Unknown(-1)
	 */
	inline bool isInBounds(const cv::Point& pixel) const {
		return pixel.x >= 0 && pixel.x < cvMatrix_.cols &&
				pixel.y >= 0 && pixel.y < cvMatrix_.rows;
	}
};

#endif /* INCLUDE_WANDERER_COSTMAP_H_ */
