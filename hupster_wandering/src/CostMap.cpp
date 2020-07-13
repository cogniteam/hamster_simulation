/**
 * Filename: CostMap.cpp
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

#include <hupster_wandering/costmap/CostMap.h>

CostMap::CostMap(ICostMapDataSource* dataSource, double inflationRadius, double mapWidth,
		double mapHeight, double resolution, const string& frameId) {

	initializeCostMap(dataSource, NULL, inflationRadius,
			mapWidth, mapHeight, resolution, frameId);
}

CostMap::CostMap(ICostMapDataSource* dataSource,
		IParametersProvider* parametersProvider) {
	BOOST_ASSERT_MSG(parametersProvider != NULL, "CostMap: Parameters provider is NULL");
	initializeCostMap(dataSource, parametersProvider, 0, 0, 0, 0, "");
}

CostMap::~CostMap() {
	if (dataSource_ != NULL) {
		delete dataSource_;
		dataSource_ = NULL;
	}

	if (parametersProvider_ != NULL) {
		delete parametersProvider_;
		parametersProvider_ = NULL;
	}
}

void CostMap::initializeCostMap(ICostMapDataSource* dataSource,
		IParametersProvider* parametersProvider,
		double inflationRadius, double mapWidth, double mapHeight,
		double resolution, string frameId) {

	BOOST_ASSERT_MSG(dataSource != NULL, "CostMap: Data source is NULL");

	/**
	 * If parametersProvider presents, use it
	 * to fetch parameters
	 */
	if (parametersProvider != NULL) {
		inflationRadius = parametersProvider->getInflationRadius();
		mapWidth = parametersProvider->getMapWidth();
		mapHeight = parametersProvider->getMapHeight();
		resolution = parametersProvider->getMapResolution();
		frameId = parametersProvider->getMapFrameId();
	}

	dataSource_ = dataSource;
	parametersProvider_ = parametersProvider;
	inflationRadius_ = inflationRadius;

	/**
	 * Creates map array
	 */
	createOccupancyGrid(mapWidth, mapHeight, resolution, frameId);

	/**
	 * Subscribe to data source's callbacks
	 */
	dataSource_->clearMapCallback =
			boost::bind(&CostMap::clearMapCallback, this);
	dataSource_->emitPointCallback =
			boost::bind(&CostMap::addPointCallback, this, _1, _2, _3, _4);

	printSummary();
}

void CostMap::printSummary() const {
	ROS_INFO("CostMap summary:");

	ROS_INFO(" Data source: %s", dataSource_->getName().c_str());

	ROS_INFO("       Width: %fm (%ipx)",
			occupancyGrid_->info.width * occupancyGrid_->info.resolution,
			occupancyGrid_->info.width);

	ROS_INFO("      Height: %fm (%ipx)",
			occupancyGrid_->info.height * occupancyGrid_->info.resolution,
			occupancyGrid_->info.height);

	ROS_INFO("  Resolution: %fm/px",
			occupancyGrid_->info.resolution);

	ROS_INFO("   Inflation: %fm (%ipx)",
			inflationRadius_,
			(int)(inflationRadius_ / occupancyGrid_->info.resolution));

	ROS_INFO("    Origin x: %fm (%ipx)",
			occupancyGrid_->info.origin.position.x,
			(int)(occupancyGrid_->info.origin.position.x / occupancyGrid_->info.resolution));

	ROS_INFO("    Origin y: %fm (%ipx)",
			occupancyGrid_->info.origin.position.y,
			(int)(occupancyGrid_->info.origin.position.y / occupancyGrid_->info.resolution));
}

void CostMap::createOccupancyGrid(double mapWidth,
		double mapHeight, double resolution, const string& frameId) {

	occupancyGrid_ = nav_msgs::OccupancyGrid::Ptr(
			new nav_msgs::OccupancyGrid());

	occupancyGrid_->header.frame_id = frameId;
	occupancyGrid_->info.width = mapWidth / resolution;
	occupancyGrid_->info.height = mapHeight / resolution;
	occupancyGrid_->info.resolution = resolution;
	occupancyGrid_->info.origin.position.x = -mapWidth * 0.5;
	occupancyGrid_->info.origin.position.y = -mapHeight * 0.5;
	occupancyGrid_->info.origin.orientation.w = 1;

	occupancyGrid_->data.resize(
			occupancyGrid_->info.width * occupancyGrid_->info.height);

	cvMatrix_ = cv::Mat(
			occupancyGrid_->info.height, occupancyGrid_->info.width,
			CV_8SC1, occupancyGrid_->data.data());

}

signed char CostMap::getCellValue(const geometry_msgs::Pose& pose) const {
	cv::Point pixel = localCoordinatesToPixel(pose.position.x, pose.position.y);

	if (isInBounds(pixel))
		return cvMatrix_.at<char>(pixel);

	/**
	 * Out of map points considered as unknown
	 */
	return -1;
}

void CostMap::addPointCallback(double x, double y, const string& frameId, const ros::Time& stamp) {
	tf::StampedTransform dataSourceToMapTransform;

	try {
		tfListener_.lookupTransform(occupancyGrid_->header.frame_id,
				frameId, ros::Time(0), dataSourceToMapTransform);
	}
	catch (tf::TransformException& exception) {
		ROS_ERROR("Failed to transform point: \n%s", exception.what());
		return;
	}

	tf::Vector3 localMapCoordinates = dataSourceToMapTransform * tf::Vector3(x, y, 0);
	cv::Point pixel = localCoordinatesToPixel(localMapCoordinates);

	if (pixel.x >= 0 && pixel.x < cvMatrix_.cols &&
			pixel.y >= 0 && pixel.y < cvMatrix_.rows) {
		cv::circle(cvMatrix_, pixel, inflationRadius_ / occupancyGrid_->info.resolution, 100, -1);
	}
}
