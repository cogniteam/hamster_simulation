/*
 * RosParametersProvider.h
 *
 *  Created on: Dec 3, 2014
 *      Author: blackpc
 */

#ifndef ROSPARAMETERSPROVIDER_H_
#define ROSPARAMETERSPROVIDER_H_


#include <ros/ros.h>

#include <hupster_wandering/costmap/parameters/IParametersProvider.h>


/*
 * Cost map parameters provider, that fetches parameters from ROS parameters server
 */
class RosParametersProvider: public IParametersProvider {

public:

	/**
	 *
	 * @param ns
	 */
	inline RosParametersProvider(const string& ns = "costmap")
		: nodePrivate_("~/" + ns) {  }

	virtual ~RosParametersProvider() { }

public:

	/**
	 * Gets map width in meters
	 * @return
	 */
	virtual inline double getMapWidth() const {
		double mapWidth;
		nodePrivate_.param("width", mapWidth, 3.0);
		return mapWidth;
	}

	/**
	 * Gets map height in meters
	 * @return
	 */
	virtual inline double getMapHeight() const {
		double mapHeight;
		nodePrivate_.param("height", mapHeight, 3.0);
		return mapHeight;
	}

	/**
	 * Gets map resolution in p/m
	 * @return
	 */
	virtual inline double getMapResolution() const {
		double resolution;
		nodePrivate_.param("resolution", resolution, 0.025);
		return resolution;
	}

	/**
	 * Gets obstacle inflation radius in meters
	 * @return
	 */
	virtual inline double getInflationRadius() const {
		double inflation;
		nodePrivate_.param("inflation", inflation, 0.25);
		return inflation;
	}

	/**
	 * Gets the frame id of the cost map
	 * @return
	 */
	virtual inline string getMapFrameId() const {
		string frameId;
		nodePrivate_.param("frame_id", frameId, string("base_link"));
		return frameId;
	}

private:

	ros::NodeHandle nodePrivate_;

};

#endif /* ROSPARAMETERSPROVIDER_H_ */
