/**
 * Filename: ICostMapDataSource.h
 *   Author: Igor Makhtes
 *     Date: Nov 28, 2014
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

#ifndef INCLUDE_WANDERER_ICOSTMAPDATASOURCE_H_
#define INCLUDE_WANDERER_ICOSTMAPDATASOURCE_H_


#include <string>

#include <boost/function.hpp>

#include <ros/ros.h>


using namespace std;


//class CostMap;


/*
 * Data source interface for CostMap
 */
class ICostMapDataSource {

//	friend CostMap;

public:

	virtual ~ICostMapDataSource() { }

public:

	/**
	 * Name of this data source
	 * @return
	 */
	virtual string getName() const = 0;

	/**
	 * Clears cost map
	 */
	inline void clearMap() const {
		clearMapCallback();
	}

	/**
	 * Publishes a point to the cost map
	 * @param x
	 * @param y
	 * @param frameId
	 * @param stamp
	 */
	inline void emitPoint(double x, double y, const string& frameId, const ros::Time& stamp) const {
		emitPointCallback(x, y, boost::ref(frameId), boost::ref(stamp));
	}

//private:
public:

	typedef boost::function<void()> ClearMapCallback;
	typedef boost::function<void(double, double, const string&, const ros::Time&)> EmitPointCallback;

	ClearMapCallback clearMapCallback;
	EmitPointCallback emitPointCallback;

};

#endif /* INCLUDE_WANDERER_ICOSTMAPDATASOURCE_H_ */
