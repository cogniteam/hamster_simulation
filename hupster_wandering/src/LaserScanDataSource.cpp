/**
 * Filename: LaserScanDataSource.cpp
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

#include <hupster_wandering/costmap/datasource/LaserScanDataSource.h>

LaserScanDataSource::LaserScanDataSource(ros::NodeHandle& nodeHandle,
		const string& topic) {

	laserScanSubscriber_ = nodeHandle.subscribe(
			topic, 1, &LaserScanDataSource::laserScanCallback, this);

}

void LaserScanDataSource::laserScanCallback(
		const sensor_msgs::LaserScan::ConstPtr& scan)
{
	ICostMapDataSource::clearMap();

	for (int i = 0; i < scan->ranges.size(); ++i) {
		if (scan->ranges[i] <= scan->range_max && scan->ranges[i] >= scan->range_min) {
			double x = cos(scan->angle_min + scan->angle_increment * i) * scan->ranges[i];
			double y = sin(scan->angle_min + scan->angle_increment * i) * scan->ranges[i];

			ICostMapDataSource::emitPoint(x, y, scan->header.frame_id, scan->header.stamp);
		}
	}
}
