/**
 * Filename: Trajectory.cpp
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

#include <hupster_wandering/trajectory/Trajectory.h>


Trajectory::Trajectory(const IMotionModel* motionModel, double weight)
	: weight_(weight), motionModel_(const_cast<IMotionModel*>(motionModel)) {
	path_ = nav_msgs::Path::Ptr(new nav_msgs::Path());
}

void Trajectory::addPosition(double x, double y) {
	geometry_msgs::PoseStamped newPose;

	newPose.header.stamp = ros::Time(0);
	newPose.header.frame_id = frame_;
	newPose.pose.orientation.w = 1;
	newPose.pose.position.x = x;
	newPose.pose.position.y = y;

	path_->poses.push_back(newPose);
}

void Trajectory::addPosition(const tf::Vector3& position,
		const tf::Quaternion& orientation)
{
	geometry_msgs::PoseStamped newPose;

	// newPose.header.frame_id = ""; // set in getPath()
	newPose.header.stamp = ros::Time(0);
    newPose.header.frame_id = frame_;

	newPose.pose.position.x = position.x();
	newPose.pose.position.y = position.y();
	newPose.pose.position.z = position.z();

	newPose.pose.orientation.x = orientation.x();
	newPose.pose.orientation.y = orientation.y();
	newPose.pose.orientation.z = orientation.z();
	newPose.pose.orientation.w = orientation.w();

	path_->poses.push_back(newPose);
}

nav_msgs::Path::Ptr Trajectory::getPath(bool updateStamp, const string& frameId) {
	path_->header.frame_id = frameId;

	for(auto&& pose : path_->poses) {
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = frameId;
	}

	return path_;
}

void Trajectory::setWeight(double weight) {
	if (weight < 0 || weight > 1)
		throw new std::invalid_argument("weight");

	weight_ = weight;
}

void Trajectory::rotate(double angle) {
	tf::Stamped<tf::Transform> rotationTf;
	rotationTf.setOrigin(tf::Vector3(0, 0, 0));
	rotationTf.setRotation(tf::createQuaternionFromYaw(angle));

	for(auto&& pose : path_->poses) {
		tf::Stamped<tf::Transform> tf;
		tf::poseStampedMsgToTF(pose, tf);
		tf.setData(rotationTf * tf);
		tf::poseStampedTFToMsg(tf, pose);
	}
}
