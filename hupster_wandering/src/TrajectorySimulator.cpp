/**
 * Filename: TrajectorySimulator.cpp
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

#include <hupster_wandering/trajectory/simulator/TrajectorySimulator.h>

TrajectorySimulator::TrajectorySimulator(double simulationTime, double granularity)
	: simulationTime_(simulationTime), granularity_(granularity)
{
}

Trajectory::Ptr TrajectorySimulator::simulate(const IMotionModel* model) const
{
	Trajectory::Ptr trajectory(new Trajectory(model));

	tf::Transform position;
	position.setIdentity();

	int steps = simulationTime_ / granularity_;
	float timeStep = granularity_;

	tf::Transform velocityVector = model->simulateStep(timeStep);

	for (int step = 0; step < steps; ++step) {
		position = position * velocityVector;
		trajectory->addPosition(position.getOrigin(), position.getRotation());

		if (!model->isConstantMotion())
			velocityVector = model->simulateStep(timeStep);
	}

	return trajectory;
}

//Trajectory::Ptr TrajectorySimulator::simulate(double linearVelocity,
//		double angularVelocity) const
//{
//	Trajectory::Ptr trajectory(new Trajectory(NULL));
//
//	int steps = simulationTime_ / granularity_;
//	float timeStep = granularity_;
//
//	tf::Transform position;
//	position.setIdentity();
//
//	tf::Transform velocityVector =
//			createVelocityTransform(linearVelocity, angularVelocity, timeStep);
//
//	for (int step = 0; step < steps; ++step) {
//		position = position * velocityVector;
//		trajectory->addPosition(position.getOrigin(), position.getRotation());
//	}
//
//	return trajectory;
//}

tf::Transform TrajectorySimulator::createVelocityTransform(
		double linearVelocity, double angularVelocity, double timeStep) const {

	tf::Transform velocityVector;

	velocityVector.setOrigin(tf::Vector3(linearVelocity * timeStep, 0, 0));

	tf::Quaternion rotationVelocity;
	rotationVelocity.setRPY(0, 0, angularVelocity *  timeStep);

	velocityVector.setRotation(rotationVelocity);

	return velocityVector;
}
