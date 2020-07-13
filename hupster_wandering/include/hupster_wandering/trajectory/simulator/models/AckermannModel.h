/*
 * AckermannModel.h
 *
 *  Created on: Dec 1, 2014
 *      Author: blackpc
 */

#ifndef ACKERMANNMODEL_H_
#define ACKERMANNMODEL_H_


#include <ackermann_msgs/AckermannDriveStamped.h>

#include <hupster_wandering/trajectory/simulator/models/IMotionModel.h>


/*
 * Trajectory simulation motion model based on ackermann steering model
 */
class AckermannModel : public IMotionModel {

public:

	/**
	 * Constructs the ackermann motino model
	 * @param wheelSeparation The distance between front and rear wheels in meters
	 * @param speed Speed in m/s
	 * @param steeringAngle Steering angle in radians
	 */
	AckermannModel(double wheelSeparation, double speed,
			double steeringAngle)
		: IMotionModel(true), wheelSeparation_(wheelSeparation),
		  speed_(speed), steeringAngle_(steeringAngle) { }

	/**
	 * Simulate the transformation of a single step
	 * @param timeDelta
	 * @return
	 */
	virtual tf::Transform simulateStep(double timeDelta) const {
		const double steeringAngle = steeringAngle_;
		const double distance = speed_ * timeDelta;

		double offsetX = 0;
		double offsetY = 0;
		double heading = 0;

		if (fabs(steeringAngle_) < 0.001) {
			offsetX = distance;
			offsetY = 0;
		} else {
			double radius = wheelSeparation_ / tan(steeringAngle);
			heading = distance / radius;
			offsetX = radius * sin(heading);
			offsetY = radius - radius * cos(heading);

			if (isnan(offsetX) || isnan(offsetY)) {
				offsetX = 0;
				offsetY = 0;
			}
		}

		return tf::Transform(
				tf::createQuaternionFromYaw(heading),
				tf::Vector3(offsetX, offsetY, 0));
	}

	/**
	 * Returns the distance between front and rear wheels in meters
	 * @return
	 */
	inline double getWheelSeparation() const {
		return wheelSeparation_;
	}

	/**
	 * Return the speed in m/s
	 * @return
	 */
	inline double getSpeed() const {
		return speed_;
	}

	/**
	 * Returns the steering angle in radians
	 * @return
	 */
	inline double getSteeringAngle() const {
		return steeringAngle_;
	}

	/**
	 * Returns speed and steering angle as @see ackermann_msgs::AckermannDriveStamped
	 * @return
	 */
	inline ackermann_msgs::AckermannDriveStamped getAckermannMessage() const {
		ackermann_msgs::AckermannDriveStamped message;
		message.drive.speed = speed_;
		message.drive.steering_angle = steeringAngle_ * 3.0;
		return message;
	}

private:

	double wheelSeparation_;
	double steeringAngle_;
	double speed_;

};

#endif /* ACKERMANNMODEL_H_ */
