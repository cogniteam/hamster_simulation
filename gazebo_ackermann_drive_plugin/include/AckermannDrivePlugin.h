/*
 *    Filename: AckermannDrivePlugin.h
 *  Created on: Jun 2, 2020
 *      Author: nix <daria@cogniteam.com>
 *
 */


#ifndef GAZEBO_ACKERMANN_DRIVE_PLUGIN_H_
#define GAZEBO_ACKERMANN_DRIVE_PLUGIN_H_


#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <cmath>
#include <ignition/math/Vector3.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsTypes.hh>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>


namespace gazebo{

using namespace std;

class AckermannDrivePlugin : public ModelPlugin {

public:

    AckermannDrivePlugin();

    virtual ~AckermannDrivePlugin();

public:

    /**
     * @brief Overriden function from base class. Loads init joint states
     * 
     * @param model gazebo model 
     * @param sdf 
     */

    virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

    /**
     * @brief Joints speed/rotation update with timer
     * 
     * @param info
     */

    void Update(const common::UpdateInfo &info);

private:
    
    /**
     * @brief Set parameters from urdf and initial values 
     * 
     * @param sdf - model description
     */

    void initParams(sdf::ElementPtr sdf);

    /**
     * @brief Saves steering and velocity
     * 
     * @param cmd - received drive command
     */

    void commandCallback(const ackermann_msgs::AckermannDriveStamped::Ptr& cmd);

    /**
     * @brief Right wheel steering calculation with ackermann behavior model
     * 
     * @param baseAngle - base steer angle 
     * @return double - right wheel steering
     */

    inline double rightWheelSteering(double baseAngle);

    /**
     * @brief Left wheel steering calculation with ackermann behavior model
     * 
     * @param baseAngle  - base steer angle
     * @return double - left wheel steering
     */

    inline double leftWheelSteering(double baseAngle);

    /**
     * @brief odometry -> base transform and topic publishing
     * Data receiving from gazebo model 
     */

    void publishOdometry();

    /**
     * @brief  
     */
    template<typename T>
    T getParam(const std::string& paramName, T initParam, sdf::ElementPtr sdf) {
        auto param = initParam;
        if (sdf->HasElement(paramName)) {
            param = sdf->GetElement(paramName)->Get<T>();
        }
        gzerr << paramName << "is set to" << param << endl;
        return param;
    }
    
private:

    /**
     * @brief Drive command ros subscriber 
     */

    ros::Subscriber cmdSubscriber_;

    /**
     * @brief Odometry ros topic publisher 
     */

    ros::Publisher odomPublisher_;
    
    /**
     * @brief Odometry ft publisher 
     */

    tf::TransformBroadcaster tfBroadcaster_;


    /**
     * @brief Gazebo hamster model, described in urdf 
     */

    physics::ModelPtr model_;

    /**
     * @brief Gazebo update event
     */

    event::ConnectionPtr updateConnection_;


    /**
     * @brief Last time for gazebo update loop
     */

    common::Time lastUpdate_ = common::Time();

    /**
     * @brief Drive command 
     */

    ackermann_msgs::AckermannDriveStamped currentCommand_;

    /**
     * @brief Left/right front wheel PID  
     */

    common::PID steeringfrontLeftPid_;
    common::PID steeringfrontRightPid_;

    /**
     * @brief Gazebo plugin parameters
     */

    double wheelRadius_;

    std::string robotNamespace_;

    std::string frontLeftWheelJoint_;
    std::string frontRightWheelJoint_;
    std::string rearLeftWheelJoint_;
    std::string rearRightWheelJoint_;

    std::string frontRightWheelSteeringJoint_;
    std::string frontLeftWheelSteeringJoint_;

    std::string baseFrame_;
    std::string odomFrame_;

    std::string driveTopic_;
    std::string odomTopic_;

    double torque_;

};

GZ_REGISTER_MODEL_PLUGIN(AckermannDrivePlugin)
    
}

#endif // GAZEBO_ACKERMANN_DRIVE_PLUGIN_H_

