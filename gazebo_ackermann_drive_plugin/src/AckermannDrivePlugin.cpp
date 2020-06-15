/*
 *    Filename: AckermannDrivePlugin.cpp
 *  Created on: Jun 2, 2020
 *      Author: nix <daria@cogniteam.com>
 */

#include <AckermannDrivePlugin.h>


namespace gazebo{


AckermannDrivePlugin::AckermannDrivePlugin() : 
    
    steeringfrontLeftPid_(10, 0.0, 0.01),
    steeringfrontRightPid_(10, 0.0, 0.01), wheelRadius_(0.029)
    {
    steeringfrontLeftPid_.SetCmdMax(10);
    steeringfrontRightPid_.SetCmdMax(10);

    steeringfrontLeftPid_.SetCmdMin(-10);
    steeringfrontRightPid_.SetCmdMin(-10);
    
    currentCommand_.drive.speed = 0.0;
    currentCommand_.drive.steering_angle = 0.0;
}

AckermannDrivePlugin::~AckermannDrivePlugin() {

}

void AckermannDrivePlugin::commandCallback(
    const ackermann_msgs::AckermannDriveStamped::Ptr& cmd) {

    currentCommand_ = *cmd;
}

void AckermannDrivePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {

    robotNamespace_ = sdf->Get<std::string>("robotNamespace");

    ros::NodeHandle node;

    cmdSubscriber_ = node.subscribe(
        robotNamespace_ + "/ackermann_cmd", 2, &AckermannDrivePlugin::commandCallback, this);

    odomPublisher_ = node.advertise<nav_msgs::Odometry>("odom", 5, false);
  
    this->model_ = model;

    this->model_->GetJointController()->SetPositionPID(
        this->model_->GetJoint(robotNamespace_ + "::" +
            "front_right_wheel_steering_joint")->GetScopedName(), 
                steeringfrontRightPid_);

    this->model_->GetJointController()->SetPositionPID(
        this->model_->GetJoint(robotNamespace_ + "::" + 
            "front_left_wheel_steering_joint")->GetScopedName(), 
                steeringfrontLeftPid_);

    this->model_->GetJointController()->SetPositionTarget(
            robotNamespace_ + "::" + "front_right_wheel_steering_joint", 0.0);

    this->model_->GetJointController()->SetPositionTarget(
        robotNamespace_ + "::" + "front_left_wheel_steering_joint", 0.0);

    this->model_->GetJoint(robotNamespace_ + "::" + "rear_right_wheel_joint")->
        SetParam("fmax", 0, (double)0.01);
    this->model_->GetJoint(robotNamespace_ + "::" + "rear_right_wheel_joint")->
        SetParam("vel", 0, 0.0);
    this->model_->GetJoint(robotNamespace_ + "::" + "rear_left_wheel_joint")-> 
        SetParam("fmax", 0, (double)0.01);
    this->model_->GetJoint(robotNamespace_ + "::" + "rear_left_wheel_joint")-> 
        SetParam("vel", 0, 0.0);
    this->model_->GetJoint(robotNamespace_ + "::" + "front_left_wheel_joint")->
        SetParam("fmax", 0, (double)0.01);
    this->model_->GetJoint(robotNamespace_ + "::" + "front_left_wheel_joint")->
        SetParam("vel", 0, 0.0);
    this->model_->GetJoint(robotNamespace_ + "::" + "front_right_wheel_joint")->
        SetParam("fmax", 0,(double)0.01);
    this->model_->GetJoint(robotNamespace_ + "::" + "front_right_wheel_joint")->
        SetParam("vel", 0, 0.0);

    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&AckermannDrivePlugin::Update, this, std::placeholders::_1));

    gzdbg << "AckermannDrive plugin loaded!" << endl;
}

void AckermannDrivePlugin::Update(const common::UpdateInfo &info) {

     if (lastUpdate_ == common::Time()) {

        lastUpdate_ = info.simTime;

        return;
    }

    double timeDeltaSec = (info.simTime - lastUpdate_).Double();

    if (timeDeltaSec > 0.02) {

        //
        // Steering constraince
        //

        auto steering = fmax(-0.23, fmin(0.23, currentCommand_.drive.steering_angle));

        this->model_->GetJointController()->SetPositionTarget(
            this->model_->GetJoint(robotNamespace_ + "::" + 
                "front_right_wheel_steering_joint")->GetScopedName(), 
                    rightWheelSteering((double)steering));

        this->model_->GetJointController()->SetPositionTarget(
            this->model_->GetJoint(robotNamespace_ + "::" + 
                "front_left_wheel_steering_joint")->GetScopedName(), 
                    leftWheelSteering((double)steering));

        this->model_->GetJoint(robotNamespace_ + "::" + "rear_right_wheel_joint")->
        SetParam("fmax", 0, (double)0.01);
        this->model_->GetJoint(robotNamespace_ + "::" + "rear_left_wheel_joint")->
        SetParam("fmax", 0, (double)0.01);
        this->model_->GetJoint(robotNamespace_ + "::" + "front_right_wheel_joint")->
        SetParam("fmax", 0, (double)0.01);
        this->model_->GetJoint(robotNamespace_ + "::" + "front_left_wheel_joint")->
        SetParam("fmax", 0, (double)0.01);

        this->model_->GetJoint(
            robotNamespace_ + "::" + "rear_left_wheel_joint")->SetParam(
                "vel", 0, (double)currentCommand_.drive.speed / wheelRadius_);

        this->model_->GetJoint(
            robotNamespace_ + "::" + "rear_right_wheel_joint")->SetParam(
                "vel", 0, (double)currentCommand_.drive.speed / wheelRadius_);
        
        this->model_->GetJoint(
            robotNamespace_ + "::" + "front_left_wheel_joint")->SetParam(
                "vel", 0, (double)currentCommand_.drive.speed / wheelRadius_);

        this->model_->GetJoint(
            robotNamespace_ + "::" + "front_right_wheel_joint")->SetParam(
                "vel", 0, (double)currentCommand_.drive.speed / wheelRadius_);
            
        publishOdometry();
   
        ros::spinOnce();
        
        lastUpdate_ = info.simTime;
    }
}

inline double AckermannDrivePlugin::rightWheelSteering(double baseAngle) {

    return atan(2 * 0.17 * sin(baseAngle)/ (2 * 0.17 * cos(baseAngle) - 0.166 * sin(baseAngle)));
}

inline double AckermannDrivePlugin::leftWheelSteering(double baseAngle) {
    
    return atan(2 * 0.17 * sin(baseAngle)/ (2 * 0.17 * cos(baseAngle) + 0.166 * sin(baseAngle)));
}

void AckermannDrivePlugin::publishOdometry() {

    auto position = this->model_->GetLink(
        robotNamespace_ + "::" + "base_link")->WorldPose().Pos();

    auto rotation = this->model_->GetLink(
        robotNamespace_ + "::" + "base_link")->WorldPose().Rot();

    tf::Transform transform;

    transform.setOrigin(tf::Vector3(position.X(), position.Y(), position.Z()));
    transform.setRotation(tf::Quaternion(
        rotation.X(), rotation.Y(), rotation.Z(), rotation.W()));

    tfBroadcaster_.sendTransform(tf::StampedTransform(
        transform, ros::Time::now(), robotNamespace_ + "/odom", robotNamespace_ + "/base_link"));

    nav_msgs::Odometry odomMgs;

    odomMgs.header.frame_id = "odom";
    odomMgs.header.stamp = ros::Time::now();

}

} // namespace gazebo

