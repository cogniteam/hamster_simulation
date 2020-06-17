/*
 *    Filename: AckermannDrivePlugin.cpp
 *  Created on: Jun 2, 2020
 *      Author: nix <daria@cogniteam.com>
 */

#include <AckermannDrivePlugin.h>


namespace gazebo{


AckermannDrivePlugin::AckermannDrivePlugin() : 
    
    steeringfrontLeftPid_(10, 0.0, 0.01),
    steeringfrontRightPid_(10, 0.0, 0.01)
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

    initParams(sdf);
    
    ros::NodeHandle node(robotNamespace_);

    cmdSubscriber_ = node.subscribe(
        driveTopic_, 2, &AckermannDrivePlugin::commandCallback, this);

    odomPublisher_ = node.advertise<nav_msgs::Odometry>(odomTopic_, 5, false);
  
    this->model_ = model;

    // this->model_->GetJointController()->SetPositionPID(
    //     this->model_->GetJoint(frontRightWheelSteeringJoint_)->GetScopedName(), 
    //             steeringfrontRightPid_);

    // this->model_->GetJointController()->SetPositionPID(
    //     this->model_->GetJoint(frontLeftWheelSteeringJoint_)->GetScopedName(), 
    //             steeringfrontLeftPid_);

    // this->model_->GetJointController()->SetPositionTarget(
    //     frontRightWheelSteeringJoint_, 0.0);

    // this->model_->GetJointController()->SetPositionTarget(
    //     frontLeftWheelSteeringJoint_, 0.0);

    // this->model_->GetJoint(rearLeftWheelJoint_)->SetParam(
    //     "fmax", 0, (double)torque_);
    // this->model_->GetJoint(rearLeftWheelJoint_)->SetParam(
    //     "vel", 0, 0.0);
    // this->model_->GetJoint(rearRightWheelJoint_)->SetParam(
    //     "fmax", 0, (double)torque_);
    // this->model_->GetJoint(rearRightWheelJoint_)->SetParam(
    //     "vel", 0, 0.0);
    // this->model_->GetJoint(frontLeftWheelJoint_)->SetParam(
    //     "fmax", 0, (double)torque_);
    // this->model_->GetJoint(frontLeftWheelJoint_)->SetParam(
    //     "vel", 0, 0.0);
    // this->model_->GetJoint(frontRightWheelJoint_)->SetParam(
    //     "fmax", 0,(double)torque_);
    // this->model_->GetJoint(frontRightWheelJoint_)->SetParam(
    //     "vel", 0, 0.0);

    // this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
    //     std::bind(&AckermannDrivePlugin::Update, this, std::placeholders::_1));

    ROS_INFO("AckermannDrive plugin loaded!");
    
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
            this->model_->GetJoint(frontRightWheelSteeringJoint_)->GetScopedName(), 
                    rightWheelSteering((double)steering));

        this->model_->GetJointController()->SetPositionTarget(
            this->model_->GetJoint(frontLeftWheelSteeringJoint_)->GetScopedName(), 
                    leftWheelSteering((double)steering));

        this->model_->GetJoint(rearRightWheelJoint_)->SetParam(
            "fmax", 0, (double)torque_);
        this->model_->GetJoint(rearLeftWheelJoint_)->SetParam(
            "fmax", 0, (double)torque_);
        this->model_->GetJoint(frontRightWheelJoint_)->SetParam(
            "fmax", 0, (double)torque_);
        this->model_->GetJoint(frontLeftWheelJoint_)->SetParam(
            "fmax", 0, (double)torque_);

        this->model_->GetJoint(
            rearLeftWheelJoint_)->SetParam("vel", 0,
                (double)currentCommand_.drive.speed / wheelRadius_);

        this->model_->GetJoint(
            rearRightWheelJoint_)->SetParam("vel", 0, 
                (double)currentCommand_.drive.speed / wheelRadius_);
        
        this->model_->GetJoint(
            frontLeftWheelJoint_)->SetParam("vel", 0, 
                (double)currentCommand_.drive.speed / wheelRadius_);

        this->model_->GetJoint(
            frontRightWheelJoint_)->SetParam("vel", 0, 
                (double)currentCommand_.drive.speed / wheelRadius_);
            
        publishOdometry();
        ros::spinOnce();
        lastUpdate_ = info.simTime;
    }
}

inline double AckermannDrivePlugin::rightWheelSteering(double baseAngle) {
    return atan(2 * 0.17 * sin(baseAngle)/ (2 * 0.17 * cos(baseAngle) - 
        0.166 * sin(baseAngle)));
}

inline double AckermannDrivePlugin::leftWheelSteering(double baseAngle) {
    return atan(2 * 0.17 * sin(baseAngle)/ (2 * 0.17 * cos(baseAngle) + 
        0.166 * sin(baseAngle)));
}

void AckermannDrivePlugin::publishOdometry() {
    auto position = this->model_->GetLink(baseFrame_)->WorldPose().Pos();
    auto rotation = this->model_->GetLink(baseFrame_)->WorldPose().Rot();

    tf::Transform transform;

    transform.setOrigin(tf::Vector3(position.X(), position.Y(), position.Z()));
    transform.setRotation(tf::Quaternion(
        rotation.X(), rotation.Y(), rotation.Z(), rotation.W()));

    tfBroadcaster_.sendTransform(tf::StampedTransform(
        transform, ros::Time::now(), odomFrame_, baseFrame_));

    nav_msgs::Odometry odomMgs;

    odomMgs.header.frame_id = odomFrame_;
    odomMgs.header.stamp = ros::Time::now();
}

void AckermannDrivePlugin::initParams(sdf::ElementPtr sdf) {

    robotNamespace_ = getParam<std::string>("robotNamespace", "", sdf);
    wheelRadius_ = getParam<double>("wheelRadius", 0.029, sdf);
    frontLeftWheelJoint_ = getParam<std::string>(
        "frontLeftWheelJoint", "front_left_wheel_joint", sdf);
    frontRightWheelJoint_ = getParam<std::string>(
        "frontRightWheelJoint", "front_right_wheel_joint", sdf);
    rearLeftWheelJoint_ = getParam<std::string>(
        "reartLeftWheelJoint", "rear_left_wheel_joint", sdf);
    rearRightWheelJoint_ = getParam<std::string>(
        "rearRightWheelJoint", "rear_right_wheel_joint", sdf);

    frontRightWheelSteeringJoint_ = getParam<std::string>(
        "frontRightWheelSteeringJoint", "front_right_wheel_steering_joint", sdf);
    frontLeftWheelSteeringJoint_ = getParam<std::string>(
        "frontLeftWheelSteeringJoint", "front_left_wheel_steering_joint", sdf);

    torque_ = getParam<double>("torque", 0.01, sdf);
    baseFrame_ = getParam<std::string>("baseFrame", "base_link", sdf);
    odomFrame_ = getParam<std::string>("odomFrame", "odom", sdf);

    odomTopic_ = getParam<std::string>("odomTopic", "odom", sdf);
    driveTopic_ = getParam<std::string>("driveTopic", "ackermann_cmd", sdf);

    if (!robotNamespace_.empty()) {
        ROS_INFO("Init params with namespase");
        frontLeftWheelJoint_ = robotNamespace_ + "::" + frontLeftWheelJoint_;
        frontRightWheelJoint_ = robotNamespace_ + "::" + frontRightWheelJoint_;
        rearLeftWheelJoint_ = robotNamespace_ + "::" + rearLeftWheelJoint_;
        rearRightWheelJoint_ = robotNamespace_ + "::" + rearRightWheelJoint_;

        frontRightWheelSteeringJoint_ =
                robotNamespace_ + "::" + frontRightWheelSteeringJoint_; 
        frontLeftWheelSteeringJoint_ =
                robotNamespace_ + "::" + frontLeftWheelSteeringJoint_;
    } else {
        ROS_INFO("Init default params");
    }

    ROS_INFO("Robot namespace: %s", robotNamespace_.c_str());
    ROS_INFO("Wheel radius: %f", wheelRadius_);
    ROS_INFO("Drive topic: %s", driveTopic_.c_str());
}

} // namespace gazebo

