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

    ROS_INFO("frontRightWheelSteeringJoint_: %s", frontRightWheelSteeringJoint_.c_str());
    ROS_INFO("frontLeftWheelSteeringJoint_: %s", frontLeftWheelSteeringJoint_.c_str());
    ROS_INFO("rearLeftWheelJoint_: %s", rearLeftWheelJoint_.c_str());
    ROS_INFO("rearRightWheelJoint_: %s", rearRightWheelJoint_.c_str());
    ROS_INFO("frontLeftWheelJoint_: %s", frontLeftWheelJoint_.c_str());
    ROS_INFO("frontRightWheelJoint_: %s", frontRightWheelJoint_.c_str());

    this->model_->GetJointController()->SetPositionPID(
        this->model_->GetJoint(frontRightWheelSteeringJoint_)->GetScopedName(), 
                steeringfrontRightPid_);

    this->model_->GetJointController()->SetPositionPID(
        this->model_->GetJoint(frontLeftWheelSteeringJoint_)->GetScopedName(), 
                steeringfrontLeftPid_);

    this->model_->GetJointController()->SetPositionTarget(
        frontRightWheelSteeringJoint_, 0.0);

    this->model_->GetJointController()->SetPositionTarget(
        frontLeftWheelSteeringJoint_, 0.0);

    this->model_->GetJoint(rearLeftWheelJoint_)->SetParam(
        "fmax", 0, (double)torque_);
    this->model_->GetJoint(rearLeftWheelJoint_)->SetParam(
        "vel", 0, 0.0);
    this->model_->GetJoint(rearRightWheelJoint_)->SetParam(
        "fmax", 0, (double)torque_);
    this->model_->GetJoint(rearRightWheelJoint_)->SetParam(
        "vel", 0, 0.0);
    this->model_->GetJoint(frontLeftWheelJoint_)->SetParam(
        "fmax", 0, (double)torque_);
    this->model_->GetJoint(frontLeftWheelJoint_)->SetParam(
        "vel", 0, 0.0);
    this->model_->GetJoint(frontRightWheelJoint_)->SetParam(
        "fmax", 0,(double)torque_);
    this->model_->GetJoint(frontRightWheelJoint_)->SetParam(
        "vel", 0, 0.0);

    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&AckermannDrivePlugin::Update, this, std::placeholders::_1));

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
    return atan(2 * lateralWheelSeparation_ * sin(baseAngle)/ (
            2 * lateralWheelSeparation_ * cos(baseAngle) - 
                longitudalWheelSeparation_ * sin(baseAngle)));
}

inline double AckermannDrivePlugin::leftWheelSteering(double baseAngle) {
    return atan(2 * lateralWheelSeparation_ * sin(baseAngle)/ (
            2 * lateralWheelSeparation_ * cos(baseAngle) + 
                longitudalWheelSeparation_ * sin(baseAngle)));
}

void AckermannDrivePlugin::publishOdometry() {

    auto position = this->model_->GetLink(baseLink_)->WorldPose().Pos();
    auto rotation = this->model_->GetLink(baseLink_)->WorldPose().Rot();


    tf::Transform transform;

    transform.setOrigin(tf::Vector3(position.X(), position.Y(), position.Z()));
    transform.setRotation(tf::Quaternion(
        rotation.X(), rotation.Y(), rotation.Z(), rotation.W()));

    tfBroadcaster_.sendTransform(tf::StampedTransform(
        transform, ros::Time::now(), odomFrame_, baseFrame_));

    // nav_msgs::Odometry odomMgs;

    // odomMgs.header.frame_id = odomFrame_;
    // odomMgs.header.stamp = ros::Time::now();
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
    lateralWheelSeparation_ = getParam<double>("lateralSeparation", 0.17, sdf);
    longitudalWheelSeparation_ = getParam<double>("longitudalSeparation", 0.166, sdf);

    baseFrame_ = getParam<std::string>("baseFrame", "base_link", sdf);
    odomFrame_ = getParam<std::string>("odomFrame", "odom", sdf);

    odomTopic_ = getParam<std::string>("odomTopic", "odom", sdf);
    driveTopic_ = getParam<std::string>("driveTopic", "ackermann_cmd", sdf);

    baseLink_ = getParam<std::string>("robotBaseLink", "base_link", sdf);

    if (!robotNamespace_.empty()) {
        frontLeftWheelJoint_ = robotNamespace_ + "::" + frontLeftWheelJoint_;
        frontRightWheelJoint_ = robotNamespace_ + "::" + frontRightWheelJoint_;
        rearLeftWheelJoint_ = robotNamespace_ + "::" + rearLeftWheelJoint_;
        rearRightWheelJoint_ = robotNamespace_ + "::" + rearRightWheelJoint_;

        frontRightWheelSteeringJoint_ =
                robotNamespace_ + "::" + frontRightWheelSteeringJoint_; 
        frontLeftWheelSteeringJoint_ =
                robotNamespace_ + "::" + frontLeftWheelSteeringJoint_;

        baseFrame_ = tf::resolve(robotNamespace_, baseFrame_);
        odomFrame_ = tf::resolve(robotNamespace_, odomFrame_);

        baseLink_ = robotNamespace_ + "::" + baseLink_;

    }

}

} // namespace gazebo

