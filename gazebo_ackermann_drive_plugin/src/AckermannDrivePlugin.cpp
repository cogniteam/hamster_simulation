/*
 *    Filename: AckermannDrivePlugin.cpp
 *  Created on: Jun 2, 2020
 *      Author: nix <daria@cogniteam.com>
 *
 *
 * Cogniteam LTD CONFIDENTIAL
 *
 * Unpublished Copyright (c) 2016-2017 Cogniteam,        All Rights Reserved.
 *
 * NOTICE:  All information contained  herein  is,  and  remains the property
 * of Cogniteam.   The   intellectual   and   technical   concepts  contained
 * herein are proprietary to Cogniteam and may  be  covered  by  Israeli  and
 * Foreign Patents, patents in process,  and  are  protected  by trade secret
 * or copyright law. Dissemination of  this  information  or  reproduction of
 * this material is strictly forbidden unless  prior  written  permission  is
 * obtained  from  Cogniteam.  Access  to  the  source  code contained herein
 * is hereby   forbidden   to   anyone  except  current  Cogniteam employees,
 * managers   or   contractors   who   have   executed   Confidentiality  and
 * Non-disclosure    agreements    explicitly    covering     such     access
 *
 * The copyright notice  above  does  not  evidence  any  actual  or intended
 * publication  or  disclosure    of    this  source  code,   which  includes
 * information that is confidential  and/or  proprietary,  and  is  a   trade
 * secret, of   Cogniteam.    ANY REPRODUCTION,  MODIFICATION,  DISTRIBUTION,
 * PUBLIC   PERFORMANCE,  OR  PUBLIC  DISPLAY  OF  OR  THROUGH USE   OF  THIS
 * SOURCE  CODE   WITHOUT   THE  EXPRESS  WRITTEN  CONSENT  OF  Cogniteam  IS
 * STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND INTERNATIONAL
 * TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE  CODE  AND/OR RELATED
 * INFORMATION DOES  NOT CONVEY OR IMPLY ANY RIGHTS  TO  REPRODUCE,  DISCLOSE
 * OR  DISTRIBUTE ITS CONTENTS, OR TO  MANUFACTURE,  USE,  OR  SELL  ANYTHING
 * THAT      IT     MAY     DESCRIBE,     IN     WHOLE     OR     IN     PART
 *
 */


#include <AckermannDrivePlugin.h>


namespace gazebo{


AckermannDrivePlugin::AckermannDrivePlugin() : 
    
    steeringfrontLeftPid_(0.1, 0.0, 0.01),
    steeringfrontRightPid_(0.1, 0.0, 0.01),

    velocityrearLeftPid_(100, 0, 0.01),
    velocityrearRightPid_(100, 0, 0.01),
    velocityfrontLeftPid_(100, 0, 0.01),
    velocityfrontRightPid_(100, 0, 0.01) {

    steeringfrontLeftPid_.SetCmdMax(0.122173);
    steeringfrontRightPid_.SetCmdMax(0.122173);
    velocityrearLeftPid_.SetCmdMax(100);
    velocityrearRightPid_.SetCmdMax(100);
    velocityfrontLeftPid_.SetCmdMax(100);
    velocityfrontRightPid_.SetCmdMax(100);

    steeringfrontLeftPid_.SetCmdMin(-0.122173);
    steeringfrontRightPid_.SetCmdMin(-0.122173);
    velocityrearLeftPid_.SetCmdMin(-100);
    velocityrearRightPid_.SetCmdMin(-100);
    velocityfrontLeftPid_.SetCmdMin(-100);
    velocityfrontRightPid_.SetCmdMin(-100);
    
    currentCommand_.drive.speed = 0;
    currentCommand_.drive.steering_angle = 0;
}

AckermannDrivePlugin::~AckermannDrivePlugin() {

}

void AckermannDrivePlugin::commandCallback(
    const ackermann_msgs::AckermannDriveStamped::Ptr& cmd) {

    currentCommand_ = *cmd;

        // this->model_->GetJointController()->SetVelocityTarget(
        //     this->model_->GetJoint(robotNamespace_ + "::" + "rear_left_wheel_joint")->GetScopedName(),
        //     currentCommand_.drive.speed);
    
    // this->model_->GetJointController()->SetVelocityTarget(
    //     this->model_->GetJoint(robotNamespace_ + "::" + "rear_right_wheel_joint")->GetScopedName(),
        // currentCommand_.drive.speed);
    
        // this->model_->GetJointController()->SetVelocityTarget(
        //     this->model_->GetJoint(robotNamespace_ + "::" + "front_right_wheel_joint")->GetScopedName(),
        //     currentCommand_.drive.speed);
    
        // this->model_->GetJointController()->SetVelocityTarget(
        //     this->model_->GetJoint(robotNamespace_ + "::" + "front_left_wheel_joint")->GetScopedName(),
        //     currentCommand_.drive.speed);

                auto wheelSteering = fmax(-0.2967, fmin(currentCommand_.drive.steering_angle, 0.2967)); 
    
        this->model_->GetJointController()->SetPositionTarget(
            this->model_->GetJoint(robotNamespace_ + "::" + "front_right_wheel_steering_joint")->GetScopedName(), 
            currentCommand_.drive.steering_angle);
    
        this->model_->GetJointController()->SetPositionTarget(
            this->model_->GetJoint(robotNamespace_ + "::" + "front_left_wheel_steering_joint")->GetScopedName(), 
            currentCommand_.drive.steering_angle);

        this->model_->GetJointController()->SetVelocityTarget(
            this->model_->GetJoint(robotNamespace_ + "::" + "rear_right_wheel_joint")->GetScopedName(),
            currentCommand_.drive.speed);

        this->model_->GetJointController()->SetVelocityTarget(
            this->model_->GetJoint(robotNamespace_ + "::" + "rear_left_wheel_joint")->GetScopedName(),
            currentCommand_.drive.speed);

        this->model_->GetJointController()->SetVelocityTarget(
            this->model_->GetJoint(robotNamespace_ + "::" + "front_left_wheel_joint")->GetScopedName(),
            currentCommand_.drive.speed);

        this->model_->GetJointController()->SetVelocityTarget(
            this->model_->GetJoint(robotNamespace_ + "::" + "front_right_wheel_joint")->GetScopedName(),
            currentCommand_.drive.speed);
 
}

void AckermannDrivePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {

    ROS_INFO("Load");

    robotNamespace_ = sdf->Get<std::string>("robotNamespace");

    ros::NodeHandle node;

    cmdSubscriber_ = node.subscribe(
        robotNamespace_ + "/ackermann_cmd", 2, &AckermannDrivePlugin::commandCallback, this);
  
    this->model_ = model;

    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&AckermannDrivePlugin::Update, this, std::placeholders::_1));

    this->model_->GetJointController()->SetPositionPID(
            this->model_->GetJoint(robotNamespace_ + "::" + "front_right_wheel_steering_joint")->GetScopedName(), 
            steeringfrontRightPid_);

    this->model_->GetJointController()->SetPositionPID(
            this->model_->GetJoint(robotNamespace_ + "::" + "front_left_wheel_steering_joint")->GetScopedName(), 
            steeringfrontLeftPid_);

    // this->model_->GetJoint(robotNamespace_ + "::" + "rear_right_wheel_joint")->SetVelocity(0, 0.0);

    this->model_->GetJointController()->SetVelocityPID(
            this->model_->GetJoint(robotNamespace_ + "::" + "rear_right_wheel_joint")->GetScopedName(), 
            velocityrearRightPid_);

    this->model_->GetJointController()->SetVelocityPID(
            this->model_->GetJoint(robotNamespace_ + "::" + "rear_left_wheel_joint")->GetScopedName(), 
            velocityrearLeftPid_);

    this->model_->GetJointController()->SetVelocityPID(
            this->model_->GetJoint(robotNamespace_ + "::" + "front_right_wheel_joint")->GetScopedName(), 
            velocityfrontRightPid_);

    this->model_->GetJointController()->SetVelocityPID(
            this->model_->GetJoint(robotNamespace_ + "::" + "front_left_wheel_joint")->GetScopedName(), 
            velocityfrontLeftPid_);

    // this->model_->GetJointController()->Update();

    // this->model_->GetJointController()->SetPositionTarget(
    //     this->model_->GetJoint(robotNamespace_ + "::" + "front_right_wheel_steering_joint")->GetScopedName(), 
    //     0);

    // this->model_->GetJointController()->SetPositionTarget(
    //     this->model_->GetJoint(robotNamespace_ + "::" + "front_left_wheel_steering_joint")->GetScopedName(), 
    //     0);

    // this->model_->GetJointController()->SetVelocityTarget(
    //     this->model_->GetJoint(robotNamespace_ + "::" + "rear_left_wheel_joint")->GetScopedName(),
    //     0);

    // this->model_->GetJointController()->SetVelocityTarget(
    //     this->model_->GetJoint(robotNamespace_ + "::" + "rear_right_wheel_joint")->GetScopedName(),
    //     0);

    // this->model_->GetJointController()->SetVelocityTarget(
    //     this->model_->GetJoint(robotNamespace_ + "::" + "front_right_wheel_joint")->GetScopedName(),
    //     0);

    // this->model_->GetJointController()->SetVelocityTarget(
    //     this->model_->GetJoint(robotNamespace_ + "::" + "front_left_wheel_joint")->GetScopedName(),
    //     0);

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
        // Steering angle constraints. In rads
        //



        this->model_->GetJointController()->Update();
    
        ros::spinOnce();

        lastUpdate_ = info.simTime;

    }

}

} // namespace gazebo

