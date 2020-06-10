/*
 *    Filename: AckermannDrivePlugin.h
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


#ifndef GAZEBO_ACKERMANN_DRIVE_PLUGIN_H_
#define GAZEBO_ACKERMANN_DRIVE_PLUGIN_H_


#include <ros/ros.h>


#include <cmath>


#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsTypes.hh>

#include <ackermann_msgs/AckermannDriveStamped.h>


namespace gazebo{

using namespace std;

class AckermannDrivePlugin : public ModelPlugin {

public:

    AckermannDrivePlugin();

    virtual ~AckermannDrivePlugin();

public:

    virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

    void Update(const common::UpdateInfo &info);

private:

    void commandCallback(const ackermann_msgs::AckermannDriveStamped::Ptr& cmd);

    double setVelocity(double wheelRadius);

private:

    ros::Subscriber cmdSubscriber_;
    
    physics::ModelPtr model_;

    event::ConnectionPtr updateConnection_;

    std::string robotNamespace_;

    common::Time lastUpdate_ = common::Time();

    ackermann_msgs::AckermannDriveStamped currentCommand_;

    common::PID steeringfrontLeftPid_;
    common::PID steeringfrontRightPid_;

    physics::JointControllerPtr controller_;

    double wheelRadius_;
};

GZ_REGISTER_MODEL_PLUGIN(AckermannDrivePlugin)
    
}

#endif // GAZEBO_ACKERMANN_DRIVE_PLUGIN_H_

