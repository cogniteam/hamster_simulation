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


AckermannDrivePlugin::AckermannDrivePlugin() {

    ros::NodeHandle node;

    cmdSubscriber_ = node.subscribe(
        "ackermann_cmd", 2, &AckermannDrivePlugin::commandCallback, this);

}

AckermannDrivePlugin::~AckermannDrivePlugin() {

}

void AckermannDrivePlugin::commandCallback(
    const ackermann_msgs::AckermannDriveStamped::Ptr& cmd) {



}

void AckermannDrivePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {

    this->model_ = model;

    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&AckermannDrivePlugin::Update, this, std::placeholders::_1));

    // this->model_->GetJoint("xact_camera::yaw_joint")->SetPosition(0, 0);
    
    // this->model_->GetJoint("xact_camera::pitch_joint")->SetPosition(0, 0);

    this->model_->GetJointController()->SetPositionPID(
            this->model_->GetJoint("xact_camera::yaw_joint")->GetScopedName(), 
            yawPid_);

    this->model_->GetJointController()->SetPositionPID(
            this->model_->GetJoint("xact_camera::pitch_joint")->GetScopedName(), 
            pitchPid_);

    this->model_->GetJointController()->SetPositionTarget(
            this->model_->GetJoint("xact_camera::yaw_joint")->GetScopedName(), 
            M_PI);

    this->model_->GetJointController()->SetPositionTarget(
            this->model_->GetJoint("xact_camera::pitch_joint")->GetScopedName(), 
            -angles::from_degrees(15));

    gzdbg << "XactCameraPlugin loaded!" << endl;
}



}

