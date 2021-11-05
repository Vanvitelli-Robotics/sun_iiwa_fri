/**

The following license terms and conditions apply, unless a redistribution
agreement or other license is obtained by KUKA Roboter GmbH, Augsburg, Germany.

SCOPE

The software �KUKA Sunrise.Connectivity FRI Client SDK� is targeted to work in
conjunction with the �KUKA Sunrise.Connectivity FastRobotInterface� toolkit.
In the following, the term �software� refers to all material directly
belonging to the provided SDK �Software development kit�, particularly source
code, libraries, binaries, manuals and technical documentation.

COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2016
KUKA Roboter GmbH
Augsburg, Germany

LICENSE

Redistribution and use of the software in source and binary forms, with or
without modification, are permitted provided that the following conditions are
met:
a) The software is used in conjunction with KUKA products only.
b) Redistributions of source code must retain the above copyright notice, this
list of conditions and the disclaimer.
c) Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the disclaimer in the documentation and/or other
materials provided with the distribution. Altered source code of the
redistribution must be made available upon request with the distribution.
d) Modification and contributions to the original software provided by KUKA
must be clearly marked and the authorship must be stated.
e) Neither the name of KUKA nor the trademarks owned by KUKA may be used to
endorse or promote products derived from this software without specific prior
written permission.

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS," without warranty of
any kind, including without limitation the warranties of merchantability,
fitness for a particular purpose and non-infringement.
KUKA makes no warranty that the Software is free of defects or is suitable for
any particular purpose. In no event shall KUKA be responsible for loss or
damages arising from the installation or use of the Software, including but
not limited to any indirect, punitive, special, incidental or consequential
damages of any character including, without limitation, damages for loss of
goodwill, work stoppage, computer failure or malfunction, or any and all other
commercial damages or losses.
The entire risk to the quality and performance of the Software is not borne by
KUKA. Should the Software prove defective, KUKA is not liable for the entire
cost of any service and repair.



\file
\version {1.11}
*/
#include "sun_iiwa_fri/IIWAFRIClient.h"
#include "friLBRState.h"
#include <cstdio>
#include <cstring>

#include "sensor_msgs/JointState.h"
#include "sun_iiwa_fri/Monitoring.h"

using namespace KUKA::FRI;
using namespace sun::iiwa::fri;

//******************************************************************************
RosFriClient::RosFriClient(const ros::NodeHandle &nh) : nh_(nh) {
  nh_.setCallbackQueue(&cb_queue_);
  init();
}

//******************************************************************************
RosFriClient::~RosFriClient() {}

//******************************************************************************

void RosFriClient::init() {
  updateParameters();
  initPubsSubs();
}

void RosFriClient::updateParameters() {

  joint_cmd_topic_ = "iiwa/command/joint_position";
  joint_state_cmd_topic_ = "iiwa/command/joint_state";
  joint_state_topic_ = "iiwa/state/joint_state";
  monitoring_topic_ = "iiwa/state/monitoring";
  joint_names_ = {"A1", "A2", "A3", "A4", "A5", "A6", "A7"};
  joint_state_frame_id_ = "iiwa";
}

void RosFriClient::initPubsSubs() {
  joint_state_pub_ =
      nh_.advertise<sensor_msgs::JointState>(joint_state_topic_, 1);
  monitoring_pub_ =
      nh_.advertise<sun_iiwa_fri::Monitoring>(monitoring_topic_, 1);

  joint_cmd_sub_ =
      nh_.subscribe(joint_cmd_topic_, 1, &RosFriClient::joint_cmd_cb, this);

  joint_state_cmd_sub_ =
      nh_.subscribe(joint_state_cmd_topic_, 1, &RosFriClient::joint_state_cmd_cb, this);
}

void RosFriClient::spinOnce(const ros::WallDuration &timeout) {
  cb_queue_.callAvailable(timeout);
}

void RosFriClient::publishAll() {
  pubJointState();
  pubMonitoring();
}

void RosFriClient::pubMonitoring() {

  sun_iiwa_fri::MonitoringPtr msg(new sun_iiwa_fri::Monitoring);

  msg->header.stamp = ros::Time::now();

  msg->client_command_mode = robotState().getClientCommandMode();

  msg->commanded_joint_position.insert(
      msg->commanded_joint_position.end(),
      robotState().getCommandedJointPosition(),
      &robotState().getCommandedJointPosition()[robotState().NUMBER_OF_JOINTS]);

  // TODO robotState().getCommandedTorque();

  if (robotState().getIpoJointPosition() != nullptr) {
    msg->ipo_joint_position.insert(
        msg->ipo_joint_position.end(), robotState().getIpoJointPosition(),
        &robotState().getIpoJointPosition()[robotState().NUMBER_OF_JOINTS]);
  }

  msg->connection_quality = robotState().getConnectionQuality();
  msg->control_mode = robotState().getControlMode();
  msg->drive_state = robotState().getDriveState();
  msg->operation_mode = robotState().getOperationMode();
  msg->overlay_type = robotState().getOverlayType();
  msg->safety_state = robotState().getSafetyState();
  msg->sample_time = robotState().getSampleTime();
  msg->session_state = robotState().getSessionState();
  msg->cabinet_timestamp = ros::Time(robotState().getTimestampSec(),
                                     robotState().getTimestampNanoSec());
  msg->tracking_performance = robotState().getTrackingPerformance();

  monitoring_pub_.publish(msg);
}

void RosFriClient::pubJointState() {
  sensor_msgs::JointState::Ptr msg(new sensor_msgs::JointState);
  msg->header.stamp = ros::Time::now();
  msg->header.frame_id = joint_state_frame_id_;
  msg->name = joint_names_;

  msg->position.insert(
      msg->position.end(), robotState().getMeasuredJointPosition(),
      &robotState().getMeasuredJointPosition()[robotState().NUMBER_OF_JOINTS]);

  msg->effort.insert(
      msg->effort.end(), robotState().getMeasuredTorque(),
      &robotState().getMeasuredTorque()[robotState().NUMBER_OF_JOINTS]);

  // TODO
  // robotState().getExternalTorque();
  // robotState().getMeasuredTorque();

  joint_state_pub_.publish(msg);
}

void RosFriClient::joint_cmd_cb(const sun_iiwa_fri::IIWACommandConstPtr &msg) {
  last_cmd_ = msg;
}

void RosFriClient::joint_state_cmd_cb(
    const sensor_msgs::JointStateConstPtr &msg) {
  sun_iiwa_fri::IIWACommandPtr last_cmd(new sun_iiwa_fri::IIWACommand);
  last_cmd->header = msg->header;
  last_cmd->joint_position = msg->position;
  last_cmd_ = last_cmd;
}

void RosFriClient::initializeLastCmd() {
  sun_iiwa_fri::IIWACommandPtr last_cmd(new sun_iiwa_fri::IIWACommand);
  last_cmd->header.stamp = ros::Time::now();
  last_cmd->header.frame_id = joint_state_frame_id_;
  last_cmd->joint_position.insert(
      last_cmd->joint_position.end(), robotState().getMeasuredJointPosition(),
      &robotState().getMeasuredJointPosition()[robotState().NUMBER_OF_JOINTS]);
  last_cmd_ = last_cmd;
}

//******************************************************************************
void RosFriClient::onStateChange(ESessionState oldState,
                                 ESessionState newState) {
  LBRClient::onStateChange(oldState, newState);
  // react on state change events
  switch (newState) {
  case IDLE: {
    printf("FRI STATE CHANGED TO IDLE\n");
    break;
  }
  case MONITORING_WAIT: {
    printf("FRI STATE CHANGED TO MONITORING_WAIT\n");
    break;
  }
  case MONITORING_READY: {
    printf("FRI STATE CHANGED TO MONITORING_READY\n");
    break;
  }
  case COMMANDING_WAIT: {
    printf("FRI STATE CHANGED TO COMMANDING_WAIT\n");
    break;
  }
  case COMMANDING_ACTIVE: {
    printf("FRI STATE CHANGED TO COMMANDING_ACTIVE\n");
    break;
  }
  default: {
    printf("FRI STATE error: %d\n", newState);
    break;
  }
  }
}

//******************************************************************************
void RosFriClient::monitor() {
  LBRClient::monitor();

  /***************************************************************************/
  /*                                                                         */
  /*   Place user Client Code here                                           */
  /*                                                                         */
  /***************************************************************************/

  publishAll();
}

//******************************************************************************
void RosFriClient::waitForCommand() {
  // In waitForCommand(), the joint values have to be mirrored. Which is done,
  // by calling the base method.
  LBRClient::waitForCommand();

  /***************************************************************************/
  /*                                                                         */
  /*   Place user Client Code here                                           */
  /*                                                                         */
  /***************************************************************************/

  initializeLastCmd();

  publishAll();
}

//******************************************************************************
void RosFriClient::command() {
  /***************************************************************************/
  /*                                                                         */
  /*   Place user Client Code here                                           */
  /*                                                                         */
  /***************************************************************************/

  // In command(), the joint angle values have to be set.
  // robotCommand().setJointPosition( newJointValues );
  // calculate new offset
  spinOnce();

  if (!last_cmd_) {
    initializeLastCmd();
  }

  robotCommand().setJointPosition(last_cmd_->joint_position.data());

  publishAll();
}
