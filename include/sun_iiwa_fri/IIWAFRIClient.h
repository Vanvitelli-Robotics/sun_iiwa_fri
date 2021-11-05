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
#ifndef SUN_IIWA_FRI_CLIENT_H
#define SUN_IIWA_FRI_CLIENT_H

#include "friLBRClient.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "sun_iiwa_fri/IIWACommand.h"
#include "sensor_msgs/JointState.h"

namespace sun::iiwa::fri {

/**
 * \brief IIWA CLIENT
 */
class RosFriClient : public KUKA::FRI::LBRClient {

protected:
  ros::NodeHandle nh_;
  ros::CallbackQueue cb_queue_;

  ros::Publisher joint_state_pub_;
  std::string joint_state_topic_;

  ros::Publisher monitoring_pub_;
  std::string monitoring_topic_;

  ros::Subscriber joint_cmd_sub_;
  std::string joint_cmd_topic_;
  sun_iiwa_fri::IIWACommandConstPtr last_cmd_;

  ros::Subscriber joint_state_cmd_sub_;
  std::string joint_state_cmd_topic_;

  std::vector<std::string> joint_names_;
  std::string joint_state_frame_id_;

public:
  /**
   * \brief Constructor.
   */
  RosFriClient(const ros::NodeHandle &nh);

  /**
   * \brief Destructor.
   */
  ~RosFriClient();

  void init();

  void updateParameters();

  void initPubsSubs();

  void spinOnce(const ros::WallDuration &timeout = ros::WallDuration(0.0));

  void publishAll();

  void pubMonitoring();

  void pubJointState();

  void joint_cmd_cb(const sun_iiwa_fri::IIWACommandConstPtr &msg);

  void joint_state_cmd_cb(const sensor_msgs::JointStateConstPtr& msg);

  void initializeLastCmd();

  /**
   * \brief Callback for FRI state changes.
   *
   * @param oldState
   * @param newState
   */
  virtual void onStateChange(KUKA::FRI::ESessionState oldState,
                             KUKA::FRI::ESessionState newState);

  /**
   * \brief Callback for the FRI session states 'Monitoring Wait' and
   * 'Monitoring Ready'.
   *
   * If you do not want to change the default-behavior, you do not have to
   * implement this method.
   */
  virtual void monitor();

  /**
   * \brief Callback for the FRI session state 'Commanding Wait'.
   *
   * If you do not want to change the default-behavior, you do not have to
   * implement this method.
   */
  virtual void waitForCommand();

  /**
   * \brief Callback for the FRI state 'Commanding Active'.
   *
   * If you do not want to change the default-behavior, you do not have to
   * implement this method.
   */
  virtual void command();
};

} // namespace sun::iiwa::fri

#endif // SUN_IIWA_FRI_CLIENT_H
