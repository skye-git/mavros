/****************************************************************************
*
*   Copyright (C) 2016 Aerotainment Labs. All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, is not permitted unless explicitely stated by the owner
*   of the copyright.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

/**
 * @file skye_setpoint_6dof.cpp
 * Mavros plugin to send mavlink "setpoint_6dof" message to the FMU.
 *
 * @author Marco Tranzatto <marco@skye.aero>
 */
#include <mavros/mavros_plugin.h>

#include <pluginlib/class_list_macros.h>
#include <mavros/skye_common_helpers.h>
#include <geometry_msgs/Twist.h>
#include "mavros_msgs/SkyeSendStep.h"

namespace mavplugin {

//settings
const double kUser3DMouseSendingFrequency = 25.0; // [Hz]
typedef ros::WallTime Time3dMouse; // Use wallclock to send 3D mouse
typedef ros::WallTimer TimerStepCommand; // Use wallclock to send a step command
typedef ros::WallDuration DurationStepCommand; // Use wallclock to send a step command

class SkyeSetpoint6DofPlugin : public MavRosPlugin {
public:
//-----------------------------------------------------------------------------
  SkyeSetpoint6DofPlugin() :
    nh_private("~"),
    nh_public(),
    uas(nullptr),
    user_3d_mouse_last_time(Time3dMouse::now()),
    sending_step_command(false)
  { };

//-----------------------------------------------------------------------------
  /**
   * Plugin initializer. Constructor should not do this.
   */
  void initialize(UAS &uas_)
  {
    uas = &uas_;

    // Subscribe to ROS node which publishes 3D mouse inputs
    joystick_teleoperate_sub = nh_public.subscribe(
                                "/spacenav/twist",
                                10,
                                &SkyeSetpoint6DofPlugin::joystick_teleop_callback,
                                this);

    // Advertise topic where final 3D mouse inputs, sent to the FMU, are displayed
    setpoint_6dof_pub = nh_private.advertise<geometry_msgs::Twist>("setpoint_6dof",
                                                                   10);

    // Advertise service to send a step command for limited amount fo time
    send_step_srv = nh_private.advertiseService(
                              "send_step_setpoint_6dof",
                              &SkyeSetpoint6DofPlugin::send_step_setpoint_6dof,
                              this);
  }

//-----------------------------------------------------------------------------
  /**
   * This function returns message<->handler mapping
   *
   * Each entry defined by @a MESSAGE_HANDLER() macro
   */
  const message_map get_rx_handlers() {
          return { /* Rx disabled */ };
  }

private:
  ros::NodeHandle nh_private;
  ros::NodeHandle nh_public;
  UAS *uas;

  Time3dMouse user_3d_mouse_last_time; /*< Last time 3D mouse input was sent to the FMU. */
  ros::Subscriber joystick_teleoperate_sub; // keyboard teleoperator
  ros::Publisher setpoint_6dof_pub; /*< User setpoint sent to px4. */
  geometry_msgs::Twist setpoint_6dof_sent; /* 6DOF setpoint sent to the FMU .*/
  ros::ServiceServer send_step_srv; /* Service to send a step setpoint_6dof .*/
  bool sending_step_command; /* Indicates if a step command is being sent.  */
  geometry_msgs::Twist twist_step_command; /* Step command twis. */

  // Timers to send fixed duration step command
  TimerStepCommand timerSendStepCommand; /* Step command timer. */
  TimerStepCommand timerStopStepCommand; /* Timer to stop sending step command. */

//-----------------------------------------------------------------------------
  /**
   * This function sends a "setpoint_6dof" mavlink message to the FMU.
   */
  void send_user_setpoint(const geometry_msgs::Twist &twist){

    // should we send the message now or wait a little bit more?
    if (publish_now(user_3d_mouse_last_time, kUser3DMouseSendingFrequency)) {

      mavlink_message_t msg;
      float linear_x, linear_y, linear_z; // linear velocities
      float angular_x, angular_y, angular_z; // angular velocities

      linear_x = static_cast<float>(twist.linear.x);
      linear_y = static_cast<float>(twist.linear.y);
      linear_z = static_cast<float>(twist.linear.z);
      angular_x = static_cast<float>(twist.angular.x);
      angular_y = static_cast<float>(twist.angular.y);
      angular_z = static_cast<float>(twist.angular.z);

      uint64_t timestamp = static_cast<uint64_t>(Time3dMouse::now().toNSec() / 1000.0); // in uSec

      /* Send the skye_attitude_hil message to Skye. */
      mavlink_msg_setpoint_6dof_pack_chan(UAS_PACK_CHAN(uas), &msg,
                                          timestamp,
                                          linear_x,
                                          linear_y,
                                          linear_z,
                                          angular_x,
                                          angular_y,
                                          angular_z);

      UAS_FCU(uas)->send_message(&msg);

      // Publish in ROS the data sent to the FMU
      setpoint_6dof_pub.publish(twist);
    }
  }

//-----------------------------------------------------------------------------
  /**
   * This function gets call when a new joystic ROS message is received
   */
  void joystick_teleop_callback(const geometry_msgs::TwistConstPtr &twist_teleop){

    if (!sending_step_command) { //only if step service is not being using
      // adapt input to Skye's local NED frame
      geometry_msgs::Twist twist_ned = *twist_teleop;

      twist_ned.linear.y *= -1.0;
      twist_ned.linear.z *= -1.0;
      twist_ned.angular.y *= -1.0;
      twist_ned.angular.z *= -1.0;

      send_user_setpoint(twist_ned);
    }
  }

  //-----------------------------------------------------------------------------
  /**
   * This function gets call preiodically and sends a step command
   */
  void callback_send_step_command(const ros::WallTimerEvent&){

    send_user_setpoint(twist_step_command);
  }

  //-----------------------------------------------------------------------------
  /**
   * This function stops the periodically function which sends step commands
   */
  void callback_stop_step_command(const ros::WallTimerEvent&){

    timerSendStepCommand.stop();
    sending_step_command = false;
    ROS_INFO("[skye_setpoint_6dof] sent step command");
  }

 //-----------------------------------------------------------------------------
  /**
   * This function gets call when a user requested to send a step command
   */
  bool send_step_setpoint_6dof(mavros_msgs::SkyeSendStep::Request &req,
                               mavros_msgs::SkyeSendStep::Response &res) {

    //prevent 3D mouse to send input while we are sending the step command
    sending_step_command = true;

    // Save step command
    twist_step_command.linear.x = req.linear_x;
    twist_step_command.linear.y = req.linear_y;
    twist_step_command.linear.z = req.linear_z;

    twist_step_command.angular.x = req.angular_x;
    twist_step_command.angular.y = req.angular_y;
    twist_step_command.angular.z = req.angular_z;

    // Activate timers: one to send preidically step command and
    // another one to stop the first after a fixed time
    timerSendStepCommand = nh_private.createWallTimer(
                                DurationStepCommand(1.0/kUser3DMouseSendingFrequency), // sending frequency
                                &SkyeSetpoint6DofPlugin::callback_send_step_command,
                                this);

    timerStopStepCommand = nh_private.createWallTimer(
                                DurationStepCommand(req.duration.toSec()), // step duration
                                &SkyeSetpoint6DofPlugin::callback_stop_step_command,
                                this,
                                true);  // shoot only once

    ROS_INFO("[skye_setpoint_6dof] sending step command");

    res.success = true;

    return true;
  }

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeSetpoint6DofPlugin, mavplugin::MavRosPlugin)

