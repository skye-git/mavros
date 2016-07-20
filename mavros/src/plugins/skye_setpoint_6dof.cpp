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

namespace mavplugin {

//settings
const double kUser3DMouseSendingFrequency = 25.0; // [Hz]
typedef ros::WallTime Time3dMouse; // Use wallclock to send 3D mouse

class SkyeSetpoint6DofPlugin : public MavRosPlugin {
public:
//-----------------------------------------------------------------------------
  SkyeSetpoint6DofPlugin() :
    nh_private("~"),
    nh_public(),
    uas(nullptr),
    user_3d_mouse_last_time(Time3dMouse::now())
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
                                &SkyeSetpoint6DofPlugin::joystick_teleop_callback, this);

    // Advertise topic where final 3D mouse inputs, sent to the FMU, are displayed
    setpoint_6dof_pub = nh_private.advertise<geometry_msgs::Twist>("setpoint_6dof", 10);
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

//-----------------------------------------------------------------------------
  /**
   * This function sends a "setpoint_6dof" mavlink message to the FMU.
   */
  void send_user_setpoint(const geometry_msgs::TwistConstPtr &ptwist){

    // should we send the message now or wait a little bit more?
    if (publish_now(user_3d_mouse_last_time, kUser3DMouseSendingFrequency)) {

      mavlink_message_t msg;
      float linear_x, linear_y, linear_z; // linear velocities
      float angular_x, angular_y, angular_z; // angular velocities

      // adapt input to Skye's local NED frame
      linear_x = static_cast<float>(ptwist->linear.x);
      linear_y = -static_cast<float>(ptwist->linear.y);
      linear_z = -static_cast<float>(ptwist->linear.z);
      angular_x = static_cast<float>(ptwist->angular.x);
      angular_y = -static_cast<float>(ptwist->angular.y);
      angular_z = -static_cast<float>(ptwist->angular.z);

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

      // Publish in ROS the actual data sent to the FMU
      setpoint_6dof_sent.linear.x = linear_x;
      setpoint_6dof_sent.linear.y = linear_y;
      setpoint_6dof_sent.linear.z = linear_z;
      setpoint_6dof_sent.angular.x = angular_x;
      setpoint_6dof_sent.angular.y = angular_y;
      setpoint_6dof_sent.angular.z = angular_z;
      setpoint_6dof_pub.publish(*ptwist);
    }
  }

//-----------------------------------------------------------------------------
  /**
   * This function gets call when a new joystic ROS message is received
   */
  void joystick_teleop_callback(const geometry_msgs::TwistConstPtr &twist_teleop){

//    if(!sending_step_command) //only if step service is not being using
//      send_user_setpoint(twist_teleop);
    send_user_setpoint(twist_teleop);
  }

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeSetpoint6DofPlugin, mavplugin::MavRosPlugin)

