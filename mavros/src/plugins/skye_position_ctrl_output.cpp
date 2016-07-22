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
 * @file skye_position_ctrl_output.cpp
 * Mavros plugin to read mavlink "position_ctrl_output" message from the FMU.
 *
 * @author Marco Tranzatto <marco@skye.aero>
 */
#include <mavros/mavros_plugin.h>

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Vector3.h>

namespace mavplugin {

class SkyePositionCtrlOutputPlugin : public MavRosPlugin {
public:
//-----------------------------------------------------------------------------
  SkyePositionCtrlOutputPlugin() :
    nh_private("~"),
    nh_public(),
    uas(nullptr)
  { };

//-----------------------------------------------------------------------------
  /**
   * Plugin initializer. Constructor should not do this.
   */
  void initialize(UAS &uas_)
  {
    uas = &uas_;

    // Publisher attitude controller output
    position_ctrl_output_pub = nh_private.advertise<geometry_msgs::Vector3>(
                               "skye_px4/position_ctrl_output",
                               10);

    // Check if we should apply the requested torque to the Hull.
    nh_private.param<bool>("use_allocator_output", use_allocator_output, false);

    if (!use_allocator_output) {
      // Use public node handle, most likely body_wrench plugin is listening there
      body_force_requested_pub = nh_public.advertise<geometry_msgs::Vector3>(
                                  "skye_gz/hull/force_desired",
                                  10);
    }

  }

//-----------------------------------------------------------------------------
  /**
   * This function returns message<->handler mapping
   *
   * Each entry defined by @a MESSAGE_HANDLER() macro
   */
  const message_map get_rx_handlers() {
    return {
      MESSAGE_HANDLER(MAVLINK_MSG_ID_POSITION_CTRL_OUTPUT,
                      &SkyePositionCtrlOutputPlugin::handle_position_ctrl_output)
    };
  }

private:
//-----------------------------------------------------------------------------
  ros::NodeHandle nh_private;
  ros::NodeHandle nh_public;
  UAS *uas;

  ros::Publisher position_ctrl_output_pub; /*< publisher of position controller output. */
  geometry_msgs::Vector3 position_ctrl_output_msg; /*< ROS message with position controller output*/
  bool use_allocator_output; /*  indicates if we should apply the requested torque to the hull*/
  ros::Publisher body_force_requested_pub; /*< publisher of body force to be applied. */

//-----------------------------------------------------------------------------
  /**
   * This function gets call when a new position_ctrl_output mavlink message is received
   */
  void handle_position_ctrl_output(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {

    mavlink_position_ctrl_output_t position_ctrl_output;
    mavlink_msg_position_ctrl_output_decode(msg, &position_ctrl_output);

    // Fill
    position_ctrl_output_msg.x = position_ctrl_output.F_x;
    position_ctrl_output_msg.y = position_ctrl_output.F_y;
    position_ctrl_output_msg.z = position_ctrl_output.F_z;

    // Publish in ROS
    position_ctrl_output_pub.publish(position_ctrl_output_msg);

    // Apply the force to Gazebo, if we are not using allocator output
    if (!use_allocator_output) {
      body_force_requested_pub.publish(position_ctrl_output_msg);
    }

  }

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyePositionCtrlOutputPlugin, mavplugin::MavRosPlugin)

