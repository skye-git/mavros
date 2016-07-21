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
 * @file skye_position_hil.cpp
 * Mavros plugin to send mavlink "skye_position_hil" message to the FMU.
 *
 * @author Marco Tranzatto <marco@skye.aero>
 */
#include <mavros/mavros_plugin.h>

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/LinkState.h>

namespace mavplugin {

class SkyePositionHilPlugin : public MavRosPlugin {
public:
  SkyePositionHilPlugin() :
    nh_private("~"),
    nh_public(),
    uas(nullptr)
  { };

  /**
   * Plugin initializer. Constructor should not do this.
   */
  void initialize(UAS &uas_)
  {
    uas = &uas_;

    // Subscriber to ground truth topic with NED convention
    skye_ros_ground_truth_sub = nh_public.subscribe("skye_ros/ground_truth/hull",
                                                    10,
                                                    &SkyePositionHilPlugin::ground_truth_callback,
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

  ros::Subscriber skye_ros_ground_truth_sub; /* Ground truth topic in Skye's IMU frame. */


  //-----------------------------------------------------------------------------
  void ground_truth_callback(const gazebo_msgs::LinkStateConstPtr &ground_truth){

    mavlink_message_t msg;
    float x, y, z, vx, vy, vz;

    x = static_cast<float>(ground_truth->pose.position.x);
    y = static_cast<float>(ground_truth->pose.position.y);
    z = static_cast<float>(ground_truth->pose.position.z);
    vx = static_cast<float>(ground_truth->twist.linear.x);
    vy = static_cast<float>(ground_truth->twist.linear.y);
    vz = static_cast<float>(ground_truth->twist.linear.z);

    uint64_t timestamp = static_cast<uint64_t>(ros::Time::now().toNSec() / 1000.0); // in uSec

    // Send the skye_attitude_hil message to the FMU
    mavlink_msg_position_hil_pack_chan(UAS_PACK_CHAN(uas), &msg,
                                       timestamp,
                                       x,
                                       y,
                                       z,
                                       vx,
                                       vy,
                                       vz);
    UAS_FCU(uas)->send_message(&msg);
  }

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyePositionHilPlugin, mavplugin::MavRosPlugin)

