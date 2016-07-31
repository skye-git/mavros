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
 * @file skye_gyroscopes_hil.cpp
 * Mavros plugin to send mavlink "gyroscpes_hil" message to the FMU.
 *
 * @author Marco Tranzatto <marco@skye.aero>
 */
#include <mavros/mavros_plugin.h>

#include <pluginlib/class_list_macros.h>
#include <mavros/skye_common_helpers.h>
#include <sensor_msgs/Imu.h>

//settings
const double kGyroscopesHilSendingFrequency = 25.0; // [Hz]
typedef ros::WallTime TimeGyroscopesHil; // Use wallclock to send attitude hil

namespace mavplugin {

class SkyeGyroscopesHilPlugin : public MavRosPlugin {
//-----------------------------------------------------------------------------
public:
  SkyeGyroscopesHilPlugin() :
    nh_private("~"),
    nh_public(),
    uas(nullptr),
    gyroscopes_hil_last_time(TimeGyroscopesHil::now())
  { };
//-----------------------------------------------------------------------------
  /**
   * Plugin initializer. Constructor should not do this.
   */
  void initialize(UAS &uas_)
  {
    uas = &uas_;

    // Subscriber to NOISY IMU topic with NED convention
    skye_ros_imu_sk_sub = nh_public.subscribe("skye_ros/sensor_msgs/imu_sk_noisy",
                                              10,
                                              &SkyeGyroscopesHilPlugin::imu_sk_callback, this);
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
//-----------------------------------------------------------------------------
  ros::NodeHandle nh_private;
  ros::NodeHandle nh_public;
  UAS *uas;

  ros::Subscriber skye_ros_imu_sk_sub; /* IMU topic in Skye's IMU frame. */
  TimeGyroscopesHil gyroscopes_hil_last_time; /*< Last time gyroscopes hil was sent to the FMU. */


//-----------------------------------------------------------------------------
  /**
   * This function gets call when a new IMU message has been received
   */
  void imu_sk_callback(const sensor_msgs::ImuConstPtr &imu_sk_p) {

    // Should we send the message now or wait a little bit more?
    if (publish_now(gyroscopes_hil_last_time, kGyroscopesHilSendingFrequency)) {

      mavlink_message_t msg;

      // Send the skye_attitude_hil message to Skye
      mavlink_msg_gyroscopes_hil_pack_chan(
                                         UAS_PACK_CHAN(uas), &msg,
                                         imu_sk_p->angular_velocity.x,
                                         imu_sk_p->angular_velocity.y,
                                         imu_sk_p->angular_velocity.z);

      UAS_FCU(uas)->send_message(&msg);
    }
  }


};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeGyroscopesHilPlugin, mavplugin::MavRosPlugin)

