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
 * @file skye_attitude_hil.cpp
 * Mavros plugin to send mavlink "skye_attitude_hil" message to the FMU.
 *
 * @author Marco Tranzatto <marco@skye.aero>
 */
#include <mavros/mavros_plugin.h>

#include <pluginlib/class_list_macros.h>
#include <mavros/skye_common_helpers.h>
#include <sensor_msgs/Imu.h>

//settings
const double kAttitudeHilSendingFrequency = 25.0; // [Hz]
typedef ros::WallTime TimeAttitudeHil; // Use wallclock to send attitude hil

namespace mavplugin {

class SkyeAttitudeHilPlugin : public MavRosPlugin {
//-----------------------------------------------------------------------------
public:
  SkyeAttitudeHilPlugin() :
    nh_private("~"),
    nh_public(),
    uas(nullptr),
    attitude_hil_last_time(TimeAttitudeHil::now())
  { };
//-----------------------------------------------------------------------------
  /**
   * Plugin initializer. Constructor should not do this.
   */
  void initialize(UAS &uas_)
  {
    uas = &uas_;

    // Subscriber to NOISELESS IMU topic with NED convention
    skye_ros_imu_sk_sub = nh_public.subscribe("skye_ros/sensor_msgs/imu_sk_noiseless",
                                              10,
                                              &SkyeAttitudeHilPlugin::imu_sk_callback, this);
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
  TimeAttitudeHil attitude_hil_last_time; /*< Last time attitude hil was sent to the FMU. */


//-----------------------------------------------------------------------------
  /**
   * Custom function to obtain euler angles (rotation order ZYX) in local axis.
   * This function returns roll in (-pi,pi), pitch in (-pi/2,pi/2) and yaw in (-pi,pi).
   * Use this function instead of Eigen::eulerAngles(2, 1, 0) because Eigen's version
   * returns yaw in (0,-pi).
  */
  void skye_quat_to_eu(const Eigen::Quaterniond q, float &roll, float &pitch, float &yaw) {
    roll = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
    pitch = asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
    yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  }

//-----------------------------------------------------------------------------
  /**
   * This function gets call when a new IMU message has been received
   */
  void imu_sk_callback(const sensor_msgs::ImuConstPtr &imu_sk_p) {

    // Should we send the message now or wait a little bit more?
    if (publish_now(attitude_hil_last_time, kAttitudeHilSendingFrequency)) {

      mavlink_message_t msg;
      Eigen::Quaterniond q_imu;
      float roll, pitch, yaw;
      float rollspeed, pitchspeed, yawspeed;
      float q[4];

      // Convert data to fullfill a mavlink message
      q_imu.w() = imu_sk_p->orientation.w;
      q_imu.x() = imu_sk_p->orientation.x;
      q_imu.y() = imu_sk_p->orientation.y;
      q_imu.z() = imu_sk_p->orientation.z;

      // Convert quaternion to euler angles
      skye_quat_to_eu(q_imu, roll, pitch, yaw);

      rollspeed = static_cast<float>(imu_sk_p->angular_velocity.x);
      pitchspeed = static_cast<float>(imu_sk_p->angular_velocity.y);
      yawspeed = static_cast<float>(imu_sk_p->angular_velocity.z);

      q[0] = static_cast<float>(q_imu.w());
      q[1] = static_cast<float>(q_imu.x());
      q[2] = static_cast<float>(q_imu.y());
      q[3] = static_cast<float>(q_imu.z());

      uint64_t timestamp = static_cast<uint64_t>(ros::Time::now().toNSec() / 1000.0); // in uSec

      // Send the skye_attitude_hil message to Skye
      mavlink_msg_attitude_hil_pack_chan(UAS_PACK_CHAN(uas), &msg,
                                         timestamp,
                                         roll,
                                         pitch,
                                         yaw,
                                         rollspeed,
                                         pitchspeed,
                                         yawspeed,
                                         q);
      UAS_FCU(uas)->send_message(&msg);
    }
  }


};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeAttitudeHilPlugin, mavplugin::MavRosPlugin)

