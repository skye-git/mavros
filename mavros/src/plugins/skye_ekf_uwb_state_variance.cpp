/**
* @brief Read data from Skye's Firmware running on PX4 and publish them in ROS.
* @file skye_ekf_uwb_state_variance.cpp
* @author Vincent Mai <maiv@ethz.ch>
*
* @addtogroup plugin
* @{
*/
#include <cmath>
#include <eigen3/Eigen/Core>

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_srvs/Empty.h>

#include "kalman_filter/kalman_filter_variance_msg.h"


namespace mavplugin {

  /**
 * @brief Read data from Skye's Firmware running on PX4 and publish them in ROS.
 */
  class SkyeEKFUWBStateVariancePlugin : public MavRosPlugin {
    //-----------------------------------------------------------------------------
  public:
    SkyeEKFUWBStateVariancePlugin() :
      nh("~"),
      uas(nullptr)
    {};

    //-----------------------------------------------------------------------------
    void initialize(UAS &uas_){
      uas = &uas_;
      state_var_pub = nh.advertise<kalman_filter::kalman_filter_variance_msg>("/skye_luci/ekf_uwb_state_variance", 5);
      seq_id = 0;
    }

    //-----------------------------------------------------------------------------
    const message_map get_rx_handlers(){
      return {
          MESSAGE_HANDLER(MAVLINK_MSG_ID_SKYE_EKF_UWB_STATE_VARIANCE, &SkyeEKFUWBStateVariancePlugin::handle_ekf_uwb_var_out),
        };
    }

    //-----------------------------------------------------------------------------
  private:
    ros::NodeHandle nh;
    UAS *uas;
    unsigned int seq_id;

    ros::Publisher state_var_pub; /*< EKF UWB state. */

    //-----------------------------------------------------------------------------
    void handle_ekf_uwb_var_out(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid){

      mavlink_skye_ekf_uwb_state_variance_t state_var_mav;
      mavlink_msg_skye_ekf_uwb_state_variance_decode(msg, &state_var_mav);

      auto state_var_ros_msg = boost::make_shared<kalman_filter::kalman_filter_variance_msg>();

      /*Eigen::Quaternionf = orientation_quat;
      Eigen::Vector3f orientation_euler;
      orientation_euler_ << state_mav.orientation[0] << state_mav.orientation[1] << state_mav.orientation[2];
      //https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Conversion
      float phi = orientation_euler[0];
      float theta = orientation_euler[1];
      float psi = orientation_euler[2];
      orientation_quat.w = cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2);
      orientation_quat.x = sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2);
      orientation_quat.y = cos(phi/2)*sin(theta/2)*cos(psi/2) + sin(phi/2)*cos(theta/2)*sin(psi/2);
      orientation_quat.z = cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*sin(theta/2)*cos(psi/2);*/

      // fill
      state_var_ros_msg->header.seq = seq_id++;
      state_var_ros_msg->header.stamp = ros::Time::now();
      state_var_ros_msg->header.frame_id = "0";
      state_var_ros_msg->position_variance.x	=             state_var_mav.position[0];
      state_var_ros_msg->position_variance.y	=             state_var_mav.position[1];
      state_var_ros_msg->position_variance.z	=             state_var_mav.position[2];
      state_var_ros_msg->orientation_variance.x =          state_var_mav.orientation[0];   //Delta variance
      state_var_ros_msg->orientation_variance.y =          state_var_mav.orientation[1];
      state_var_ros_msg->orientation_variance.z =          state_var_mav.orientation[2];
      state_var_ros_msg->linear_velocity_variance.x	=     state_var_mav.velocity[0];
      state_var_ros_msg->linear_velocity_variance.y	=     state_var_mav.velocity[1];
      state_var_ros_msg->linear_velocity_variance.z	=     state_var_mav.velocity[2];
      state_var_ros_msg->angular_velocity_variance.x	=     state_var_mav.angular_velocity[0];
      state_var_ros_msg->angular_velocity_variance.y	=     state_var_mav.angular_velocity[1];
      state_var_ros_msg->angular_velocity_variance.z	=     state_var_mav.angular_velocity[2];
      state_var_ros_msg->external_force_variance.x	=     state_var_mav.external_force[0];
      state_var_ros_msg->external_force_variance.y	=     state_var_mav.external_force[1];
      state_var_ros_msg->external_force_variance.z	=     state_var_mav.external_force[2];
      state_var_ros_msg->external_torque_variance.x	=     state_var_mav.external_torque[0];
      state_var_ros_msg->external_torque_variance.y	=     state_var_mav.external_torque[1];
      state_var_ros_msg->external_torque_variance.z  =     state_var_mav.external_torque[2];
      state_var_ros_msg->accelerometer_bias_variance.x =   state_var_mav.accelerometer_bias[0];
      state_var_ros_msg->accelerometer_bias_variance.y =   state_var_mav.accelerometer_bias[1];
      state_var_ros_msg->accelerometer_bias_variance.z =   state_var_mav.accelerometer_bias[2];

      // publish
      state_var_pub.publish(state_var_ros_msg);

    }

  };
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeEKFUWBStateVariancePlugin, mavplugin::MavRosPlugin)