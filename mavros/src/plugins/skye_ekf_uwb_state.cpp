/**
* @brief Read data from Skye's Firmware running on PX4 and publish them in ROS.
* @file skye_ekf_uwb_state.cpp
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
#include <tf/transform_broadcaster.h>


#include "kalman_filter/kalman_filter_msg.h"


namespace mavplugin {

  /**
 * @brief Read data from Skye's Firmware running on PX4 and publish them in ROS.
 */
  class SkyeEKFUWBStatePlugin : public MavRosPlugin {
    //-----------------------------------------------------------------------------
  public:
    SkyeEKFUWBStatePlugin() :
      nh("~"),
      uas(nullptr)
    {};

    //-----------------------------------------------------------------------------
    void initialize(UAS &uas_){
      uas = &uas_;
      state_pub = nh.advertise<kalman_filter::kalman_filter_msg>("/skye_luci/ekf_uwb_state", 5);
      seq_id = 0;
    }

    //-----------------------------------------------------------------------------
    const message_map get_rx_handlers(){
      return {
          MESSAGE_HANDLER(MAVLINK_MSG_ID_SKYE_EKF_UWB_STATE, &SkyeEKFUWBStatePlugin::handle_ekf_uwb_out),
        };
    }

    //-----------------------------------------------------------------------------
  private:
    ros::NodeHandle nh;
    UAS *uas;
    unsigned int seq_id;

    ros::Publisher state_pub; /*< EKF UWB state. */

    //-----------------------------------------------------------------------------
    void handle_ekf_uwb_out(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid){
      ROS_INFO("[Mavros]Converting EKF UWB state. \n");
      mavlink_skye_ekf_uwb_state_t state_mav;
      mavlink_msg_skye_ekf_uwb_state_decode(msg, &state_mav);

      auto state_ros_msg = boost::make_shared<kalman_filter::kalman_filter_msg>();

      Eigen::Quaternionf orientation_quat;
      Eigen::Vector3f orientation_euler;
      orientation_euler << state_mav.orientation[0], state_mav.orientation[1], state_mav.orientation[2];

      //https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Conversion
      float phi = orientation_euler[0];
      float theta = orientation_euler[1];
      float psi = orientation_euler[2];

      orientation_quat.w() = cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2);
      orientation_quat.x() = sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2);
      orientation_quat.y() = cos(phi/2)*sin(theta/2)*cos(psi/2) + sin(phi/2)*cos(theta/2)*sin(psi/2);
      orientation_quat.z() = cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*sin(theta/2)*cos(psi/2);
      ROS_INFO("[Mavros]EKF UWB state converted. \n");


      // fill
      state_ros_msg->header.seq = seq_id++;
      state_ros_msg->header.stamp = ros::Time::now();
      state_ros_msg->header.frame_id = "0";
      state_ros_msg->position.x	=             state_mav.position[0];
      state_ros_msg->position.y	=             state_mav.position[1];
      state_ros_msg->position.z	=             state_mav.position[2];
      state_ros_msg->orientation.w =          orientation_quat.w();
      state_ros_msg->orientation.x =          orientation_quat.x();
      state_ros_msg->orientation.y =          orientation_quat.y();
      state_ros_msg->orientation.z =          orientation_quat.z();
      state_ros_msg->linear_velocity.x	=     state_mav.velocity[0];
      state_ros_msg->linear_velocity.y	=     state_mav.velocity[1];
      state_ros_msg->linear_velocity.z	=     state_mav.velocity[2];
      state_ros_msg->angular_velocity.x	=     state_mav.angular_velocity[0];
      state_ros_msg->angular_velocity.y	=     state_mav.angular_velocity[1];
      state_ros_msg->angular_velocity.z	=     state_mav.angular_velocity[2];
      state_ros_msg->external_force.x	=     state_mav.external_force[0];
      state_ros_msg->external_force.y	=     state_mav.external_force[1];
      state_ros_msg->external_force.z	=     state_mav.external_force[2];
      state_ros_msg->external_torque.x	=     state_mav.external_torque[0];
      state_ros_msg->external_torque.y	=     state_mav.external_torque[1];
      state_ros_msg->external_torque.z  =     state_mav.external_torque[2];
      state_ros_msg->accelerometer_bias.x =   state_mav.accelerometer_bias[0];
      state_ros_msg->accelerometer_bias.y =   state_mav.accelerometer_bias[1];
      state_ros_msg->accelerometer_bias.z =   state_mav.accelerometer_bias[2];
      state_ros_msg->outlier_detection_mode = state_mav.outlier_rejection_mode;
      state_ros_msg->state_valid = state_mav.state_valid;

      // publish
      state_pub.publish(state_ros_msg);
      ROS_INFO("[Mavros]EKF UWB state published. \n");

      tf2_ros::TransformBroadcaster br;
      geometry_msgs::TransformStamped transform;


      transform.header.stamp = state_ros_msg->header.stamp;
      transform.header.frame_id = "InertialNED";
      transform.child_frame_id = "EKF_UWB";

      // setRotation()
      transform.transform.rotation = state_ros_msg->orientation;

      // setOrigin()
      transform.transform.translation.x = state_ros_msg->position.x;
      transform.transform.translation.y = state_ros_msg->position.y;
      transform.transform.translation.z = state_ros_msg->position.z;

      br.sendTransform(transform);
      }

  };
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeEKFUWBStatePlugin, mavplugin::MavRosPlugin)