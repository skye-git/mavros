/**
 * @brief Send data from ROS to Skye's firmware running in PX4 Stack.
 * @file skye_talker.cpp
 * @author Marco Tranzatto <marco@aerotainment.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <cmath>

#include <Eigen/Geometry>
#include <ros/console.h>
#include <boost/any.hpp>
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/Imu.h>
#include <mavros_msgs/SkyeCMode.h>
#include <mavros_msgs/ParamSet.h>
#include <gazebo_msgs/LinkState.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/SetSkyePosCtrlParms.h>
#include <std_srvs/Empty.h>

#include "mavros/skye_base.h"

//debugging
static int count = 0;
#define MAX_COUNT 50
//debugging

namespace mavplugin {

  /**
   * @brief Send data from skye_ros pkg to Skye's firmware running on the PX4 stack.
   */
class SkyeTalkerPlugin : public MavRosPlugin {

public:
  SkyeTalkerPlugin() :
    nh("~"),
    uas(nullptr),
    received_first_heartbit(false)
  {}
//-----------------------------------------------------------------------------
void set_parameter(std::string param_name, int param_value){

  mavlink_message_t msg;
  float *float_var = (float*)(&param_value);//todo find a better solution to this workaround
  char c_buffer[16]; // 16 is the maximum length of mavlink param name
  strcpy(c_buffer, param_name.c_str());

  mavlink_msg_param_set_pack_chan(UAS_PACK_CHAN(uas), &msg,
                                  uas->get_tgt_system(),
                                  (uint8_t)MAV_COMP_ID_ALL,
                                  c_buffer,
                                  *float_var,
                                  (uint8_t)MAV_PARAM_TYPE_INT32);
  UAS_FCU(uas)->send_message(&msg);
}

//-----------------------------------------------------------------------------
void set_parameter(std::string param_name, float param_value){

  mavlink_message_t msg;
  char c_buffer[16]; // 16 is the maximum length of mavlink param name
  strcpy(c_buffer, param_name.c_str());

  mavlink_msg_param_set_pack_chan(UAS_PACK_CHAN(uas), &msg,
                                  uas->get_tgt_system(),
                                  (uint8_t)MAV_COMP_ID_ALL,
                                  c_buffer,
                                  param_value,
                                  MAV_PARAM_TYPE_REAL32);
  UAS_FCU(uas)->send_message(&msg);
}

//-----------------------------------------------------------------------------
void set_hil_mode(bool hil_on){
  if(hil_on){
      set_parameter("SKYE_HIL_MODE", 1);
      ROS_INFO_STREAM("[skye_talker]: set HIL mode to 1");
    }
  else{
      set_parameter("SKYE_HIL_MODE", 0);
      ROS_INFO_STREAM("[skye_talker]: set HIL mode to 0");
    }

}

//-----------------------------------------------------------------------------
void initialize(UAS &uas_){
  uas = &uas_;

  skye_ros_imu_sk_sub = nh.subscribe(skye_base.getImuTopicName(),
                                     10,
                                     &SkyeTalkerPlugin::imu_sk_callback, this);

  set_c_mod_pos_srv = nh.advertiseService("/skye_mr/set_pos_c_mod", &SkyeTalkerPlugin::set_pos_c_mod, this);
  set_c_mod_att_srv = nh.advertiseService("/skye_mr/set_att_c_mod", &SkyeTalkerPlugin::set_att_c_mod, this);

  set_skye_param_srv = nh.advertiseService("/skye_mr/set_param", &SkyeTalkerPlugin::set_skye_param, this);

  set_skye_pos_ctrl_srv = nh.advertiseService("/skye_mr/set_param/pos_ctrl",
                                              &SkyeTalkerPlugin::set_skye_pos_ctrl_params, this);

  send_step_x_srv = nh.advertiseService("/skye_mr/send_step_x",
                                        &SkyeTalkerPlugin::send_step_x, this);

  skye_ros_ground_truth_sub = nh.subscribe("/skye_ros/ground_truth/hull",
                                           10,
                                           &SkyeTalkerPlugin::ground_truth_callback, this);

  joystick_teleoperate_sub = nh.subscribe("/spacenav/twist",
                                          10,
                                          &SkyeTalkerPlugin::joystick_teleop_callback, this);
  user_setpoint_pub = nh.advertise<geometry_msgs::Twist>("/skye_mr/user_setpoint", 10);

  sending_step_command = false;
}

//-----------------------------------------------------------------------------
~SkyeTalkerPlugin(){
  // disable HIL mode before exiting
  set_hil_mode(false);
}

//-----------------------------------------------------------------------------
const message_map get_rx_handlers(){
  return {
    MESSAGE_HANDLER(MAVLINK_MSG_ID_HEARTBEAT, &SkyeTalkerPlugin::handle_heartbeat)
  };
}

//-----------------------------------------------------------------------------
private:
  ros::NodeHandle nh;
  UAS *uas;
  ros::Subscriber skye_ros_imu_sk_sub; // IMU topic in Skye's IMU frame
  ros::Subscriber skye_ros_ground_truth_sub; // Ground truth topic
  ros::Subscriber joystick_teleoperate_sub; // keyboard teleoperator
  skye_base::SkyeBase skye_base; // base class to interface with simulation of Skye in Gazebo
  ros::ServiceServer set_c_mod_pos_srv; // service to set C_MOD_POS parameter in px4
  ros::ServiceServer set_c_mod_att_srv; // service to set C_MOD_ATT parameter in px4
  ros::ServiceServer set_skye_param_srv; // service to set a parameter in px4
  ros::ServiceServer set_skye_pos_ctrl_srv; // service to set a position controller parameters in px4
  ros::ServiceServer send_step_x_srv; // service to send a step velocity command on the x axis for 2 seconds
  ros::Publisher user_setpoint_pub; /*< user setpoint sent to px4. */
  bool received_first_heartbit;
  bool sending_step_command;

//-----------------------------------------------------------------------------
/* Custom function to obtain euler angles (rotation order ZYX) in local axis.
 * This function returns roll in (-pi,pi), pitch in (-pi/2,pi/2) and yaw in (-pi,pi).
 * Use this function instead of Eigen::eulerAngles(2, 1, 0) because Eigen's version
 * returns yaw in (0,-pi).
*/
void skye_quat_to_eu(const Eigen::Quaterniond q, float &roll, float &pitch, float &yaw){

//  Eigen::Matrix3d m(q);
//  roll = atan2(m(2,1), m(2,2));
//  pitch = atan2(-m(2,0), sqrt(m(2,1) * m(2,1) + m(2,2) * m(2,2)));
//  yaw = atan2(m(1,0), m(0,0));

  roll = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
  pitch = asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
  yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

//-----------------------------------------------------------------------------
void imu_sk_callback(const sensor_msgs::ImuConstPtr &imu_sk_p){

  mavlink_message_t msg;
  Eigen::Quaterniond q_imu;
  float roll, pitch, yaw;
  float rollspeed, pitchspeed, yawspeed;
  float q[4];

  /* Convert data to fullfill a mavlink message. */
  q_imu.w() = imu_sk_p->orientation.w;
  q_imu.x() = imu_sk_p->orientation.x;
  q_imu.y() = imu_sk_p->orientation.y;
  q_imu.z() = imu_sk_p->orientation.z;

//  Eigen::Vector3d euler_angles = q_imu.matrix().eulerAngles(2, 1, 0); // Tait-Bryan, NED
// //The above fcn return a yaw angle in [0,pi], we need yaw in [-pi,pi]
//  roll = static_cast<float>(euler_angles[2]);
//  pitch = static_cast<float>(euler_angles[1]);
//  yaw = static_cast<float>(euler_angles[0]);

  skye_quat_to_eu(q_imu, roll, pitch, yaw);

  //TODO delete me
//  if(count >= MAX_COUNT){
//    ROS_INFO("yaw: %f, pitch: %f, roll: %f", yaw, pitch, roll);
//    count = 0;
//  }
//  else
//    count++;
  //TODO end delete me

  rollspeed = static_cast<float>(imu_sk_p->angular_velocity.x);
  pitchspeed = static_cast<float>(imu_sk_p->angular_velocity.y);
  yawspeed = static_cast<float>(imu_sk_p->angular_velocity.z);

  q[0] = static_cast<float>(q_imu.w());
  q[1] = static_cast<float>(q_imu.x());
  q[2] = static_cast<float>(q_imu.y());
  q[3] = static_cast<float>(q_imu.z());

  uint64_t timestamp = static_cast<uint64_t>(ros::Time::now().toNSec() / 1000.0); // in uSec

  /* Send the skye_attitude_hil message to Skye. */
  mavlink_msg_skye_attitude_hil_pack_chan(UAS_PACK_CHAN(uas), &msg,
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

//-----------------------------------------------------------------------------
void send_user_setpoint(const geometry_msgs::TwistConstPtr &ptwist){
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

  uint64_t timestamp = static_cast<uint64_t>(ros::Time::now().toNSec() / 1000.0); // in uSec

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

  user_setpoint_pub.publish(*ptwist);
}

//-----------------------------------------------------------------------------
void handle_heartbeat(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid){
  // the first time we receive the hearbit msg we enable HIL mode
  if(!received_first_heartbit){
      // enable HIL mode before exiting
      set_hil_mode(true);
      received_first_heartbit = true;
    }
}

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

  //ROS_INFO("\n Pos: %f, %f, %f \n Vel: %f, %f, %f", x, y, z, vx, vy, vz);

  uint64_t timestamp = static_cast<uint64_t>(ros::Time::now().toNSec() / 1000.0); // in uSec

  /* Send the skye_attitude_hil message to Skye. */
  mavlink_msg_skye_position_hil_pack_chan(UAS_PACK_CHAN(uas), &msg,
                                          timestamp,
                                          x,
                                          y,
                                          z,
                                          vx,
                                          vy,
                                          vz);
  UAS_FCU(uas)->send_message(&msg);
}

//-----------------------------------------------------------------------------
void joystick_teleop_callback(const geometry_msgs::TwistConstPtr &twist_teleop){

  if(!sending_step_command) //only if step service is not being using
    send_user_setpoint(twist_teleop);
}

//-----------------------------------------------------------------------------
bool set_att_c_mod(mavros_msgs::SkyeCMode::Request &req,
                  mavros_msgs::SkyeCMode::Response &res){

  bool send_new_mode = false;
  std::string str_msg = "";

  // check mode is valid in according to SKYE_ATT_C_MOD in skye.h
  switch(req.mode){
    case SKYE_ATT_C_MOD_MANUAL:
      str_msg = "[skye_talker]: ATT_C_MOD switched to SKYE_ATT_C_MOD_MANUAL";
      send_new_mode = true;
      break;

    case SKYE_ATT_C_MOD_5_DOF:
      str_msg = "[skye_talker]: ATT_C_MOD switched to SKYE_ATT_C_MOD_5_DOF";
      send_new_mode = true;
      break;

    case SKYE_ATT_C_MOD_6_DOF:
      str_msg = "[skye_talker]: ATT_C_MOD switched to SKYE_ATT_C_MOD_6_DOF";
      send_new_mode = true;
      break;

    case SKYE_ATT_C_MOD_6_DOFI:
      str_msg = "[skye_talker]: ATT_C_MOD switched to SKYE_ATT_C_MOD_6_DOFI";
      send_new_mode = true;
      break;

    case SKYE_ATT_C_MOD_GEOM:
      str_msg = "[skye_talker]: ATT_C_MOD switched to SKYE_ATT_C_MOD_GEOM";
      send_new_mode = true;
      break;

    case SKYE_ATT_C_MOD_MAX:
      str_msg = "[skye_talker]: ATT_C_MOD switched to SKYE_ATT_C_MOD_MAX";
      send_new_mode = true;
      break;

    default:
      str_msg = "[skye_talker]: specified mode in ATT_C_MOD request is not valid";
      send_new_mode = false;
      break;
  }

  if(send_new_mode){
    set_parameter("ATT_C_MOD", req.mode);
  }

  ROS_INFO_STREAM(str_msg);

  res.success = send_new_mode;

  return true;
}

//-----------------------------------------------------------------------------
bool set_pos_c_mod(mavros_msgs::SkyeCMode::Request &req,
                  mavros_msgs::SkyeCMode::Response &res){

  bool send_new_mode = false;
  std::string str_msg = "";

  // check mode is valid in according to SKYE_POS_C_MOD in skye.h
  switch(req.mode){
    case SKYE_POS_C_MOD_MANUAL:
      str_msg = "[skye_talker]: POS_C_MOD switched to SKYE_POS_C_MOD_MANUAL";
      send_new_mode = true;
      break;

    case SKYE_POS_C_MOD_CASCADE_PID:
      str_msg = "[skye_talker]: POS_C_MOD switched to SKYE_POS_C_MOD_CASCADE_PID";
      send_new_mode = true;
      break;

    default:
      str_msg = "[skye_talker]: specified mode in POS_C_MOD request is not valid";
      send_new_mode = false;
      break;
  }

  if(send_new_mode){
    set_parameter("POS_C_MOD", req.mode);
  }

  ROS_INFO_STREAM(str_msg);

  res.success = send_new_mode;

  return true;
}

//-----------------------------------------------------------------------------
bool set_skye_param(mavros_msgs::ParamSet::Request &req,
                    mavros_msgs::ParamSet::Response &res){


  if(req.value.integer != 0){
    int param = req.value.integer;
    set_parameter(req.param_id, param);
  }
  else{
    float param = req.value.real;
    set_parameter(req.param_id, param);
  }

  res.success = true;

  return true;
}

//-----------------------------------------------------------------------------
bool set_skye_pos_ctrl_params(mavros_msgs::SetSkyePosCtrlParms::Request &req,
                              mavros_msgs::SetSkyePosCtrlParms::Response &res){


  float outer_p, outer_i, inner_p;

  outer_p = req.outer_p;
  outer_i = req.outer_i;
  inner_p = req.inner_p;

  set_parameter("POS_OUTER_P", outer_p);
  set_parameter("POS_OUTER_I", outer_i);
  set_parameter("POS_INNER_P", inner_p);

  res.success = true;

  ROS_INFO("[skye_talker] new pos ctrl parameters: %f, %f, %f", outer_p, outer_i, inner_p);

  return true;
}

//-----------------------------------------------------------------------------
bool send_step_x(std_srvs::Empty::Request &req,
                 std_srvs::Empty::Response &res){

  //make sure we are in controlled mode within position controller
  //set_parameter("POS_C_MOD", SKYE_POS_C_MOD_CASCADE_PID);

  //send 1 in X direction for 5 seconds
  ros::Time begin = ros::Time::now();
  ros::Duration d(5.0);
  ros::Rate r(50); // 50 hz
  mavlink_message_t msg;
  geometry_msgs::TwistPtr pt(new geometry_msgs::Twist());


  pt->linear.x = 1.0;
  pt->linear.y = 0.0;
  pt->linear.z = 0.0;

  pt->angular.x = 0.0;
  pt->angular.y = 0.0;
  pt->angular.z = 0.0;

  sending_step_command = true;
  ROS_INFO("[skye_talker] sending step command in linear_x");

  while(ros::Time::now() <= (begin + d)){

    send_user_setpoint(pt);

    r.sleep();
  }

  sending_step_command = false;
  ROS_INFO("[skye_talker] sent step command in linear_x");

  return true;
}


};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeTalkerPlugin, mavplugin::MavRosPlugin)

