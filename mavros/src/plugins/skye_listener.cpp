/**
 * @brief Read data from Skye's Firmware running on PX4 and publish them in ROS.
 * @file skye_listner.cpp
 * @author Marco Tranzatto <marco@aerotainment.com>
 *
 * @addtogroup plugin
 * @{
 */
#include <cmath>

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_srvs/Empty.h>

#include "mavros/skye_base.h"
#include "skye_ros/ApplyForceBf.h"
#include "skye_ros/ApplyTorqueBf.h"
#include "skye_ros/AllocatorOutput.h"

static const double kDegToRad = M_PI / 180.0;
static const int kDurationMultiplier = 5;

namespace mavplugin {

/**
 * @brief Read data from Skye's Firmware running on PX4 and publish them in ROS.
 */
class SkyeListenerPlugin : public MavRosPlugin {
//-----------------------------------------------------------------------------
public:
  SkyeListenerPlugin() :
    nh("~"),
    uas(nullptr)
  {};

//-----------------------------------------------------------------------------
void initialize(UAS &uas_){
  uas = &uas_;
  torque_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/skye_px4/attitude_ctrl_output", 10);
  allocator_output_pub = nh.advertise<skye_ros::AllocatorOutput>("/skye_px4/allocator_output", 10);
  force_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/skye_px4/position_ctrl_output", 10);
  debug_vec3_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/skye_px4/debug_vec3", 10);
  seq_id = 0;

//  debug_srv_ = nh.advertiseService("/skye_mr/debug_srv",
//                                   &SkyeListenerPlugin::debug_srv, this);

  time_last_pos_ctrl_out = time_last_att_ctrl_out = time_last_allocator_out = ros::Time::now();
}

//-----------------------------------------------------------------------------
const message_map get_rx_handlers(){
  return {
    MESSAGE_HANDLER(MAVLINK_MSG_ID_ATTITUDE_CTRL_OUTPUT, &SkyeListenerPlugin::handle_att_ctrl_out),
    MESSAGE_HANDLER(MAVLINK_MSG_ID_ALLOCATION_OUTPUT, &SkyeListenerPlugin::handle_allocator_out),
    MESSAGE_HANDLER(MAVLINK_MSG_ID_POSITION_CTRL_OUTPUT, &SkyeListenerPlugin::handle_pos_ctrl_out),
    MESSAGE_HANDLER(MAVLINK_MSG_ID_SKYE_DEBUG_VEC3, &SkyeListenerPlugin::handle_debug_vec3)
  };
}

//-----------------------------------------------------------------------------
private:
  ros::NodeHandle nh;
  UAS *uas;
  unsigned int seq_id;

  ros::Publisher torque_pub; /*< attitide controller output torque in to be applied in the CoG. */
  ros::Publisher allocator_output_pub; /*< allocator output thrust and angle for every AU. */
  ros::Publisher force_pub; /*< position controller output force. */
  ros::Publisher debug_vec3_pub; /*< debug vector publisher. */
//  ros::ServiceServer debug_srv_; /*< debugging service. */
  skye_base::SkyeBase skye_base;

  //"last time" variables
  ros::Time time_last_pos_ctrl_out;
  ros::Time time_last_att_ctrl_out;
  ros::Time time_last_allocator_out;

//-----------------------------------------------------------------------------
void handle_att_ctrl_out(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid){

  mavlink_attitude_ctrl_output_t attiude_ctrl_output;
  mavlink_msg_attitude_ctrl_output_decode(msg, &attiude_ctrl_output);

  auto vector3_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();

  // fill
  vector3_msg->header.seq = seq_id++;
  vector3_msg->header.stamp = ros::Time::now();
  vector3_msg->header.frame_id = "0";
  vector3_msg->vector.x	= attiude_ctrl_output.M_x;
  vector3_msg->vector.y	= attiude_ctrl_output.M_y;
  vector3_msg->vector.z	= attiude_ctrl_output.M_z;

  // publish
  torque_pub.publish(vector3_msg);

  // apply the torque to Gazebo
  skye_ros::ApplyTorqueBf  srv;

  // set torque, it's applied relative to the link's frame
  srv.request.torque.x = attiude_ctrl_output.M_x;
  srv.request.torque.y = attiude_ctrl_output.M_y;
  srv.request.torque.z = attiude_ctrl_output.M_z;

  srv.request.start_time = ros::Time::now();

  /* Torque application duration should be greater than the actual
   * time elapsed between two consecutive requests of torque application, otherwise
   * when "torque_duration" is elapsed torque's components go to 0.
   * Use the time difference between last two questes as estimate of next time
   * difference. Multuple by duration_multiplier to increase the estimated duration.
   */
  srv.request.duration = (srv.request.start_time - time_last_att_ctrl_out) * kDurationMultiplier;

  // Should we use attitude controller output instead of allocator output?
  if(!skye_base.useAllocatorOutput()){
    // call service if available
    if(skye_base.isBodyTorqueAvail()){
      if(skye_base.setBodyTorque(srv)){
        //ROS_INFO("wrench applied!");
      }
      else{
        ROS_ERROR("[skye_listener] Failed to apply body torque");
      }
    }
  }

  time_last_att_ctrl_out = srv.request.start_time;

}

//-----------------------------------------------------------------------------
void handle_pos_ctrl_out(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid){

  mavlink_position_ctrl_output_t position_ctrl_output;
  mavlink_msg_position_ctrl_output_decode(msg, &position_ctrl_output);

  auto vector3_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();

  // fill
  vector3_msg->header.seq = seq_id++;
  vector3_msg->header.stamp = ros::Time::now();
  vector3_msg->header.frame_id = "0";
  vector3_msg->vector.x = position_ctrl_output.F_x;
  vector3_msg->vector.y = position_ctrl_output.F_y;
  vector3_msg->vector.z = position_ctrl_output.F_z;

  // publish
  force_pub.publish(vector3_msg);

  // apply the force to Gazebo
  skye_ros::ApplyForceBf  srv;

  // set force, it's applied relative to the link's frame
  srv.request.force.x = position_ctrl_output.F_x;
  srv.request.force.y = position_ctrl_output.F_y;
  srv.request.force.z = position_ctrl_output.F_z;

  srv.request.start_time = ros::Time::now();

  /* Force application duration should be greater than the actual
   * time elapsed between two consecutive requests of force application, otherwise
   * when "force_duration" is elapsed force's components go to 0.
   * Use the time difference between last two questes as estimate of next time
   * difference. Multuple by duration_multiplier to increase the estimated duration.
   */
  srv.request.duration = (srv.request.start_time - time_last_pos_ctrl_out) * kDurationMultiplier;

  // Should we use attitude controller output instead of allocator output?
  if(!skye_base.useAllocatorOutput()){
    // call service if available
    if(skye_base.isBodyForceAvail()){
      if(skye_base.setBodyForce(srv)){
        //ROS_INFO("wrench applied!");
      }
      else{
        ROS_ERROR("[skye_listener] Failed to apply body force");
      }
    }
  }

  time_last_pos_ctrl_out = srv.request.start_time;

}

//-----------------------------------------------------------------------------
void handle_allocator_out(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid){

  // Apply a 2D force to each AU based on the output of the allocator
  mavlink_allocation_output_t allocator_output;
  mavlink_msg_allocation_output_decode(msg, &allocator_output);

  auto allocator_out_msg = boost::make_shared<skye_ros::AllocatorOutput>();

  // fill
  allocator_out_msg->header.seq = seq_id++;
  allocator_out_msg->header.stamp = ros::Time::now();
  allocator_out_msg->header.frame_id = "0";

  skye_ros::ApplyForce2DCogBf  srv;
  srv.request.start_time = ros::Time::now();

  /* Force application duration should be greater than the actual
   * time elapsed between two consecutive requests of force application, otherwise
   * when "force_duration" is elapsed force's components go to 0.
   * Use the time difference between last two questes as estimate of next time
   * difference. Multuple by duration_multiplier to increase the estimated duration.
   */
  srv.request.duration = (srv.request.start_time - time_last_allocator_out) * kDurationMultiplier;

  for(int i = 0; i < skye_base.getAuNumber(); i++){
    srv.request.Fx = allocator_output.thrust[i] * cos(allocator_output.angle[i] * kDegToRad);
    srv.request.Fy = allocator_output.thrust[i] * sin(allocator_output.angle[i] * kDegToRad);

    allocator_out_msg->thrust[i] = allocator_output.thrust[i];
    allocator_out_msg->angle[i] = allocator_output.angle[i];

    // Should we use allocator output ?
    if(skye_base.useAllocatorOutput()){
      // call service if available
      if(skye_base.isAuForce2DAvail(i)){ //TODO restore me
        if(skye_base.setAuForce2D(srv, i)){
          //ROS_INFO("force applied!");
        }
        else{
          ROS_ERROR_STREAM("[skye_listener] Failed to apply 2D force to AU " << std::to_string(i+1));
        }
      }
    }
  }

  // publish
  allocator_output_pub.publish(allocator_out_msg);

  time_last_allocator_out = srv.request.start_time;

}

//-----------------------------------------------------------------------------
void handle_debug_vec3(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid){
  mavlink_skye_debug_vec3_t debug_vec3;
  mavlink_msg_skye_debug_vec3_decode(msg, &debug_vec3);

  auto vector3_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();

  // fill
  vector3_msg->header.seq = seq_id++;
  vector3_msg->header.stamp = ros::Time::now();
  vector3_msg->header.frame_id = "0";
  vector3_msg->vector.x = debug_vec3.x;
  vector3_msg->vector.y = debug_vec3.y;
  vector3_msg->vector.z = debug_vec3.z;

  // publish
  debug_vec3_pub.publish(vector3_msg);
}

//-----------------------------------------------------------------------------
/*bool debug_srv(std_srvs::Empty::Request &req,
               std_srvs::Empty::Response &res){


  static ros::ServiceClient apply_2dforce_hull = nh.serviceClient<skye_ros::ApplyForce2DCogBf>("/skye_gz/au_1/apply_force_2D");

  skye_ros::ApplyForce2DCogBf  srv;
  srv.request.start_time = ros::Time::now();



  ros::Time begin = ros::Time::now();
  ros::Duration d(2.0);
  ros::Rate r(50); // 50 hz

  ROS_INFO("[skye_listener] sending debug command");

//  while(ros::Time::now() <= (begin + d)){

//    apply_2dforce_hull.call(srv);

//    r.sleep();
//  }

  //Test thrust
//  srv.request.Fx = 15.0;
//  srv.request.Fy = 0.0;
//  srv.request.duration = ros::Duration(2.0);
//  apply_2dforce_hull.call(srv);
//  ros::Duration(2.0).sleep();

//  srv.request.Fx = 0.0;
//  srv.request.Fy = 0.0;
//  srv.request.duration = ros::Duration(2.0);
//  apply_2dforce_hull.call(srv);
//  ros::Duration(2.0).sleep();

//  srv.request.Fx = 5.0;
//  srv.request.Fy = 0.0;
//  srv.request.duration = ros::Duration(2.0);
//  apply_2dforce_hull.call(srv);
//  ros::Duration(2.0).sleep();

//  srv.request.Fx = 0.0;
//  srv.request.Fy = 0.0;
//  srv.request.duration = ros::Duration(2.0);
//  apply_2dforce_hull.call(srv);
//  ros::Duration(2.0).sleep();

//  srv.request.Fx = 1.0;
//  srv.request.Fy = 0.0;
//  srv.request.duration = ros::Duration(2.0);
//  apply_2dforce_hull.call(srv);
//  ros::Duration(2.0).sleep();

//  srv.request.Fx = 0.0;
//  srv.request.Fy = 0.0;
//  srv.request.duration = ros::Duration(2.0);
//  apply_2dforce_hull.call(srv);
//  ros::Duration(2.0).sleep();

  //Test orientation
  double theta;

  theta = 0.5;
  srv.request.Fx = cos(theta);
  srv.request.Fy = sin(theta);
  srv.request.duration = ros::Duration(2.0);
  apply_2dforce_hull.call(srv);
  ros::Duration(2.0).sleep();

  theta = 0.0;
  srv.request.Fx = cos(theta);
  srv.request.Fy = sin(theta);
  srv.request.duration = ros::Duration(2.0);
  apply_2dforce_hull.call(srv);
  ros::Duration(2.0).sleep();

  theta = 1.0;
  srv.request.Fx = cos(theta);
  srv.request.Fy = sin(theta);
  srv.request.duration = ros::Duration(2.0);
  apply_2dforce_hull.call(srv);
  ros::Duration(2.0).sleep();

  theta = 0.0;
  srv.request.Fx = cos(theta);
  srv.request.Fy = sin(theta);
  srv.request.duration = ros::Duration(2.0);
  apply_2dforce_hull.call(srv);
  ros::Duration(2.0).sleep();

  theta = M_PI;
  srv.request.Fx = cos(theta);
  srv.request.Fy = sin(theta);
  srv.request.duration = ros::Duration(2.0);
  apply_2dforce_hull.call(srv);
  ros::Duration(2.0).sleep();

  srv.request.Fx = 0.0;
  srv.request.Fy = 0.0;
  srv.request.duration = ros::Duration(2.0);
  apply_2dforce_hull.call(srv);
  ros::Duration(2.0).sleep();

  ROS_INFO("[skye_listener] sent debug command");

  return true;
}*/

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeListenerPlugin, mavplugin::MavRosPlugin)

