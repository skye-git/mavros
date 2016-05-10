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

#include "mavros/skye_base.h"
#include "skye_ros/ApplyWrenchCogBf.h"
#include "skye_ros/AllocatorOutput.h"

#define DEG_TO_RAD M_PI / 180.0	

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
  seq_id = 0;
}

//-----------------------------------------------------------------------------
const message_map get_rx_handlers(){
  return {
    MESSAGE_HANDLER(MAVLINK_MSG_ID_ATTITUDE_CTRL_OUTPUT, &SkyeListenerPlugin::handle_att_ctrl_out),
    MESSAGE_HANDLER(MAVLINK_MSG_ID_ALLOCATION_OUTPUT, &SkyeListenerPlugin::handle_allocator_out),
    MESSAGE_HANDLER(MAVLINK_MSG_ID_POSITION_CTRL_OUTPUT, &SkyeListenerPlugin::handle_pos_ctrl_out)
  };
}

//-----------------------------------------------------------------------------
private:
  ros::NodeHandle nh;
  UAS *uas;
  unsigned int seq_id;

  ros::Publisher torque_pub;	/*< attitide controller output torque in to be applied in the CoG. */
  ros::Publisher allocator_output_pub;	/*< allocator output thrust and angle for every AU. */
  ros::Publisher force_pub;
  skye_base::SkyeBase skye_base;

//-----------------------------------------------------------------------------
void handle_att_ctrl_out(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid){

  mavlink_attitude_ctrl_output_t attiude_ctrl_output;
  mavlink_msg_attitude_ctrl_output_decode(msg, &attiude_ctrl_output);

  auto vector3_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();

  // fill
  vector3_msg->header.seq = seq_id++;
  vector3_msg->header.stamp = ros::Time::now();
  vector3_msg->header.frame_id = "0";
  vector3_msg->vector.x	=	attiude_ctrl_output.M_x;
  vector3_msg->vector.y	=	attiude_ctrl_output.M_y;
  vector3_msg->vector.z	=	attiude_ctrl_output.M_z;

  // publish
  torque_pub.publish(vector3_msg);

  // apply the torque to Gazebo
  skye_ros::ApplyWrenchCogBf  srv;

  // the set wrench force to 0. This is a relative force added to the force which is
  // already applied ot the body.
  srv.request.wrench.force.x = 0.0;
  srv.request.wrench.force.y = 0.0;
  srv.request.wrench.force.z = 0.0;

  // the set wrench torque. This is a relative torque added to the torque which is
  // already applied ot the body.
  srv.request.wrench.torque.x = attiude_ctrl_output.M_x;
  srv.request.wrench.torque.y = attiude_ctrl_output.M_y;
  srv.request.wrench.torque.z = attiude_ctrl_output.M_z;

  srv.request.start_time = ros::Time::now();
  //srv.request.duration = ros::Duration(-1); //<--- causes problems
  srv.request.duration = ros::Duration(1.0/25.0);//TEST  this one works!

  // call service if available
  if(skye_base.isBodyWrenchAvail()){
    if(skye_base.setBodyWrench(srv)){
      //ROS_INFO("wrench applied!");
    }
    else{
      ROS_ERROR("[skye_listener] Failed to apply body wrench");
    }
  }

}

//-----------------------------------------------------------------------------
void handle_allocator_out(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid){

  // Apply a 2D force to each AU based on the output of the allocator
  mavlink_allocation_output_t allocator_output;
  mavlink_msg_allocation_output_decode(msg, &allocator_output);

  auto alocator_out_msg = boost::make_shared<skye_ros::AllocatorOutput>();

  // fill
  alocator_out_msg->header.seq = seq_id++;
  alocator_out_msg->header.stamp = ros::Time::now();
  alocator_out_msg->header.frame_id = "0";

  skye_ros::ApplyForce2DCogBf  srv;

  for(int i = 0; i < skye_base.getAuNumber(); i++){
      srv.request.Fx = allocator_output.thrust[i] * cos(allocator_output.angle[i] * DEG_TO_RAD);
      srv.request.Fx = allocator_output.thrust[i] * sin(allocator_output.angle[i] * DEG_TO_RAD);

      alocator_out_msg->thrust[i] = allocator_output.thrust[i];
      alocator_out_msg->angle[i] = allocator_output.angle[i];

      srv.request.start_time = ros::Time::now();
      srv.request.duration = ros::Duration(1.0/25.0);//TEST  this one works!
      // call service if available
      /*if(skye_base.isAuForce2DAvail(i)){ //TODO restore me
            if(skye_base.setAuForce2D(srv, i))
            {
                //ROS_INFO("force applied!");
            }
            else
            {
                ROS_ERROR_STREAM("[skye_listener] Failed to apply 2D force to AU " << std::to_string(i+1));
            }
        }*/
    }

  // publish
  allocator_output_pub.publish(alocator_out_msg);
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
  vector3_msg->vector.x =	position_ctrl_output.F_x;
  vector3_msg->vector.y =	position_ctrl_output.F_y;
  vector3_msg->vector.z =	position_ctrl_output.F_z;

  // publish
  force_pub.publish(vector3_msg);

  // apply the force to Gazebo
  skye_ros::ApplyWrenchCogBf  srv;

  // the set wrench force. This is a relative force added to the force which is
  // already applied ot the body.
  srv.request.wrench.force.x = position_ctrl_output.F_x;
  srv.request.wrench.force.y = position_ctrl_output.F_y;
  srv.request.wrench.force.z = position_ctrl_output.F_z;

  // the set wrench troque to 0. This is a relative torque added to the torque which is
  // already applied ot the body.
  srv.request.wrench.torque.x = 0.0;
  srv.request.wrench.torque.y = 0.0;
  srv.request.wrench.torque.z = 0.0;

  srv.request.start_time = ros::Time::now();
  srv.request.duration = ros::Duration(1.0/25.0);//TEST  this one works!

  // call service if available
  if(skye_base.isBodyWrenchAvail()){
    if(skye_base.setBodyWrench(srv)){
      //ROS_INFO("wrench applied!");
    }
    else{
      ROS_ERROR("[skye_listener] Failed to apply body wrench");
    }
  }

}

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeListenerPlugin, mavplugin::MavRosPlugin)

