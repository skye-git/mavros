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
	

namespace mavplugin {

/**
 * @brief Read data from Skye's Firmware running on PX4 and publish them in ROS.
 */
class SkyeListenerPlugin : public MavRosPlugin {
public:
	SkyeListenerPlugin() :
		skye_listner_nh("~"),
		uas(nullptr)
	{};

	void initialize(UAS &uas_)
	{
		uas = &uas_;
	  /* Wait fort service to be advertised and available. */
    /*if(!client_skye_ros_apply_wrench.waitForExistence(ros::Duration(60.0))){
      ROS_FATAL("Service ApplyWrenchCogBf not advertised. Timeout reached, close application.");
    }*/
    seq_id = 0;
  }

	const message_map get_rx_handlers() {
		return {
							MESSAGE_HANDLER(MAVLINK_MSG_ID_ATTITUDE_CTRL_OUTPUT, &SkyeListenerPlugin::handle_att_ctrl_out),
							//MESSAGE_HANDLER(MAVLINK_MSG_ID_ALLOCATION_OUTPUT, &SkyeListenerPlugin::handle_allocator_out)
		};
	}

private:
	ros::NodeHandle skye_listner_nh;
	UAS *uas;
	skye_base::SkyeBase skye_base; // simple interface to interact with Skye simulation in Gazebo
  int seq_id;

	ros::Publisher torque_pub;	/*< attitide controller output torque in to be applied in the CoG. */
	ros::Publisher allocation_output_pub;	/*< allocator output thrust and angle for every AU. */
  ros::ServiceClient  client_skye_ros_apply_wrench; /**< Client to apply a body wrench in using skye_ros. */

	/* -*- message handlers -*- */

	void handle_att_ctrl_out(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) 
	{

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
    
    // apply the torque to Gazebo using skye_ros interface
    skye_ros::ApplyWrenchCogBf  srv;

    srv.request.wrench.force.x = 0.0;
    srv.request.wrench.force.y = 0.0;
    srv.request.wrench.force.z = 0.0;

    srv.request.wrench.torque.x = attiude_ctrl_output.M_x;
    srv.request.wrench.torque.y = attiude_ctrl_output.M_y;
    srv.request.wrench.torque.z = attiude_ctrl_output.M_z;

    srv.request.start_time = ros::Time::now();
    //srv.request.duration = ros::Duration(-1); //<--- causes problems
    srv.request.duration = ros::Duration(0.1);//TEST  this one works!    
                                                                    
    if (skye_base.setBodyWrench(srv))
    {
      //ROS_INFO("wrench applied!");
    }
    else
    {
      ROS_ERROR("[skye_listener] Failed to apply body wrench");
    }

  }

  void handle_allocator_out(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) 
	{

		/*ROS_INFO("*********************** allocator_out!!!! ***************************");

    mavlink_allocation_output_t allocator_output;
    mavlink_msg_allocation_output_decode(msg, &allocator_output);

    auto vector3_msg = boost::make_shared<geometry_msgs::Vector3>();
    
    // fill
		vector3_msg->x	=	attiude_ctrl_output.M_x;
		vector3_msg->y	=	attiude_ctrl_output.M_y;
		vector3_msg->z	=	attiude_ctrl_output.M_z;

		// publish
		allocation_output_pub.publish(vector3_msg);*/
  }
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeListenerPlugin, mavplugin::MavRosPlugin)

