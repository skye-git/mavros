/**
 * @brief Read data from Skye's Firmware running on PX4 and publish them in ROS.
 * @file skye_listner.cpp
 * @author Marco Tranzatto <marco@aerotainment.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <cmath>

//test - better to include this in the CMakeLists.txt file of Mavros
#include </home/marco/skye-git/c_library/skye/mavlink.h>

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Vector3.h>
	

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

		torque_pub = skye_listner_nh.advertise<geometry_msgs::Vector3>("/skye/attitude_ctrl_output", 10);

		ROS_INFO("*********************** Initi SkyeListenerPlugin! ***************************");
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
	std::string frame_id;

	ros::Publisher torque_pub;	/*< attitide controller output torque in to be applied in the CoG. */
	ros::Publisher allocation_output_pub;	/*< allocator output thrust and angle for every AU. */

	/* -*- message handlers -*- */

	void handle_att_ctrl_out(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) 
	{

		ROS_INFO("*********************** att_ctrl_out!!!! ***************************");

    mavlink_attitude_ctrl_output_t attiude_ctrl_output;
    mavlink_msg_attitude_ctrl_output_decode(msg, &attiude_ctrl_output);

    auto vector3_msg = boost::make_shared<geometry_msgs::Vector3>();
    
    // fill
		vector3_msg->x	=	attiude_ctrl_output.M_x;
		vector3_msg->y	=	attiude_ctrl_output.M_y;
		vector3_msg->z	=	attiude_ctrl_output.M_z;

		// publish
		torque_pub.publish(vector3_msg);
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

