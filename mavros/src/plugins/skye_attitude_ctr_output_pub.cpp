/**
 * @brief Skye attitude controller plugin
 * @file skye_attitude_ctr_output_pub.cpp
 * @author Marco Tranzatto <marco@aerotainment.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <cmath>

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Vector3.h>

namespace mavplugin {

/**
 * @brief Skye attitude controller output data publication plugin
 */
class SkyeAttCtrPubPlugin : public MavRosPlugin {
public:
	SkyeAttCtrPubPlugin() :
		skye_att_ctr_nh("~"),
		uas(nullptr)
	{};

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		torque_pub = skye_att_ctr_nh.advertise<geometry_msgs::Vector3>("/skye/attitude_ctrl_output", 10);

		ROS_INFO_NAMED("SKYEEEEEEE", "initialize");
	}

	const message_map get_rx_handlers() {
		return {
							MESSAGE_HANDLER(MAVLINK_MSG_ID_ATTITUDE_CTRL_OUTPUT, &SkyeAttCtrPubPlugin::handle_att_ctrl_out)
		};
	}

private:
	ros::NodeHandle skye_att_ctr_nh;
	UAS *uas;
	std::string frame_id;

	ros::Publisher torque_pub;

	/* -*- message handlers -*- */

	void handle_att_ctrl_out(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) 
	{

		ROS_INFO("Skye!!!!");

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
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeAttCtrPubPlugin, mavplugin::MavRosPlugin)

