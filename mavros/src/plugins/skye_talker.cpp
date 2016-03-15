/**
 * @brief Send data from ROS to Skye's firmware running in PX4 Stack.
 * @file skye_talker.cpp
 * @author Marco Tranzatto <marco@aerotainment.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <cmath>

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/GetLinkState.h>
	

namespace mavplugin {

/**
 * @brief Send data from skye_ros pkg to Skye's firmware running on the PX4 stack.
 */
class SkyeTalkerPlugin : public MavRosPlugin {
public:
	SkyeTalkerPlugin() :
		skye_talker_nh("~"),
		uas(nullptr)
	{};

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		skye_ros_imu_ned_sub = skye_talker_nh.subscribe<sensor_msgs::Imu>("skye_ros/sensor_msgs/imu_ned",
                                                    1,
                                                    boost::bind(&SkyeTalkerPlugin::imu_ned_callback, this, _1));

		client_skye_ros_get_link_state = skye_talker_nh.serviceClient<gazebo_msgs::GetLinkState>("skye_ros/get_link_state_ned");

		ROS_INFO("*********************** Initi SkyeTalkerPlugin! ***************************");
	}

	const message_map get_rx_handlers() {
		return {
							MESSAGE_HANDLER(MAVLINK_MSG_ID_HEARTBEAT, &SkyeTalkerPlugin::handle_heartbeat)
		};
	}

private:
	ros::NodeHandle skye_talker_nh;
	UAS *uas;

	ros::Subscriber skye_ros_imu_ned_sub;
	ros::ServiceClient client_skye_ros_get_link_state;

	/* -*- message handlers -*- */
	void handle_heartbeat(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		ROS_INFO_NAMED("SkyeTalkerPlugin", "SkyeTalkerPlugin::handle_heartbeat(%p, %u, %u)",
				msg, sysid, compid);
	}

	void imu_ned_callback(const sensor_msgs::ImuConstPtr &imu_ned_p) {
		ROS_INFO("****************** SkyeTalkerPlugin: imu ned from skye ros ********");

		/* For now use perfect information taken from Gazebo about the state of Skye. 
		 * Use skye_ros/sensor_msgs/imu_ned only for timing.
		 * Call the service skye_ros/get_link_state_ned from package skye_ros
		 * and read the state of Skye's hull.
		*/



		/* Use message vehicle_attitude_hil which has the same fields of the original
		 * vehicle_attiude topic. Fill it with the information from Gazebo.
		*/

	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeTalkerPlugin, mavplugin::MavRosPlugin)

