/**
 * @brief Publish skye_ros pkg IMU data to Skye's firmware running in PX4 Stack.
 * @file skye_gazebo_imu.cpp
 * @author Marco Tranzatto <marco@aerotainment.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <cmath>

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/Imu.h>
	

namespace mavplugin {

/**
 * @brief Send IMU data from skye_ros pkg to Skye's firmware running on the PX4 stack.
 */
class SkyeRosImu : public MavRosPlugin {
public:
	SkyeRosImu() :
		skye_ros_imu_nh("~"),
		uas(nullptr)
	{};

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		skye_ros_imu_ned_sub = skye_ros_imu_nh.subscribe<sensor_msgs::Imu>("skye_ros/sensor_msgs/imu_ned",
                                                        1,
                                                        boost::bind(&SkyeRosImu::imu_ned_callback, this, _1));

		ROS_INFO("*********************** Initi Skye_Gazebo_IMU! ***************************");
	}

	const message_map get_rx_handlers() {
		return {
							MESSAGE_HANDLER(MAVLINK_MSG_ID_HEARTBEAT, &SkyeRosImu::handle_heartbeat)
		};
	}

private:
	ros::NodeHandle skye_ros_imu_nh;
	UAS *uas;

	ros::Subscriber skye_ros_imu_ned_sub;

	/* -*- message handlers -*- */
	void handle_heartbeat(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		ROS_INFO_NAMED("Skye_Gazebo_IMU", "Skye_Gazebo_IMU::handle_heartbeat(%p, %u, %u)",
				msg, sysid, compid);
	}

	void imu_ned_callback(const sensor_msgs::ImuConstPtr &imu_ned_p) {
		
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeRosImu, mavplugin::MavRosPlugin)

