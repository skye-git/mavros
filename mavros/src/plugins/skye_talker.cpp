/**
 * @brief Send data from ROS to Skye's firmware running in PX4 Stack.
 * @file skye_talker.cpp
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
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/Imu.h>
	

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

		skye_ros_imu_ned_sub = skye_talker_nh.subscribe("/skye_ros/sensor_msgs/imu_ned", 
																										10, 
																										&SkyeTalkerPlugin::imu_ned_callback, this);

		ROS_INFO("*********************** Initi SkyeTalkerPlugin! ***************************");
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle skye_talker_nh;
	UAS *uas;

	ros::Subscriber skye_ros_imu_ned_sub;

	/* -*- message handlers -*- */
	void imu_ned_callback(const sensor_msgs::ImuConstPtr &imu_ned_p) {

		mavlink_message_t msg;
		Eigen::Quaterniond q_imu; 
		tf::Quaternion tf_quat;
		float roll, pitch, yaw;
		float q[4];

		/* Convert data to fullfill a mavlink message. */
	  tf::quaternionMsgToTF(imu_ned_p->orientation, tf_quat);
	  tf::quaternionTFToEigen(tf_quat, q_imu);
	  Eigen::Vector3d euler_angles = q_imu.matrix().eulerAngles(2, 1, 0); // Tait-Bryan, NED
	  roll = static_cast<float>(euler_angles[2]);
	  pitch = static_cast<float>(euler_angles[1]);
	  yaw = static_cast<float>(euler_angles[0]);

	  q[0] = static_cast<float>(q_imu.w());
	  q[1] = static_cast<float>(q_imu.x());
	  q[2] = static_cast<float>(q_imu.y());
	  q[3] = static_cast<float>(q_imu.z());

		/* Send the skye_imu_attitude_hil message to Skye. */
		/*mavlink_msg_skye_imu_attitude_hil_pack_chan(UAS_PACK_CHAN(uas), &msg, 
																								roll,
																								pitch,
																								yaw,
																								imu_ned_p->angular_velocity.x,
																								imu_ned_p->angular_velocity.y,
																								imu_ned_p->angular_velocity.z,
																								q);
		UAS_FCU(uas)->send_message(&msg);*/

	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeTalkerPlugin, mavplugin::MavRosPlugin)

