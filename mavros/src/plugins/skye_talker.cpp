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

		skye_ros_imu_bf_sub = skye_talker_nh.subscribe("/skye_ros/sensor_msgs/imu_bf", 
																										10, 
																										&SkyeTalkerPlugin::imu_bf_callback, this);

	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle skye_talker_nh;
	UAS *uas;

	ros::Subscriber skye_ros_imu_bf_sub;

	/* -*- message handlers -*- */
	void imu_bf_callback(const sensor_msgs::ImuConstPtr &imu_bf_p) {

		mavlink_message_t msg;
		Eigen::Quaterniond q_imu; 
		tf::Quaternion tf_quat;
		float roll, pitch, yaw;
		float q[4];

		/* Convert data to fullfill a mavlink message. */
	  tf::quaternionMsgToTF(imu_bf_p->orientation, tf_quat);
	  tf::quaternionTFToEigen(tf_quat, q_imu);
	  Eigen::Vector3d euler_angles = q_imu.matrix().eulerAngles(2, 1, 0); // Tait-Bryan, NED
	  roll = static_cast<float>(euler_angles[2]);
	  pitch = static_cast<float>(euler_angles[1]);
	  yaw = static_cast<float>(euler_angles[0]);

	  q[0] = static_cast<float>(q_imu.w());
	  q[1] = static_cast<float>(q_imu.x());
	  q[2] = static_cast<float>(q_imu.y());
	  q[3] = static_cast<float>(q_imu.z());

	  int i = 0;
		/* Send the skye_imu_attitude_hil message to Skye. */
		/*mavlink_msg_skye_imu_attitude_hil_pack_chan(UAS_PACK_CHAN(uas), &msg, 
																								roll,
																								pitch,
																								yaw,
																								imu_bfd_p->angular_velocity.x,
																								imu_bf_p->angular_velocity.y,
																								imu_bf_p->angular_velocity.z,
																								q);
		UAS_FCU(uas)->send_message(&msg);*/

	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeTalkerPlugin, mavplugin::MavRosPlugin)

