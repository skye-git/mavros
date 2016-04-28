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
#include "mavros/skye_base.h"
#include <ros/console.h>

#include <sensor_msgs/Imu.h>
	

namespace mavplugin {

/**
 * @brief Send data from skye_ros pkg to Skye's firmware running on the PX4 stack.
 */
class SkyeTalkerPlugin : public MavRosPlugin {
public:
	SkyeTalkerPlugin() :
		nh("~"),
		uas(nullptr)
	{}

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		skye_ros_imu_sk_sub = nh.subscribe(skye_base.getImuTopicName(), 
																			 10, 
																			 &SkyeTalkerPlugin::imu_sk_callback, this);
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle nh;
	UAS *uas;
	ros::Subscriber skye_ros_imu_sk_sub;
	skye_base::SkyeBase skye_base;

	/* -*- message handlers -*- */
	void imu_sk_callback(const sensor_msgs::ImuConstPtr &imu_sk_p) {

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

	  Eigen::Vector3d euler_angles = q_imu.matrix().eulerAngles(2, 1, 0); // Tait-Bryan, NED
	  roll = static_cast<float>(euler_angles[2]);
	  pitch = static_cast<float>(euler_angles[1]);
	  yaw = static_cast<float>(euler_angles[0]);

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

        //test
        mavlink_msg_param_set_pack_chan(UAS_PACK_CHAN(uas), &msg,
                UAS_PACK_TGT(uas),
                "SKYE_C_MODE",
                2,
                MAV_PARAM_TYPE_INT32 //(uint8_t)MAV_PARAM_TYPE_INT32
                );
        UAS_FCU(uas)->send_message(&msg);
        //end test

        //test
        /*mavlink_msg_param_set_pack_chan(UAS_PACK_CHAN(uas), &msg,
                UAS_PACK_TGT(uas),
                "SKYE_HIL_MODE",
                1,
                MAV_PARAM_TYPE_INT32 //(uint8_t)MAV_PARAM_TYPE_INT32
                );
        UAS_FCU(uas)->send_message(&msg);*/
        //end test
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeTalkerPlugin, mavplugin::MavRosPlugin)

