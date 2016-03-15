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
#include <skye_ros/GetLinkStateNed.h>
	

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

		client_skye_ros_get_link_state = skye_talker_nh.serviceClient<skye_ros::GetLinkStateNed>("/skye_ros/get_link_state_ned");

		ROS_INFO("*********************** Initi SkyeTalkerPlugin! ***************************");
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle skye_talker_nh;
	UAS *uas;

	ros::Subscriber skye_ros_imu_ned_sub;
	ros::ServiceClient client_skye_ros_get_link_state;

	/* -*- message handlers -*- */
	void imu_ned_callback(const sensor_msgs::ImuConstPtr &imu_ned_p) {
		//ROS_INFO("****************** SkyeTalkerPlugin: imu ned from skye ros ********");

		/* For now use perfect information taken from Gazebo about the state of Skye. 
		 * Use skye_ros/sensor_msgs/imu_ned only for timing.
		 */
		skye_ros::GetLinkStateNed srv;
		mavlink_message_t msg;
		Eigen::Quaterniond q_skye; /* Orientation of Skye's NED frame in Gazebo NED fixed frame. */
		tf::Quaternion tf_quat;

		/* Call the service skye_ros/get_link_state_ned from package skye_ros
		 * and read the state of Skye's hull.
		 */
		srv.request.link_name = "hull"; /**< @todo remove this hard coded name. */

		if(!client_skye_ros_get_link_state.call(srv))
	  {
    	ROS_ERROR("Failed to call service get_link_state_ned from skye_ros pkg.");
      return;
	  }

		/* Use message vehicle_attitude_hil which has the same fields of the original
		 * vehicle_attiude topic. Fill it with the information from Gazebo.
		 */ 
		q_skye.w() = srv.response.link_state.pose.orientation.w;
		q_skye.x() = srv.response.link_state.pose.orientation.x;
		q_skye.y() = srv.response.link_state.pose.orientation.y;
		q_skye.z() = srv.response.link_state.pose.orientation.z;
	  /*tf::quaternionMsgToTF(srv.response.link_state.pose.orientation, tf_quat);
	  tf::quaternionTFToEigen(tf_quat, q_skye);*/

	  //debug
	  /*Eigen::Vector3d ea = q_skye.matrix().eulerAngles(2, 1, 0);
	  ROS_INFO("q.w: %f\tq.x: %f\tq.y: %f\tq.z: %f\nyaw: %f\tpitch: %f\t roll: %f\n", 
	  					q_skye.w(), q_skye.x(), q_skye.y(), q_skye.z(),
	  					ea[0] * 180 / M_PI, ea[1] * 180 / M_PI, ea[2] * 180 / M_PI);*/
	  ROS_INFO("rr: %f\tpr: %f\tyr: %f", srv.response.link_state.twist.angular.x,
	  																	 srv.response.link_state.twist.angular.y,
	  																	 srv.response.link_state.twist.angular.z);
		/* Send the vehicle_attitude_hil message to Skye. */

		//mavlink_msg_TODO_pack_chan(UAS_PACK_CHAN(uas), &msg, TODO );
		//UAS_FCU(uas)->send_message(&msg);

	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeTalkerPlugin, mavplugin::MavRosPlugin)

