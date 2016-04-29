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
public:
    SkyeListenerPlugin() :
        nh("~"),
        uas(nullptr)
    {};

    void initialize(UAS &uas_)
    {
        uas = &uas_;
        torque_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/skye_px4/attitude_ctrl_output", 10);
        allocator_output_pub = nh.advertise<skye_ros::AllocatorOutput>("/skye_px4/allocator_output", 10);
        seq_id = 0;
    }

    const message_map get_rx_handlers() {
        return {
            MESSAGE_HANDLER(MAVLINK_MSG_ID_ATTITUDE_CTRL_OUTPUT, &SkyeListenerPlugin::handle_att_ctrl_out),
            MESSAGE_HANDLER(MAVLINK_MSG_ID_ALLOCATION_OUTPUT, &SkyeListenerPlugin::handle_allocator_out)
        };
    }

private:
    ros::NodeHandle nh;
    UAS *uas;
    unsigned int seq_id;

    ros::Publisher torque_pub;	/*< attitide controller output torque in to be applied in the CoG. */
    ros::Publisher allocator_output_pub;	/*< allocator output thrust and angle for every AU. */
    skye_base::SkyeBase skye_base;

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
        srv.request.duration = ros::Duration(1.0/25.0);//TEST  this one works!

        // call service if available
        if(skye_base.isBodyWrenchAvail()){
            if(skye_base.setBodyWrench(srv))
            {
                //ROS_INFO("wrench applied!");
            }
            else
            {
                ROS_ERROR("[skye_listener] Failed to apply body wrench");
            }
        }

    }

    void handle_allocator_out(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid)
    {

        // Apply a 2D force to each AU based on the output of the allocator
        mavlink_allocation_output_t allocator_output;
        mavlink_msg_allocation_output_decode(msg, &allocator_output);

        auto alocator_out_msg = boost::make_shared<skye_ros::AllocatorOutput>();

        // fill
        alocator_out_msg->header.seq = seq_id++;
        alocator_out_msg->header.stamp = ros::Time::now();
        alocator_out_msg->header.frame_id = "0";

        skye_ros::ApplyForce2DCogBf  srv;

        for(int i = 0; i < 6; i++){
            srv.request.Fx = allocator_output.thrust[i] * cos(allocator_output.angle[i] * DEG_TO_RAD);
            srv.request.Fx = allocator_output.thrust[i] * sin(allocator_output.angle[i] * DEG_TO_RAD);

            alocator_out_msg->thrust[i] = allocator_output.thrust[i];
            alocator_out_msg->angle[i] = allocator_output.angle[i];

            srv.request.start_time = ros::Time::now();
            srv.request.duration = ros::Duration(1.0/25.0);//TEST  this one works!
            // call service if available
            /*if(skye_base.isAuForce2DAvail(i)){
        if(skye_base.setAuForce2D(srv, i))
        {
          //ROS_INFO("force applied!");
        }
        else
        {
          ROS_ERROR_STREAM("[skye_listener] Failed to apply 2D force to AU " << std::to_string(i+1));
        }
      } */
        }

        // publish
        allocator_output_pub.publish(alocator_out_msg);

    }
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeListenerPlugin, mavplugin::MavRosPlugin)

