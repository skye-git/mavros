/**
 * @brief Send position control output in N from ROS to Skye's firmware running in PX4 Stack.
 * @file skye_uwb_talker.cpp
 * @author Vincent Mai <maiv@ethz.ch>
 *
 * @addtogroup plugin
 * @{
 */

#include <geometry_msgs/Vector3.h>
#include <pluginlib/class_list_macros.h>
#include <mavros/mavros_plugin.h>

namespace mavplugin
{

  class SkyePositionCtrlOutputHilPlugin : public MavRosPlugin
  {
  public:

    SkyePositionCtrlOutputHilPlugin() :
      nh("~"),
      uas(nullptr)
    {
    }

    ~SkyePositionCtrlOutputHilPlugin()
    {
    }

    void initialize(UAS &uas_)
    {
      uas = &uas_;
      force_sub = nh.subscribe("/skye_ros/sensor_msgs/input_force_hil",
                                       1,
                                       &SkyePositionCtrlOutputHilPlugin::force_callback, this);

    }

    void send_message(const geometry_msgs::Vector3 &ros_msg) {
        mavlink_message_t mav_msg;
        mavlink_msg_position_ctrl_output_pack_chan(UAS_PACK_CHAN(uas), &mav_msg,
                                           ros_msg.x,
                                           ros_msg.y,
                                           ros_msg.z);
      UAS_FCU(uas)->send_message(&mav_msg);
    }
  

    const message_map get_rx_handlers(){
      return {
        //MESSAGE_HANDLER(MAVLINK_MSG_ID_UWB_SENSOR_RAW, &SkyeListenerPlugin::handle_uwb_meas_out)
      };
    }

  private:
    ros::NodeHandle nh;
    UAS *uas;
    ros::Subscriber force_sub; 

    void force_callback(const geometry_msgs::Vector3 &ros_msg)
    {
      send_message(ros_msg);
    }
  };
};	// namespace mavplugin


PLUGINLIB_EXPORT_CLASS(mavplugin::SkyePositionCtrlOutputHilPlugin, mavplugin::MavRosPlugin)
