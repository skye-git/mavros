/**
 * @brief Send UWB distance measurements from ROS to Skye's firmware running in PX4 Stack.
 * @file skye_uwb_talker.cpp
 * @author Vincent Mai <maiv@ethz.ch>
 *
 * @addtogroup plugin
 * @{
 */

#include <uwb_sensor_plugin/uwb_sensor_plugin_msg.h>
#include <pluginlib/class_list_macros.h>
#include <mavros/mavros_plugin.h>
#include <vector>

namespace mavplugin
{

  class SkyeUWBSensorRawHilPlugin : public MavRosPlugin
  {
  public:

    SkyeUWBSensorRawHilPlugin() :
      nh("~"),
      uas(nullptr)
    {
    }

    ~SkyeUWBSensorRawHilPlugin()
    {
    }

    void initialize(UAS &uas_)
    {
      uas = &uas_;
      uwb_sensors_1_sub = nh.subscribe("/uwb_sensor_plugin/uwb_sensor/tag_1",
                                       1,
                                       &SkyeUWBSensorRawHilPlugin::UWB_1_callback, this);

      uwb_sensors_2_sub = nh.subscribe("/uwb_sensor_plugin/uwb_sensor/tag_2",
                                       1,
                                       &SkyeUWBSensorRawHilPlugin::UWB_2_callback, this);

      uwb_sensors_3_sub = nh.subscribe("/uwb_sensor_plugin/uwb_sensor/tag_3",
                                       1,
                                       &SkyeUWBSensorRawHilPlugin::UWB_3_callback, this);
    }

    void send_message(uint8_t tag_id, const uwb_sensor_plugin::uwb_sensor_plugin_msg &ros_msg) {
      std::vector<uint8_t> anchor_ids_vec = ros_msg.anchor_ids;
      std::vector<float> distances_vec(ros_msg.distances.begin(), ros_msg.distances.end());
      for(int i = 0; i < anchor_ids_vec.size(); i++) {
        mavlink_message_t mav_msg;
        double time_d = ros::Time::now().toSec();
        uint64_t time_micro = static_cast<uint64_t>(time_d*1e6);
        mavlink_msg_uwb_sensor_raw_hil_pack_chan(UAS_PACK_CHAN(uas), &mav_msg,
                                           time_micro,
                                           tag_id,
                                           anchor_ids_vec[i],
                                           distances_vec[i]);
      UAS_FCU(uas)->send_message(&mav_msg);
    }
  }

    const message_map get_rx_handlers(){
      return {
        //MESSAGE_HANDLER(MAVLINK_MSG_ID_UWB_SENSOR_RAW, &SkyeListenerPlugin::handle_uwb_meas_out)
      };
    }

  private:
    ros::NodeHandle nh;
    UAS *uas;
    ros::Subscriber uwb_sensors_1_sub; // UWB sensor topic for tag 1
    ros::Subscriber uwb_sensors_2_sub; // UWB sensor topic for tag 2
    ros::Subscriber uwb_sensors_3_sub; // UWB sensor topic for tag 3

    void UWB_1_callback(const uwb_sensor_plugin::uwb_sensor_plugin_msg &ros_msg)
    {
      send_message(0, ros_msg);
    }

    void UWB_2_callback(const uwb_sensor_plugin::uwb_sensor_plugin_msg &ros_msg)
    {
      send_message(1, ros_msg);
    }

    void UWB_3_callback(const uwb_sensor_plugin::uwb_sensor_plugin_msg &ros_msg)
    {
      send_message(2, ros_msg);
    }

  };
};	// namespace mavplugin


PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeUWBSensorRawHilPlugin, mavplugin::MavRosPlugin)
