/****************************************************************************
*
*   Copyright (C) 2016 Aerotainment Labs. All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, is not permitted unless explicitely stated by the owner
*   of the copyright.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

/**
 * @file skye_helper_services.cpp
 * Mavros plugin to provide some useful services to intercat with the FMU.
 *
 * @author Marco Tranzatto <marco@skye.aero>
 */
#include <mavros/mavros_plugin.h>

#include <pluginlib/class_list_macros.h>
#include <mavros/skye_common_helpers.h>
#include <mavros_msgs/SkyeCMode.h>
#include <mavros_msgs/ParamSet.h>


namespace mavplugin {


class SkyeHelperServicesPlugin : public MavRosPlugin {
public:
//-----------------------------------------------------------------------------
  SkyeHelperServicesPlugin() :
    nh_private("~"),
    nh_public(),
    uas(nullptr)
  { };

//-----------------------------------------------------------------------------
  /**
   * Plugin initializer. Constructor should not do this.
   */
  void initialize(UAS &uas_)
  {
    uas = &uas_;

    // Service to set POC_C_MOD parameter on the FMU
    set_c_mod_pos_srv = nh_private.advertiseService("skye_mr/set_pos_c_mod",
                                                    &SkyeHelperServicesPlugin::set_pos_c_mod,
                                                    this);

    // Service to set ATT_C_MOD parameter on the FMU
    set_c_mod_att_srv = nh_private.advertiseService("skye_mr/set_att_c_mod",
                                                    &SkyeHelperServicesPlugin::set_att_c_mod,
                                                    this);

    // Service to set a generic parameter on the FMU
    set_skye_param_srv = nh_private.advertiseService( "skye_mr/set_param",
                                                      &SkyeHelperServicesPlugin::set_skye_param,
                                                      this);
  }

//-----------------------------------------------------------------------------
  /**
   * This function returns message<->handler mapping
   *
   * Each entry defined by @a MESSAGE_HANDLER() macro
   */
  const message_map get_rx_handlers() {
          return { /* Rx disabled */ };
  }

private:
//-----------------------------------------------------------------------------
  ros::NodeHandle nh_private;
  ros::NodeHandle nh_public;
  UAS *uas;

  ros::ServiceServer set_c_mod_pos_srv; /* Service to set C_MOD_POS parameter in px4 */
  ros::ServiceServer set_c_mod_att_srv; /* Service to set C_MOD_ATT parameter in px4 */
  ros::ServiceServer set_skye_param_srv; /* Service to set a parameter in px4 */



//-----------------------------------------------------------------------------
  bool set_att_c_mod(mavros_msgs::SkyeCMode::Request &req,
                     mavros_msgs::SkyeCMode::Response &res) {

    bool send_new_mode = false;
    std::string str_msg = "";

    // check mode is valid in according to SKYE_ATT_C_MOD in skye.h
    switch (req.mode) {
      case SKYE_ATT_C_MOD_MANUAL:
        str_msg = "[skye_helper_services]: ATT_C_MOD switched to SKYE_ATT_C_MOD_MANUAL";
        send_new_mode = true;
        break;

      case SKYE_ATT_C_MOD_5_DOF:
        str_msg = "[skye_helper_services]: ATT_C_MOD switched to SKYE_ATT_C_MOD_5_DOF";
        send_new_mode = true;
        break;

      case SKYE_ATT_C_MOD_6_DOF:
        str_msg = "[skye_helper_services]: ATT_C_MOD switched to SKYE_ATT_C_MOD_6_DOF";
        send_new_mode = true;
        break;

      case SKYE_ATT_C_MOD_6_DOFI:
        str_msg = "[skye_helper_services]: ATT_C_MOD switched to SKYE_ATT_C_MOD_6_DOFI";
        send_new_mode = true;
        break;

      case SKYE_ATT_C_MOD_GEOM:
        str_msg = "[skye_helper_services]: ATT_C_MOD switched to SKYE_ATT_C_MOD_GEOM";
        send_new_mode = true;
        break;

      case SKYE_ATT_C_MOD_MAX:
        str_msg = "[skye_helper_services]: ATT_C_MOD switched to SKYE_ATT_C_MOD_MAX";
        send_new_mode = true;
        break;

      default:
        str_msg = "[skye_helper_services]: specified mode in ATT_C_MOD request is not valid";
        send_new_mode = false;
        break;
    }

    if (send_new_mode) {
      set_parameter(uas, "ATT_C_MOD", req.mode);
    }

    ROS_INFO_STREAM(str_msg);

    res.success = send_new_mode;

    return true;
  }

//-----------------------------------------------------------------------------
  bool set_pos_c_mod(mavros_msgs::SkyeCMode::Request &req,
                     mavros_msgs::SkyeCMode::Response &res) {

    bool send_new_mode = false;
    std::string str_msg = "";

    // check mode is valid in according to SKYE_POS_C_MOD in skye.h
    switch (req.mode) {
      case SKYE_POS_C_MOD_MANUAL:
        str_msg = "[skye_helper_services]: POS_C_MOD switched to SKYE_POS_C_MOD_MANUAL";
        send_new_mode = true;
        break;

      case SKYE_POS_C_MOD_CASCADE_PID:
        str_msg = "[skye_helper_services]: POS_C_MOD switched to SKYE_POS_C_MOD_CASCADE_PID";
        send_new_mode = true;
        break;

      default:
        str_msg = "[skye_helper_services]: specified mode in POS_C_MOD request is not valid";
        send_new_mode = false;
        break;
    }

    if (send_new_mode) {
      set_parameter(uas, "POS_C_MOD", req.mode);
    }

    ROS_INFO_STREAM(str_msg);

    res.success = send_new_mode;

    return true;
  }

//-----------------------------------------------------------------------------
  bool set_skye_param(mavros_msgs::ParamSet::Request &req,
                      mavros_msgs::ParamSet::Response &res) {

    if (req.value.integer != 0) {
      int param = req.value.integer;
      set_parameter(uas, req.param_id, param);
    } else {
      float param = req.value.real;
      set_parameter(uas, req.param_id, param);
    }

    res.success = true;

    return true;
  }

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeHelperServicesPlugin, mavplugin::MavRosPlugin)

