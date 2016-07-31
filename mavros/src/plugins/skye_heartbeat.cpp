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
 * @file skye_heartbeat.cpp
 * Mavros plugin to read mavlink "heartbeat" message from the FMU.
 *
 * @author Marco Tranzatto <marco@skye.aero>
 */
#include <mavros/mavros_plugin.h>

#include <pluginlib/class_list_macros.h>
#include <mavros/skye_common_helpers.h>

static const std::string kMyNameWhenPrintg("skye_heartbeat");

namespace mavplugin {

class SkyeHeartbeatPlugin : public MavRosPlugin {
public:
//-----------------------------------------------------------------------------
  SkyeHeartbeatPlugin() :
    nh("~"),
    uas(nullptr),
    received_first_heartbit(false)
  { };

//-----------------------------------------------------------------------------
  ~SkyeHeartbeatPlugin(){
    // disable HIL mode before exiting
    set_hil_mode(false);
    // set allocation case to -1 before exiting
    set_allocation_case(-1);
  }

//-----------------------------------------------------------------------------
  /**
   * Plugin initializer. Constructor should not do this.
   */
  void initialize(UAS &uas_)
  {
    uas = &uas_;

  }

//-----------------------------------------------------------------------------
  /**
   * This function returns message<->handler mapping
   *
   * Each entry defined by @a MESSAGE_HANDLER() macro
   */
  const message_map get_rx_handlers() {
    return {
      MESSAGE_HANDLER(MAVLINK_MSG_ID_HEARTBEAT,
                      &SkyeHeartbeatPlugin::handle_heartbeat)
    };
  }

private:
//-----------------------------------------------------------------------------
  ros::NodeHandle nh;
  UAS *uas;
  bool received_first_heartbit;

//-----------------------------------------------------------------------------
  /**
   * This function gets call when we want to set HIL mode in the FMU
   */
  void set_hil_mode(bool hil_on) {
    if(hil_on){
        set_parameter(uas, "SKYE_HIL_MODE", 1);
        ROS_INFO_STREAM("[" << kMyNameWhenPrintg << "] set HIL mode to 1");
      }
    else{
        set_parameter(uas, "SKYE_HIL_MODE", 0);
        ROS_INFO_STREAM("[" << kMyNameWhenPrintg << "] set HIL mode to 0");
      }

  }

//-----------------------------------------------------------------------------
  /**
   * This function gets call when we want to change allocation case in the FMU
   */
  void set_allocation_case(int allocation_case) {
    set_parameter(uas, "SKYE_AL_CASE", allocation_case);
    ROS_INFO_STREAM("[" << kMyNameWhenPrintg << "] set SKYE_AL_CASE to " << allocation_case);
  }

//-----------------------------------------------------------------------------
  /**
   * This function gets call when a new heartbeat mavlink message is received
   */
  void handle_heartbeat(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {

    // The first time we receive the hearbit msg we enable HIL mode and tell to
    // the FMU to use all AUs
    if (!received_first_heartbit) {
        // Enable HIL mode
        set_hil_mode(true);
        // Set allocation case to 0 to be able to use allocation output
        set_allocation_case(0);
        // Do not enter in this branch anymore
        received_first_heartbit = true;
    }

  }

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeHeartbeatPlugin, mavplugin::MavRosPlugin)

