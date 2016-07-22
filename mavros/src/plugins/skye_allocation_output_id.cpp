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
 * @file skye_allocation_output_id.cpp
 * Mavros plugin to read mavlink "skye_allocation_output_id" message from the FMU.
 *
 * @author Marco Tranzatto <marco@skye.aero>
 */
#include <mavros/mavros_plugin.h>

#include <map>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Vector3.h>
#include <skye_msgs/AllocatorOutput.h>

static const double kDegToRad = M_PI / 180.0;

namespace mavplugin {

class SkyeAllocationOutputIdPlugin : public MavRosPlugin {
public:
//-----------------------------------------------------------------------------
  SkyeAllocationOutputIdPlugin() :
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

    // Publisher allocation output
    allocator_output_pub = nh_private.advertise<skye_msgs::AllocatorOutput>("skye_px4/allocator_output", 10);

    // Check if we should apply the requested 2D force to the AUs.
    nh_private.param<bool>("use_allocator_output", use_allocator_output, false);

    // Init seq ID
    seq_id = 0;

    // Init max AU id
    max_au_id = 0;

    // We have at least one AU
    allocator_out_msg.thrust.resize(1, 0.0);
    allocator_out_msg.angle.resize(1, 0.0);
  }

//-----------------------------------------------------------------------------
  /**
   * This function returns message<->handler mapping
   *
   * Each entry defined by @a MESSAGE_HANDLER() macro
   */
  const message_map get_rx_handlers() {
    return {
      MESSAGE_HANDLER(MAVLINK_MSG_ID_ALLOCATION_OUTPUT_ID,
                      &SkyeAllocationOutputIdPlugin::handle_allocation_output_id)
    };
  }

private:
//-----------------------------------------------------------------------------
  ros::NodeHandle nh_private;
  ros::NodeHandle nh_public;
  UAS *uas;

  ros::Publisher allocator_output_pub; /*< Publisher of allocator output. */
  bool use_allocator_output; /*< Indicates if we should apply the requested torque to the hull*/
  std::map<int,ros::Publisher> map_au_pubs;  /*< Map of publishers of AUs' 2D forces. */
  skye_msgs::AllocatorOutput allocator_out_msg; /*< Collection of outputs of allocator app. */
  geometry_msgs::Vector3 au_force_2D_msg; /*< ROS message with desired 2D force*/
  unsigned int seq_id; /*< Message sequence ID. */
  uint8_t max_au_id; /*< Greatest id received */

//-----------------------------------------------------------------------------
  /**
   * This function gets call when a new allocation_output_id mavlink message is received
   */
  void handle_allocation_output_id(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {

    // Apply a 2D force to each AU based on the output of the allocator
    mavlink_allocation_output_id_t allocator_output_id;
    mavlink_msg_allocation_output_id_decode(msg, &allocator_output_id);
    const int au_id = allocator_output_id.id;
    // Fill ROS message with collection of allocator output
    allocator_out_msg.header.seq = seq_id++;
    allocator_out_msg.header.stamp = ros::Time::now();
    allocator_out_msg.header.frame_id = "0";
    // Check if we should increase the allocator output vector
    if (au_id > max_au_id) {
      max_au_id = au_id;
      // Reseive vectors in AllocatorOutput message
      uint8_t new_size = max_au_id + 1; // AU id starts from 0
      // When increasing size, fill with 0 elements not already received
      allocator_out_msg.thrust.resize(new_size, 0.0);
      allocator_out_msg.angle.resize(new_size, 0.0);
    }
    allocator_out_msg.thrust[au_id] = allocator_output_id.thrust;
    allocator_out_msg.angle[au_id] = allocator_output_id.angle;
    // Publish
    allocator_output_pub.publish(allocator_out_msg);

    if (use_allocator_output) {
      // Fill ROS message with requested 2D force for AU with ID au_id
      au_force_2D_msg.x = allocator_output_id.thrust * cos(allocator_output_id.angle * kDegToRad);
      au_force_2D_msg.y = allocator_output_id.thrust * sin(allocator_output_id.angle * kDegToRad);
      au_force_2D_msg.z = 0.0; // make sure Z componenet is 0

      // Check if we already have already created the publisher for this specific AU
      ros::Publisher au_force_2D_pub;
      auto search_au = map_au_pubs.find(au_id);

      if (search_au != map_au_pubs.end()) {
        // We already have the publisher
        au_force_2D_pub = search_au->second;
      } else {
        // It's the first time we received this AU id, advertise related topic
        std::string au_name("au_" + std::to_string(au_id + 1)); // Au names start from 1
        au_force_2D_pub = nh_public.advertise<geometry_msgs::Vector3>(
                                  "skye_gz/" + au_name + "/force_desired",
                                  10);

        // Store publisher
        map_au_pubs.emplace(au_id, au_force_2D_pub);
      }

      // Publish
      au_force_2D_pub.publish(au_force_2D_msg);
    }

  }

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SkyeAllocationOutputIdPlugin, mavplugin::MavRosPlugin)

