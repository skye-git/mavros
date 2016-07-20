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
 * @file skye_common_helpers.cpp
 * Common shared functions among Skye's plugins
 *
 * @author Marco Tranzatto <marco@skye.aero>
 */

#include "mavros/skye_common_helpers.h"


//-----------------------------------------------------------------------------
void set_parameter(mavros::UAS *uas, std::string param_name, int param_value){

  mavlink_message_t msg;
  float *float_var = (float*)(&param_value);//todo find a better solution to this workaround
  char c_buffer[16]; // 16 is the maximum length of mavlink param name
  strcpy(c_buffer, param_name.c_str());

  mavlink_msg_param_set_pack_chan(UAS_PACK_CHAN(uas), &msg,
                                  uas->get_tgt_system(),
                                  (uint8_t)MAV_COMP_ID_ALL,
                                  c_buffer,
                                  *float_var,
                                  (uint8_t)MAV_PARAM_TYPE_INT32);
  UAS_FCU(uas)->send_message(&msg);
}
