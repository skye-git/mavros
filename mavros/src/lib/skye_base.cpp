/*
 * Copyright 2016 Marco Tranzatto, AEROTAIN, Zurich
 * All rights reserved.
 *
 */

#include <ros/console.h>

#include "mavros/skye_base.h"


namespace skye_base
{

//-----------------------------------------------------------------------------
SkyeBase::SkyeBase() : nh_("~") {
  // Load parameters from yalm file
  getConfiguraionParams();

  // Service clients
  apply_wrench_service_name_ = "/skye_gz/" + hull_name_ + "/apply_wrench"; 
  apply_wrench_hull_cog_ = nh_.serviceClient<skye_ros::ApplyWrenchCogBf>(apply_wrench_service_name_);

  for(int i = 0; i < au_number_; i++){
    apply_au_force_service_name_.push_back("/skye_gz/" + au_base_name_ + std::to_string(i+1) + "/apply_force_2D");
    apply_au_force_2d_.push_back(nh_.serviceClient<skye_ros::ApplyForce2DCogBf>(apply_au_force_service_name_[i]));
  }
  
}

//-----------------------------------------------------------------------------
std::string SkyeBase::getImuTopicName(){
  return topic_imu_skye_;
}

//-----------------------------------------------------------------------------
bool SkyeBase::setBodyWrench(skye_ros::ApplyWrenchCogBf &wrench_srv){
  // check service is available
  if(isBodyWrenchAvail()) // check if service is available
    return apply_wrench_hull_cog_.call(wrench_srv);
  else
    return false; // service not available
}

//-----------------------------------------------------------------------------
bool SkyeBase::setAuForce2D(skye_ros::ApplyForce2DCogBf &force2D_srv, const int &au_index){
  // check service is available & index < number of AUs
  if(isAuForce2DAvail(au_index) && au_index < au_number_){ // check if service is available
    ros::ServiceClient force2D_client = apply_au_force_2d_.at(au_index);
    return force2D_client.call(force2D_srv);
  }
  else
    return false; // service not available
                
}

//-----------------------------------------------------------------------------
bool SkyeBase::isBodyWrenchAvail(){
  return ros::service::exists(apply_wrench_service_name_, false);
}

//-----------------------------------------------------------------------------
bool SkyeBase::isAuForce2DAvail(){
  bool all_available = true;

  for(int i = 0; i < au_number_; i++) {
    all_available &= isAuForce2DAvail(i);
  }

  return all_available;
}

//-----------------------------------------------------------------------------
bool SkyeBase::isAuForce2DAvail(const int &au_index){
  return ros::service::exists(apply_au_force_service_name_.at(au_index), false);
}

//-----------------------------------------------------------------------------
void SkyeBase::getConfiguraionParams()
{
  bool complete_list_params = true;

  complete_list_params &= nh_.getParam("topic_imu_skye", topic_imu_skye_);
  complete_list_params &= nh_.getParam("au_base_name_gazebo", au_base_name_);
  complete_list_params &= nh_.getParam("au_number", au_number_);
  complete_list_params &= nh_.getParam("hull_name_gz", hull_name_);

  if(!complete_list_params)
    ROS_DEBUG("Parameter(s) missing in yaml file.");
  
}

//-----------------------------------------------------------------------------
int SkyeBase::getAuNumber(){
    return au_number_;
}


}
