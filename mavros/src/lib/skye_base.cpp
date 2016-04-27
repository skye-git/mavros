/*
 * Copyright 2016 Marco Tranzatto, AEROTAIN, Zurich
 * All rights reserved.
 *
 */

#include <ros/console.h>

#include "mavros/skye_base.h"


namespace skye_base
{

SkyeBase::SkyeBase(){
  // Load parameters from yalm file
  getConfiguraionParams();

  // Service clients
  std::string service_name;

  service_name = "/skye_gz/" + hull_name_ + "/apply_wrench"; 
  apply_wrench_hull_cog_ = nh_.serviceClient<skye_ros::ApplyWrenchCogBf>(service_name);

  for(int i = 0; i < au_number_; i++){
    service_name = "/skye_gz/" + au_base_name_ + std::to_string(i+1) + "/apply_force_2D";
    apply_au_force_2d_.push_back(nh_.serviceClient<skye_ros::ApplyForce2DCogBf>(service_name));
  }
  
}

std::string SkyeBase::getImuTopicName(){

  return topic_imu_skye_;
}

bool SkyeBase::setBodyWrench(skye_ros::ApplyWrenchCogBf &wrench_srv){

  return apply_wrench_hull_cog_.call(wrench_srv);
}

bool SkyeBase::setAuForce2D(skye_ros::ApplyForce2DCogBf &force2D_srv, const int &au_index){

  ros::ServiceClient force2D_client = apply_au_force_2d_.at(au_index);
  return force2D_client.call(force2D_srv);
}

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


}