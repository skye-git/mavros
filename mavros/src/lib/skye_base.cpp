/*
 * Copyright 2016 Marco Tranzatto, AEROTAIN, Zurich
 * All rights reserved.
 *
 */

#include <ros/console.h>
#include <Eigen/Geometry>

#include "mavros/skye_base.h"


namespace skye_base
{

//-----------------------------------------------------------------------------
SkyeBase::SkyeBase()
  :nh_("~"),
  use_allocator_output_(0)
{
  // Load parameters from yalm file
  getConfiguraionParams();

  // Service clients
  apply_wrench_service_name_ = "/skye_gz/" + hull_name_ + "/apply_wrench"; 
//  apply_wrench_hull_cog_ = nh_.serviceClient<skye_ros::ApplyWrenchCogBf>(apply_wrench_service_name_);//TODO fix me

  for(int i = 0; i < au_number_; i++){
    apply_au_force_service_name_.push_back("/skye_gz/" + au_base_name_ + std::to_string(i+1) + "/apply_force_2D");
//    apply_au_force_2d_.push_back(nh_.serviceClient<skye_ros::ApplyForce2DCogBf>(apply_au_force_service_name_[i]));//TODO fix me
  }

  apply_force_service_name_ = "/skye_gz/" + hull_name_ + "/apply_force";
//  apply_force_hull_cog_ = nh_.serviceClient<skye_ros::ApplyForceBf>(apply_force_service_name_);//TODO fix me

  apply_torque_service_name_ = "/skye_gz/" + hull_name_ + "/apply_torque";
//  apply_torque_hull_cog_ = nh_.serviceClient<skye_ros::ApplyTorqueBf>(apply_torque_service_name_);//TODO fix me
  
}

//-----------------------------------------------------------------------------
std::string SkyeBase::getImuTopicName(){
  return topic_imu_skye_;
}

//-----------------------------------------------------------------------------
//bool SkyeBase::setBodyWrench(skye_ros::ApplyWrenchCogBf &wrench_srv){
//  // check service is available
//  if(isBodyWrenchAvail()){ // check if service is available
//    //check if we can use it
//    if(!use_allocator_output_)
//      return apply_wrench_hull_cog_.call(wrench_srv);
//  }

//  return false; // service not available
//}//TODO fix me

//-----------------------------------------------------------------------------
//bool SkyeBase::setBodyForce(skye_ros::ApplyForceBf &force_srv){
//  // check service is available
//  if(isBodyForceAvail()){ // check if service is available
//    //check if we can use it
//    if(!use_allocator_output_)
//      return apply_force_hull_cog_.call(force_srv);
//  }

//  return false; // service not available
//}//TODO fix me

//-----------------------------------------------------------------------------
//bool SkyeBase::setBodyTorque(skye_ros::ApplyTorqueBf &torque_srv){
//  // check service is available
//  if(isBodyTorqueAvail()) // check if service is available
//    return apply_torque_hull_cog_.call(torque_srv);
//  else
//    return false; // service not available
//}//TODO fix me

//-----------------------------------------------------------------------------
//bool SkyeBase::setAuForce2D(skye_ros::ApplyForce2DCogBf &force2D_srv, const int &au_index){
//  // check service is available & index < number of AUs
//  if(isAuForce2DAvail(au_index) && au_index < au_number_){ // check if service is available
//    ros::ServiceClient force2D_client = apply_au_force_2d_.at(au_index);

//    //check if we can use it
//    if(use_allocator_output_)
//      return force2D_client.call(force2D_srv);
//  }

//  return false; // service not available
//}//TODO fix me

//-----------------------------------------------------------------------------
bool SkyeBase::isBodyWrenchAvail(){
  return ros::service::exists(apply_wrench_service_name_, false);
}

//-----------------------------------------------------------------------------
bool SkyeBase::isBodyForceAvail(){
  return ros::service::exists(apply_force_service_name_, false);
}

//-----------------------------------------------------------------------------
bool SkyeBase::isBodyTorqueAvail(){
  return ros::service::exists(apply_torque_service_name_, false);
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
bool SkyeBase::useAllocatorOutput(){
  return use_allocator_output_;
}

//-----------------------------------------------------------------------------
void SkyeBase::computeWrenchCenterAUs(int au_index, Eigen::Ref<const Eigen::Matrix<double,3,1> > f_au,
                                      Eigen::Ref<Eigen::Matrix<double,6,1> > w_ned){


  Eigen::Matrix<double,3,1> f_ned; // force in NED frame
  Eigen::Matrix<double,3,1> t_ned; // torque in NED frame

  // use rotation matrix from AU frame to NED frame to compute force in NED
  f_ned = vector_R_S_P.at(au_index) * f_au;
  // torque in ned = offset_au_i from center AUs cross product force in NED
  t_ned = vector_P_S_P.at(au_index).cross(f_ned);

  w_ned.topRows(3) = f_ned;
  w_ned.bottomRows(3) = t_ned;
}

//-----------------------------------------------------------------------------
void SkyeBase::getConfiguraionParams()
{
  bool complete_list_params = true;

  complete_list_params &= nh_.getParam("skye/topic_imu_skye", topic_imu_skye_);
  complete_list_params &= nh_.getParam("skye/au_base_name_gazebo", au_base_name_);
  //complete_list_params &= nh_.getParam("skye/au_number", au_number_);
  complete_list_params &= nh_.getParam("skye/hull_name_gz", hull_name_);
  nh_.param<bool>("use_allocator_output", use_allocator_output_, false);
  nh_.param<std::string>("model_name", gazebo_model_name_, "tetra");

  //load AUs configuration based on the number of AUs
  if(gazebo_model_name_ == "enterprise_8aus")
    loadEnterpriseConf8Aus();
  else if(gazebo_model_name_ == "enterprise_6aus")
    loadEnterpriseConf6Aus();
  else if(gazebo_model_name_ == "enterprise_4aus")
    loadEnterpriseConf4Aus();
  else
    loadUnkownAusConfiguration();

  if(!complete_list_params)
    ROS_DEBUG("Parameter(s) missing in yaml file.");

  if(use_allocator_output_)
    ROS_INFO("********** [skye_base] Use allocator output to drive Skye with %d AUs", getAuNumber());
  else
    ROS_INFO("********** [skye_base] Use directly Attitude and Position controller outputs to drive Skye");
  
}

//-----------------------------------------------------------------------------
int SkyeBase::getAuNumber(){
    return au_number_;
}

//-----------------------------------------------------------------------------
void SkyeBase::loadEnterpriseConf8Aus(){

  //TODO delete me, dubug printing
  ROS_INFO("+++++ [skye_base] loadEnterpriseConf8Aus");

  au_number_ = 8;
  // ------------------------------- Past here MATLAB generated code ----------
  Eigen::Matrix<double,3,3> R_s_p;
  Eigen::Matrix<double,3,1> P_s_p;

  // ----- AU 1: Yaw: 0.00 Pitch: -90.00 Roll: -0.00 [deg]
  R_s_p << 	0.000000,0.000000,-1.000000,
                          0.000000,1.000000,0.000000,
                          1.000000,-0.000000,0.000000;
  //includes propeller height considered when generating lagrangina matrix
  P_s_p << 	2.605000,-0.000000,-0.000000;
  vector_R_S_P.push_back(R_s_p);
  vector_P_S_P.push_back(P_s_p);

  // ----- AU 2: Yaw: 0.00 Pitch: -90.00 Roll: -45.00 [deg]
  R_s_p << 	0.000000,0.707107,-0.707107,
                          0.000000,0.707107,0.707107,
                          1.000000,-0.000000,0.000000;
  //includes propeller height considered when generating lagrangina matrix
  P_s_p << 	1.842013,-1.842013,-0.000000;
  vector_R_S_P.push_back(R_s_p);
  vector_P_S_P.push_back(P_s_p);

  // ----- AU 3: Yaw: 0.00 Pitch: -90.00 Roll: -90.00 [deg]
  R_s_p << 	0.000000,1.000000,-0.000000,
                          0.000000,0.000000,1.000000,
                          1.000000,-0.000000,0.000000;
  //includes propeller height considered when generating lagrangina matrix
  P_s_p << 	0.000000,-2.605000,-0.000000;
  vector_R_S_P.push_back(R_s_p);
  vector_P_S_P.push_back(P_s_p);

  // ----- AU 4: Yaw: 0.00 Pitch: -90.00 Roll: -135.00 [deg]
  R_s_p << 	0.000000,0.707107,0.707107,
                          0.000000,-0.707107,0.707107,
                          1.000000,0.000000,0.000000;
  //includes propeller height considered when generating lagrangina matrix
  P_s_p << 	-1.842013,-1.842013,-0.000000;
  vector_R_S_P.push_back(R_s_p);
  vector_P_S_P.push_back(P_s_p);

  // ----- AU 5: Yaw: 0.00 Pitch: -90.00 Roll: -180.00 [deg]
  R_s_p << 	0.000000,0.000000,1.000000,
                          0.000000,-1.000000,0.000000,
                          1.000000,0.000000,0.000000;
  //includes propeller height considered when generating lagrangina matrix
  P_s_p << 	-2.605000,-0.000000,-0.000000;
  vector_R_S_P.push_back(R_s_p);
  vector_P_S_P.push_back(P_s_p);

  // ----- AU 6: Yaw: 0.00 Pitch: -90.00 Roll: 135.00 [deg]
  R_s_p << 	0.000000,-0.707107,0.707107,
                          0.000000,-0.707107,-0.707107,
                          1.000000,0.000000,0.000000;
  //includes propeller height considered when generating lagrangina matrix
  P_s_p << 	-1.842013,1.842013,-0.000000;
  vector_R_S_P.push_back(R_s_p);
  vector_P_S_P.push_back(P_s_p);

  // ----- AU 7: Yaw: 0.00 Pitch: -90.00 Roll: 90.00 [deg]
  R_s_p << 	0.000000,-1.000000,0.000000,
                          0.000000,-0.000000,-1.000000,
                          1.000000,0.000000,0.000000;
  //includes propeller height considered when generating lagrangina matrix
  P_s_p << 	-0.000000,2.605000,-0.000000;
  vector_R_S_P.push_back(R_s_p);
  vector_P_S_P.push_back(P_s_p);

  // ----- AU 8: Yaw: 0.00 Pitch: -90.00 Roll: 45.00 [deg]
  R_s_p << 	0.000000,-0.707107,-0.707107,
                          0.000000,0.707107,-0.707107,
                          1.000000,0.000000,0.000000;
  //includes propeller height considered when generating lagrangina matrix
  P_s_p << 	1.842013,1.842013,-0.000000;
  vector_R_S_P.push_back(R_s_p);
  vector_P_S_P.push_back(P_s_p);


  // ------------------------------- End MATLAB generated code ----------
}

//-----------------------------------------------------------------------------
void SkyeBase::loadEnterpriseConf6Aus(){

  //TODO delete me, dubug printing
  ROS_INFO("+++++ [skye_base] loadEnterpriseConf6Aus");

  au_number_ = 6;
  // ------------------------------- Past here MATLAB generated code ----------
  Eigen::Matrix<double,3,3> R_s_p;
  Eigen::Matrix<double,3,1> P_s_p;

  // ----- AU 1: Yaw: 0.00 Pitch: -90.00 Roll: -0.00 [deg]
  R_s_p << 	0.000000,0.000000,-1.000000,
                          0.000000,1.000000,0.000000,
                          1.000000,-0.000000,0.000000;
  //includes propeller height considered when generating lagrangina matrix
  P_s_p << 	2.605000,-0.000000,-0.000000;
  vector_R_S_P.push_back(R_s_p);
  vector_P_S_P.push_back(P_s_p);

  // ----- AU 2: Yaw: 0.00 Pitch: -90.00 Roll: -45.00 [deg]
  R_s_p << 	0.000000,0.707107,-0.707107,
                          0.000000,0.707107,0.707107,
                          1.000000,-0.000000,0.000000;
  //includes propeller height considered when generating lagrangina matrix
  P_s_p << 	1.842013,-1.842013,-0.000000;
  vector_R_S_P.push_back(R_s_p);
  vector_P_S_P.push_back(P_s_p);

  // ----- AU 3: Yaw: 0.00 Pitch: -90.00 Roll: -135.00 [deg]
  R_s_p << 	0.000000,0.707107,0.707107,
                          0.000000,-0.707107,0.707107,
                          1.000000,0.000000,0.000000;
  //includes propeller height considered when generating lagrangina matrix
  P_s_p << 	-1.842013,-1.842013,-0.000000;
  vector_R_S_P.push_back(R_s_p);
  vector_P_S_P.push_back(P_s_p);

  // ----- AU 4: Yaw: 0.00 Pitch: -90.00 Roll: -180.00 [deg]
  R_s_p << 	0.000000,0.000000,1.000000,
                          0.000000,-1.000000,0.000000,
                          1.000000,0.000000,0.000000;
  //includes propeller height considered when generating lagrangina matrix
  P_s_p << 	-2.605000,-0.000000,-0.000000;
  vector_R_S_P.push_back(R_s_p);
  vector_P_S_P.push_back(P_s_p);

  // ----- AU 5: Yaw: 0.00 Pitch: -90.00 Roll: 135.00 [deg]
  R_s_p << 	0.000000,-0.707107,0.707107,
                          0.000000,-0.707107,-0.707107,
                          1.000000,0.000000,0.000000;
  //includes propeller height considered when generating lagrangina matrix
  P_s_p << 	-1.842013,1.842013,-0.000000;
  vector_R_S_P.push_back(R_s_p);
  vector_P_S_P.push_back(P_s_p);

  // ----- AU 6: Yaw: 0.00 Pitch: -90.00 Roll: 45.00 [deg]
  R_s_p << 	0.000000,-0.707107,-0.707107,
                          0.000000,0.707107,-0.707107,
                          1.000000,0.000000,0.000000;
  //includes propeller height considered when generating lagrangina matrix
  P_s_p << 	1.842013,1.842013,-0.000000;
  vector_R_S_P.push_back(R_s_p);
  vector_P_S_P.push_back(P_s_p);


  // ------------------------------- End MATLAB generated code ----------
}

//-----------------------------------------------------------------------------
void SkyeBase::loadEnterpriseConf4Aus(){

  //TODO delete me, dubug printing
  ROS_INFO("+++++ [skye_base] loadEnterpriseConf4Aus");

  au_number_ = 4;

  // ------------------------------- Past here MATLAB generated code ----------
  Eigen::Matrix<double,3,3> R_s_p;
  Eigen::Matrix<double,3,1> P_s_p;

  // ----- AU 1: Yaw: 0.00 Pitch: -90.00 Roll: -0.00 [deg]
  R_s_p << 	0.000000,0.000000,-1.000000,
                          0.000000,1.000000,0.000000,
                          1.000000,-0.000000,0.000000;
  //includes propeller height considered when generating lagrangina matrix
  P_s_p << 	2.605000,-0.000000,-0.000000;
  vector_R_S_P.push_back(R_s_p);
  vector_P_S_P.push_back(P_s_p);

  // ----- AU 2: Yaw: 0.00 Pitch: -90.00 Roll: -90.00 [deg]
  R_s_p << 	0.000000,1.000000,-0.000000,
                          0.000000,0.000000,1.000000,
                          1.000000,-0.000000,0.000000;
  //includes propeller height considered when generating lagrangina matrix
  P_s_p << 	0.000000,-2.605000,-0.000000;
  vector_R_S_P.push_back(R_s_p);
  vector_P_S_P.push_back(P_s_p);

  // ----- AU 3: Yaw: 0.00 Pitch: -90.00 Roll: -180.00 [deg]
  R_s_p << 	0.000000,0.000000,1.000000,
                          0.000000,-1.000000,0.000000,
                          1.000000,0.000000,0.000000;
  //includes propeller height considered when generating lagrangina matrix
  P_s_p << 	-2.605000,-0.000000,-0.000000;
  vector_R_S_P.push_back(R_s_p);
  vector_P_S_P.push_back(P_s_p);

  // ----- AU 4: Yaw: 0.00 Pitch: -90.00 Roll: 90.00 [deg]
  R_s_p << 	0.000000,-1.000000,0.000000,
                          0.000000,-0.000000,-1.000000,
                          1.000000,0.000000,0.000000;
  //includes propeller height considered when generating lagrangina matrix
  P_s_p << 	-0.000000,2.605000,-0.000000;
  vector_R_S_P.push_back(R_s_p);
  vector_P_S_P.push_back(P_s_p);
  // ------------------------------- End MATLAB generated code ----------
}

//-----------------------------------------------------------------------------
void SkyeBase::loadUnkownAusConfiguration(){

  au_number_ = 0;

  ROS_ERROR_STREAM(__FILE__ << ": (" << __func__ <<
                   ") configuration for " << gazebo_model_name_ <<" not found \n" <<
                   "Please add a valid configuration in " << __FILE__ << " and compile Mavros again.");

  ros::shutdown();
}

} // namespace skye_base
