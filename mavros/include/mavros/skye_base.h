/*
 * Copyright 2016 Marco Tranzatto, AEROTAIN, Zurich
 * All rights reserved.
 *
 */

#ifndef SKYE_BASE_H
#define SKYE_BASE_H


#include <ros/ros.h>
#include <string.h>
#include <Eigen/Core>
#include<Eigen/StdVector>

//#include "skye_ros/ApplyWrenchCogBf.h"
//#include "skye_ros/ApplyForceBf.h"
//#include "skye_ros/ApplyTorqueBf.h"
//#include "skye_ros/ApplyForce2DCogBf.h" //TODO fix me

namespace skye_base
{

class SkyeBase{

public:
  SkyeBase();

  /**
   * @brief Get topic name of IMU data
   */
  std::string getImuTopicName();

  /**
   * @brief Set body wrench in the CoG of the hull. Wrench expressed in body frame.
   */
//  bool setBodyWrench(skye_ros::ApplyWrenchCogBf &wrench_srv); //TODO fix me

  /**
   * @brief Set body force in the CoG of the hull. Force expressed in body frame.
   */
//  bool setBodyForce(skye_ros::ApplyForceBf &force_srv); //TODO fix me

  /**
   * @brief Set body torque in the CoG of the hull. Torque expressed in body frame.
   */
//  bool setBodyTorque(skye_ros::ApplyTorqueBf &torque_srv); //TODO fix me

  /**
   * @brief Set 2D force in the specified AU number: 0 to au_number in yalm file.
   */
//  bool setAuForce2D(skye_ros::ApplyForce2DCogBf &force2D_srv, const int &au_index); //TODO fix me

  /**
   * @brief Check if body wrench service is available.
   */
  bool isBodyWrenchAvail();

  /**
   * @brief Check if body force service is available.
   */
  bool isBodyForceAvail();

  /**
   * @brief Check if body torque service is available.
   */
  bool isBodyTorqueAvail();

  /**
   * @brief Check if au force 2D service is available.
   */
  bool isAuForce2DAvail();

  /**
   * @brief Check if au force 2D service is available.
   */
  bool isAuForce2DAvail(const int &au_index);

  /**
   * @brief Get number of AUs.
   */
  int getAuNumber();

  /**
   * @brief Check if allocator output should be used instead of directly position and attitude output.
   */
  bool useAllocatorOutput();

  /**
   * @brief Compute the produced wrench of a force vector in AU frame in the center of AUs
   */
  void computeWrenchCenterAUs(int au_index, Eigen::Ref<const Eigen::Matrix<double,3,1> > f_au,
                              Eigen::Ref<Eigen::Matrix<double,6,1> > w_ned);

protected:
  /**
   * @brief Load configuration parameters from yaml file.
   */
  void getConfiguraionParams();

  /**
   * @brief Load AUs configuration - 8 AUs for enterprise
   */
  void loadEnterpriseConf8Aus();

  /**
   * @brief Load AUs configuration - 6 AUs for enterprise
   */
  void loadEnterpriseConf6Aus();

  /**
   * @brief Load AUs configuration - 4 AUs for enterprise
   */
  void loadEnterpriseConf4Aus();

  /**
   * @brief Load AUs configuration - unkown number of AUs
   */
  void loadUnkownAusConfiguration();

  ros::NodeHandle nh_;
  // parameters
  std::string topic_imu_skye_; // topic of IMU data in Skye's IMU frame
  std::string au_base_name_;   // base name of AU used to form complete name when calling service
  std::string hull_name_;     // name of the hull in Gazebo used to form complete name when calling service
  std::string apply_wrench_service_name_;   
  std::vector<std::string> apply_au_force_service_name_;
  std::string apply_force_service_name_;
  std::string apply_torque_service_name_;
  std::string gazebo_model_name_;
  int au_number_;
  bool use_allocator_output_;
  // service client
  ros::ServiceClient apply_wrench_hull_cog_; // server to apply a wrench expressed in Skye's body frame 
  ros::ServiceClient apply_force_hull_cog_; // server to apply a force expressed in Skye's body frame
  ros::ServiceClient apply_torque_hull_cog_; // server to apply a torque expressed in Skye's body frame
  std::vector<ros::ServiceClient> apply_au_force_2d_; // server vector to apply 2D forces to AUs
  //std::vector<ros::ServiceClient> apply_au_force_2d_; // server vector to apply 2D forces to AUs

  // AUs configuration
  std::vector<Eigen::Matrix<double,3,3>,Eigen::aligned_allocator<Eigen::Matrix<double,3,3>> > vector_R_S_P;
  std::vector<Eigen::Matrix<double,3,1>,Eigen::aligned_allocator<Eigen::Matrix<double,3,1>> > vector_P_S_P;
};


} // end namespace skye_base

#endif // SKYE_BASE_H
