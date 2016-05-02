/*
 * Copyright 2016 Marco Tranzatto, AEROTAIN, Zurich
 * All rights reserved.
 *
 */

#ifndef SKYE_BASE_H
#define SKYE_BASE_H


#include <ros/ros.h>
#include <string.h>

#include "skye_ros/ApplyWrenchCogBf.h"
#include "skye_ros/ApplyForce2DCogBf.h"

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
  bool setBodyWrench(skye_ros::ApplyWrenchCogBf &wrench_srv);
  /**
   * @brief Set 2D force in the specified AU number: 0 to au_number in yalm file.
   */
  bool setAuForce2D(skye_ros::ApplyForce2DCogBf &force2D_srv, const int &au_index);
  /**
   * @brief Check if body wrench service is available.
   */
  bool isBodyWrenchAvail();
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

protected:
  /**
   * @brief Load configuration parameters from yaml file.
   */
  void getConfiguraionParams();

  ros::NodeHandle nh_;
  // parameters
  std::string topic_imu_skye_; // topic of IMU data in Skye's IMU frame
  std::string au_base_name_;   // base name of AU used to form complete name when calling service
  std::string hull_name_;     // name of the hull in Gazebo used to form complete name when calling service
  std::string apply_wrench_service_name_;   
  std::vector<std::string> apply_au_force_service_name_; 
  int au_number_;
  // service client
  ros::ServiceClient apply_wrench_hull_cog_; // server to apply a wrench expressed in Skye's body frame 
  std::vector<ros::ServiceClient> apply_au_force_2d_; // server vector to apply 2D forces to AUs
};


} // end namespace skye_base

#endif // SKYE_BASE_H
