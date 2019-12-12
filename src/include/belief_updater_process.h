/*!*********************************************************************************
 *  \file       belief_updater_process.h
 *  \brief      BeliefUpdaterProcess definition file.
 *  \details    This file contains the BeliefUpdaterProcess declaration. To obtain more information about
 *              it's definition consult the belief_updater_process.cpp file.
 *  \authors    Guillermo De Fermin
 *  \copyright  Copyright 2016 Universidad Politecnica de Madrid (UPM)
 *
 *     This program is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with this program. If not, see http://www.gnu.org/licenses/.
 ********************************************************************************/
#ifndef BELIEF_UPDATER_PROCESS_H
#define BELIEF_UPDATER_PROCESS_H

#include "drone_process.h"
#include <string.h>
#include "droneMsgsROS/Observation3D.h"
#include "aerostack_msgs/QueryBelief.h"
#include "aerostack_msgs/AddBelief.h"
#include "aerostack_msgs/RemoveBelief.h"
#include "droneMsgsROS/obsVector.h"
#include "droneMsgsROS/dronePose.h"
#include "droneMsgsROS/battery.h"
#include <droneMsgsROS/QRInterpretation.h>
#include "droneMsgsROS/GenerateID.h"

class BeliefUpdaterProcess: public DroneProcess {
public:
  void ownSetUp();
  void ownStart();
  void ownStop();
  void ownRun();

private:
  struct Point {
    double x, y, z;
    Point(double x, double y, double z);
    Point();
    double maxDifference(Point p);
    void roundTo(double value);
  };

  ros::NodeHandle n;

  ros::Subscriber aruco_subscriber;
  ros::Subscriber pose_subscriber;
  ros::Subscriber battery_subscriber;
  ros::Subscriber qr_interpretation_subscriber;

  ros::ServiceClient add_client;
  ros::ServiceClient remove_client;
  ros::ServiceClient query_client;
  ros::ServiceClient generate_id_client;

  std::string aruco_topic;
  std::string pose_topic;
  std::string battery_topic;
  std::string qr_interpretation_topic;

  std::string previous_interpretation;

  void arucoCallback(const droneMsgsROS::obsVector& obs);
  void poseCallback(const droneMsgsROS::dronePose& pose);
  void batteryCallback(const droneMsgsROS::battery& battery);
  void qrInterpretationCallback(const droneMsgsROS::QRInterpretation& obs_vector);

  std::map<int, Point> aruco_positions;
  std::map<int, int> aruco_times_seen;
  std::map<int, bool> aruco_added;
  Point current_pose;
  std::string current_flight_state;
  std::string current_battery_level;

  bool sendFlightState(std::string flight_state);
  bool sendPose(Point pose);
  bool sendArucoPose(int id, Point pose);
  bool sendArucoVisibility(int id, bool visible);
  bool sendBatteryLevel(std::string level);
  bool sendQRInterpretation(std::string message, bool visible);


  const int REQUIRED_MESSAGES = 5;
  const double ARUCO_MIN_DISTANCE = 0.5;
  const double POSE_MIN_DISTANCE = 0.1;
  const double BATTERY_LOW_THRESHOLD = 25;
  const double BATTERY_MEDIUM_THRESHOLD = 75;

  int my_id;
 
};


#endif
