#ifndef BELIEF_UPDATER_PROCESS_H
#define BELIEF_UPDATER_PROCESS_H

#include "drone_process.h"
#include <string.h>
#include "droneMsgsROS/Observation3D.h"
#include "droneMsgsROS/executeQuery.h"
#include "droneMsgsROS/beliefList.h"
#include "droneMsgsROS/obsVector.h"
#include "droneMsgsROS/dronePose.h"
#include "droneMsgsROS/battery.h"

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

  ros::ServiceClient add_client;
  ros::ServiceClient remove_client;
  ros::ServiceClient query_client;

  std::string aruco_topic;
  std::string pose_topic;
  std::string battery_topic;

  void arucoCallback(const droneMsgsROS::obsVector& obs);
  void poseCallback(const droneMsgsROS::dronePose& pose);
  void batteryCallback(const droneMsgsROS::battery& battery);

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



  const int REQUIRED_MESSAGES = 5;
  const double ARUCO_MIN_DISTANCE = 0.5;
  const double POSE_MIN_DISTANCE = 0.1;
  const double BATTERY_LOW_THRESHOLD = 25;
  const double BATTERY_MEDIUM_THRESHOLD = 75;
};


#endif
