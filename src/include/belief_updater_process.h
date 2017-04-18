#ifndef BELIEF_UPDATER_PROCESS_H
#define BELIEF_UPDATER_PROCESS_H

#include "drone_process.h"
#include <string.h>
#include "droneMsgsROS/Observation3D.h"
#include "droneMsgsROS/executeQuery.h"
#include "droneMsgsROS/beliefList.h"
#include "droneMsgsROS/obsVector.h"

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
  };

  ros::NodeHandle n;

  ros::Subscriber aruco_subscriber;

  ros::ServiceClient add_client;
  ros::ServiceClient remove_client;
  ros::ServiceClient query_client;

  std::string aruco_topic;

  void arucoCallback(const droneMsgsROS::obsVector& obs);

  std::map<int, Point> aruco_positions;
  std::map<int, int> aruco_times_seen;
  std::map<int, bool> aruco_added;

  const int REQUIRED_MESSAGES = 5;
  const double MIN_ARUCO_DISTANCE = 0.5;
};


#endif
