#include "belief_updater_process.h"

void BeliefUpdaterProcess::ownStart() {
  aruco_subscriber = n.subscribe(aruco_topic, 1, &BeliefUpdaterProcess::arucoCallback, this);

  add_client = n.serviceClient<droneMsgsROS::beliefList>("add_beliefs");
  remove_client = n.serviceClient<droneMsgsROS::beliefList>("remove_beliefs");
  query_client = n.serviceClient<droneMsgsROS::executeQuery>("execute_query");
}

void BeliefUpdaterProcess::ownSetUp() {
  ros::param::get("~aruco_abservation_topic", aruco_topic);
  if(aruco_topic.length() == 0) {
    aruco_topic = "arucoObservation";
  }
}

void BeliefUpdaterProcess::ownStop() {
  aruco_subscriber.shutdown();
}

void BeliefUpdaterProcess::ownRun() {}

void BeliefUpdaterProcess::arucoCallback(const droneMsgsROS::obsVector& obs_vector) {
  for(auto obs: obs_vector.obs) {
    // Update visibility
    if(aruco_times_seen[obs.id] == REQUIRED_MESSAGES && !aruco_added[obs.id]) {
      droneMsgsROS::beliefList::Request req;
      droneMsgsROS::beliefList::Response res;

      std::stringstream ss;
      ss << "visible(aruco_" << obs.id << ")";
      req.belief_list = ss.str();
      req.multivalued = true;

      add_client.call(req, res);

      // Later we'll subtract 1 to all, visible or not visible
      aruco_times_seen[obs.id] += 1;
      aruco_added[obs.id] = true;
    } else if(aruco_times_seen[obs.id] < REQUIRED_MESSAGES) {
      // We add 2 since later we'll subtract 1 to all, visible or not visible
      aruco_times_seen[obs.id] += 2;
    }

    // Update position
    Point aruco_pt(obs.x, obs.y, obs.z);
    if(aruco_pt.maxDifference(aruco_positions[obs.id]) > MIN_ARUCO_DISTANCE) {
      droneMsgsROS::beliefList::Request req;
      droneMsgsROS::beliefList::Response res;

      std::stringstream ss;
      ss << "position(aruco_" << obs.id << ", (" << obs.x << ", " << obs.y << ", " << obs.z << "))";
      req.belief_list = ss.str();
      req.multivalued = false;

      add_client.call(req, res);

      if(res.success) {
        aruco_positions[obs.id] = aruco_pt;
      }
    }
  }

  // Remove visibility if needed
  for(auto times: aruco_times_seen) {
    if(times.second == 0 && aruco_added[times.first]) {
      droneMsgsROS::beliefList::Request req;
      droneMsgsROS::beliefList::Response res;

      std::stringstream ss;
      ss << "visible(aruco_" << times.first << ")";
      req.belief_list = ss.str();

      remove_client.call(req, res);
      aruco_added[times.first] = false;
    } else if(times.second > 0) {
      aruco_times_seen[times.first] -= 1;
    }
  }
}



BeliefUpdaterProcess::Point::Point(double x_coord, double y_coord, double z_coord) {
  x = x_coord;
  y = y_coord;
  z = z_coord;
}

BeliefUpdaterProcess::Point::Point() {
  x = 0;
  y = 0;
  z = 0;
}

double BeliefUpdaterProcess::Point::maxDifference(BeliefUpdaterProcess::Point p) {
  double x_diff = (x - p.x) > 0? x - p.x: p.x - x;
  double y_diff = (y - p.y) > 0? y - p.y: p.y - y;
  double z_diff = (z - p.z) > 0? z - p.z: p.z - z;

  double max = 0;
  for(double d: {x_diff, y_diff, z_diff}) {
    if(d > max) {
      max = d;
    }
  }

  return max;
}
