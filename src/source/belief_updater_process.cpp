#include "belief_updater_process.h"

void BeliefUpdaterProcess::ownSetUp() {
  n.param<std::string>("aruco_abservation_topic", aruco_topic, "arucoObservation");
  n.param<std::string>("pose_topic", pose_topic, "EstimatedPose_droneGMR_wrt_GFF");
  n.param<std::string>("battery_topic", battery_topic, "battery");
}

void BeliefUpdaterProcess::ownStart() {
  aruco_subscriber = n.subscribe(aruco_topic, 1, &BeliefUpdaterProcess::arucoCallback, this);
  pose_subscriber = n.subscribe(pose_topic, 1, &BeliefUpdaterProcess::poseCallback, this);
  battery_subscriber = n.subscribe(battery_topic, 1, &BeliefUpdaterProcess::batteryCallback, this);

  add_client = n.serviceClient<droneMsgsROS::beliefList>("add_beliefs");
  remove_client = n.serviceClient<droneMsgsROS::beliefList>("remove_beliefs");
  query_client = n.serviceClient<droneMsgsROS::executeQuery>("execute_query");
}

void BeliefUpdaterProcess::ownStop() {
  aruco_subscriber.shutdown();
  pose_subscriber.shutdown();
  battery_subscriber.shutdown();
}

void BeliefUpdaterProcess::ownRun() {}




void BeliefUpdaterProcess::arucoCallback(const droneMsgsROS::obsVector& obs_vector) {
  for(auto obs: obs_vector.obs) {
    // Update visibility
    if(aruco_times_seen[obs.id] == REQUIRED_MESSAGES && !aruco_added[obs.id]) {
      sendArucoVisibility(obs.id, true);

      // Later we'll subtract 1 to all, visible or not visible
      aruco_times_seen[obs.id] += 1;
      aruco_added[obs.id] = true;
    } else if(aruco_times_seen[obs.id] < REQUIRED_MESSAGES) {
      // We add 2 since later we'll subtract 1 to all, visible or not visible
      aruco_times_seen[obs.id] += 2;
    }

    // Update position
    Point aruco_pt(obs.x, obs.y, obs.z);
    aruco_pt.roundTo(ARUCO_MIN_DISTANCE);
    if(aruco_pt.maxDifference(aruco_positions[obs.id]) > 0) {
      bool success = sendArucoPose(obs.id, aruco_pt);

      if(success) {
        aruco_positions[obs.id] = aruco_pt;
      }
    }
  }

  // Remove visibility if needed
  for(auto times: aruco_times_seen) {
    if(times.second == 0 && aruco_added[times.first]) {
      sendArucoVisibility(times.first, false);
      aruco_added[times.first] = false;
    } else if(times.second > 0) {
      aruco_times_seen[times.first] -= 1;
    }
  }
}

void BeliefUpdaterProcess::poseCallback(const droneMsgsROS::dronePose& pose) {
  std::string new_flight_state;
  if(pose.z < 0.1) {
    new_flight_state = "LANDED";
  } else {
    new_flight_state = "FLYING";
  }

  if(current_flight_state != new_flight_state) {
    bool success = sendFlightState(new_flight_state);

    if(success) {
      current_flight_state = new_flight_state;
    }
  }

  Point new_pose(pose.x, pose.y, pose.z);
  new_pose.roundTo(POSE_MIN_DISTANCE);
  if(current_pose.maxDifference(new_pose) > 0) {
    bool success = sendPose(new_pose);

    if(success) {
      current_pose = new_pose;
    }
  }
}

void BeliefUpdaterProcess::batteryCallback(const droneMsgsROS::battery& battery) {
  std::string new_battery_level;
  if(battery.batteryPercent < BATTERY_LOW_THRESHOLD) {
    new_battery_level = "LOW";
  } else if(battery.batteryPercent < BATTERY_MEDIUM_THRESHOLD) {
    new_battery_level = "MEDIUM";
  } else {
    new_battery_level = "HIGH";
  }

  if(new_battery_level != current_battery_level) {
    bool success = sendBatteryLevel(new_battery_level);

    if(success) {
      current_battery_level = new_battery_level;
    }
  }
}




bool BeliefUpdaterProcess::sendFlightState(std::string flight_state) {
  droneMsgsROS::beliefList::Request req;
  droneMsgsROS::beliefList::Response res;

  std::stringstream ss;
  ss << "flight_state(self, " << flight_state << ")";
  req.belief_list = ss.str();
  req.multivalued = false;

  add_client.call(req, res);

  return res.success;
}

bool BeliefUpdaterProcess::sendPose(Point pose) {
  droneMsgsROS::beliefList::Request req;
  droneMsgsROS::beliefList::Response res;

  std::stringstream ss;
  ss << "position(self, (" << pose.x << ", " << pose.y << ", " << pose.z << "))";
  req.belief_list = ss.str();
  req.multivalued = false;

  add_client.call(req, res);

  return res.success;
}

bool BeliefUpdaterProcess::sendArucoPose(int id, Point pose) {
  droneMsgsROS::beliefList::Request req;
  droneMsgsROS::beliefList::Response res;

  std::stringstream ss;
  ss << "position(aruco_" << id << ", (" << pose.x << ", " << pose.y << ", " << pose.z << "))";
  req.belief_list = ss.str();
  req.multivalued = false;

  add_client.call(req, res);

  return res.success;
}

bool BeliefUpdaterProcess::sendArucoVisibility(int id, bool visible) {
  if(visible) {
    droneMsgsROS::beliefList::Request req;
    droneMsgsROS::beliefList::Response res;

    std::stringstream ss;
    ss << "visible(aruco_" << id << ")";
    req.belief_list = ss.str();
    req.multivalued = true;

    add_client.call(req, res);

    return res.success;
  } else {
    droneMsgsROS::beliefList::Request req;
    droneMsgsROS::beliefList::Response res;

    std::stringstream ss;
    ss << "visible(aruco_" << id << ")";
    req.belief_list = ss.str();

    remove_client.call(req, res);

    return res.success;
  }
}

bool BeliefUpdaterProcess::sendBatteryLevel(std::string level) {
  droneMsgsROS::beliefList::Request req;
  droneMsgsROS::beliefList::Response res;

  std::stringstream ss;
  ss << "battery_level(self, " << level << ")";
  req.belief_list = ss.str();
  req.multivalued = false;

  add_client.call(req, res);

  return res.success;
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

void BeliefUpdaterProcess::Point::roundTo(double value) {
  x = std::round(x/value)*value;
  y = std::round(y/value)*value;
  z = std::round(z/value)*value;
}
