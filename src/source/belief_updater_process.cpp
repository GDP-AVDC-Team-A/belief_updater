#include "belief_updater_process.h"

void BeliefUpdaterProcess::ownSetUp() {
  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>("pose_topic", pose_topic, "self_localization/pose");
  private_nh.param<std::string>("battery_topic", battery_topic, "sensor_measurement/battery_state");
  private_nh.param<std::string>("message_from_robot", message_from_robot,"message_from_robot");
  private_nh.param<std::string>("drone_id_namespace", drone_id_namespace, "drone1");
  private_nh.param<std::string>("shared_robot_positions_channel_topic", shared_robot_positions_channel_str,
                                 "shared_robot_positions_channel");
}

void BeliefUpdaterProcess::ownStart() {
  pose_subscriber = n.subscribe(pose_topic, 1, &BeliefUpdaterProcess::poseCallback, this);
  battery_subscriber = n.subscribe(battery_topic, 1, &BeliefUpdaterProcess::batteryCallback, this);
  message_from_robot_sub =n.subscribe('/' + message_from_robot, 100,
                            &BeliefUpdaterProcess::message_from_robotCallback, this);
  shared_robot_positions_channel_sub =
      n.subscribe('/' + shared_robot_positions_channel_str, 1000, &BeliefUpdaterProcess::sharedRobotPositionCallback, this);
  add_client = n.serviceClient<aerostack_msgs::AddBelief>("add_belief");
  remove_client = n.serviceClient<aerostack_msgs::RemoveBelief>("remove_belief");
  query_client = n.serviceClient<aerostack_msgs::QueryBelief>("query_belief");
  generate_id_client = n.serviceClient<droneMsgsROS::GenerateID>("belief_manager_process/generate_id");

//It's needed to wait at least 2 seconds because if you don't wait it is possible that the clients are not connected.
  ros::Duration(2).sleep();
//Getting the new id for the drone
  droneMsgsROS::GenerateID::Request req;
  droneMsgsROS::GenerateID::Response res;
  generate_id_client.call(req, res);
  if (res.ack)
  {
    my_id = res.id;
  }
  aerostack_msgs::QueryBelief srv;
  srv.request.query = "object(?x,drone), name(?x,"+drone_id_namespace+"), self(?x)"; 
  query_client.call(srv);
  aerostack_msgs::QueryBelief::Response response= srv.response;
  if(response.success==false){
  	aerostack_msgs::AddBelief srv2;
  	std::stringstream s;
  	s << "object(" << my_id << ", drone), name(" << my_id << ","+drone_id_namespace+"), self(" << my_id << ")";//llamar a generate id
 	srv2.request.belief_expression = s.str();
 	srv2.request.multivalued = false;
 	add_client.call(srv2);
 }
}

void BeliefUpdaterProcess::ownStop() {
  pose_subscriber.shutdown();
  battery_subscriber.shutdown();
}

void BeliefUpdaterProcess::ownRun() {}


std::vector<std::string> BeliefUpdaterProcess::getPairs(std::string subs){
  std::vector<std::string> string_portions;
  int ini=0;
  int pos=0;
  while((pos=subs.find("\n",pos))!=std::string::npos){
    string_portions.push_back(subs.substr(ini,pos-ini));
    pos=pos+1;
    ini=pos;
  }
  //now it is going to delete spaces
  std::vector<std::string> res;
  for(int j=0;j<string_portions.size();j++){
    std::string aux="";
    for(int  i = 0; string_portions[j][i] != 0;i++){
      if(string_portions[j][i] != 32){
        aux=aux+string_portions[j][i];
      }
    }
    res.push_back(aux);
  }
return res;
}

std::vector<std::string> BeliefUpdaterProcess::getSubstitutions(std::vector<std::string> pairs){
  std::vector<std::string>res;
  for (int i=0; i<pairs.size();i++){
     res.push_back(pairs[i].substr(pairs[i].find(":")+1,pairs[i].size()-1));
  }
  return res;
}

void BeliefUpdaterProcess::message_from_robotCallback(const aerostack_msgs::SocialCommunicationStatement &message) {

  if(message.sender != drone_id_namespace && message.receiver ==drone_id_namespace){
    aerostack_msgs::QueryBelief srv;

    srv.request.query = "object(?x,drone), name(?x,"+message.sender+")"; 
    query_client.call(srv);
    aerostack_msgs::QueryBelief::Response response= srv.response;
    int id;
    if(response.success==false){
      droneMsgsROS::GenerateID::Request req;
      droneMsgsROS::GenerateID::Response res;
      generate_id_client.call(req, res);
      if (res.ack)
      {
        id = res.id;
      }   
      aerostack_msgs::AddBelief srv2;
      std::stringstream s;
      s << "object(" << id << ", drone), name(" << id << ","<< message.sender <<")";
      srv2.request.belief_expression = s.str();
      srv2.request.multivalued = true;
      add_client.call(srv2);
      aerostack_msgs::AddBelief::Response response= srv2.response;
      int id;
   }else{
    std::string sub = response.substitutions;
    std::vector<std::string> pairs = getPairs(sub);
    std::vector<std::string> subs = getSubstitutions(pairs);
    id = std::stoi(subs[0]);
   }
  YAML::Node content = YAML::Load(message.content);
  if(content["TEXT"]){
    std::string text=content["TEXT"].as<std::string>();
    droneMsgsROS::GenerateID::Request req2;
    droneMsgsROS::GenerateID::Response res2;
    generate_id_client.call(req2, res2);
    int id_message;
    if (res2.ack)
    {
      id_message = res2.id;
    }
    aerostack_msgs::AddBelief::Request req;
    aerostack_msgs::AddBelief::Response res;
    std::stringstream ss;
    ss << "object(" << id_message << ", message) , sender(" << id_message<<", " << id << "), text(" << id_message<<", "<< text <<")";
    req.belief_expression = ss.str();
    req.multivalued = false;
    add_client.call(req, res);
  }
}
}
double BeliefUpdaterProcess::get_angle(geometry_msgs::Point shared_vel, geometry_msgs::Point own_vel){
  double x1 = shared_vel.x;
  double y1 = shared_vel.y;
  double z1 = shared_vel.z;

  double x2 = own_vel.x;
  double y2 = own_vel.y;
  double z2 = own_vel.z;

  double dot = x1*x2 + y1*y2;
  double lenSq1 = x1*x1 + y1*y1;
  double lenSq2 = x2*x2 + y2*y2;
  double angle = std::acos(dot/std::sqrt(lenSq1 * lenSq2))  * 180.0 / M_PI ;
  return angle;
}

bool BeliefUpdaterProcess::collision_detected(geometry_msgs::Point shared_position , geometry_msgs::Point shared_vel 
  , geometry_msgs::Point own_position , geometry_msgs::Point own_vel){
  int times= (int)TEMPORAL_HORIZON/TIME_STEP;
  for (int i = TIME_STEP; i<=times;i=i+1){
    // shared position until TEMPORAL_HORIZON 
    geometry_msgs::Point next_shared_position;
    next_shared_position.x = shared_position.x+shared_vel.x*(TIME_STEP*i);
    next_shared_position.y = shared_position.y+shared_vel.y*(TIME_STEP*i);
    next_shared_position.z = shared_position.z+shared_vel.z*(TIME_STEP*i);
    // own position until TEMPORAL_HORIZON 
    geometry_msgs::Point next_own_position;
    next_own_position.x = own_position.x+own_vel.x*(TIME_STEP*i);
    next_own_position.y = own_position.y+own_vel.y*(TIME_STEP*i);
    next_own_position.z = own_position.z+own_vel.z*(TIME_STEP*i);

    double dist = sqrt( pow(next_shared_position.x - next_own_position.x,2.) + 
      pow(next_shared_position.y - next_own_position.y,2.) + 
      pow(next_shared_position.z - next_own_position.z,2.));
  if(dist<=COLLISION_DISTANCE){
      return true; 
    }
  }
  return false;
}

void BeliefUpdaterProcess::sharedRobotPositionCallback(
    const aerostack_msgs::SharedRobotPosition &message)
{
  if(message.sender != drone_id_namespace){
    aerostack_msgs::QueryBelief srv;
    srv.request.query = "object(?x,drone), name(?x,"+message.sender+")"; 
    query_client.call(srv);
    aerostack_msgs::QueryBelief::Response response= srv.response;
    int id;
    if(response.success==false){
      droneMsgsROS::GenerateID::Request req;
      droneMsgsROS::GenerateID::Response res;
      generate_id_client.call(req, res);
      if (res.ack)
      {
        id = res.id;
      }
      aerostack_msgs::AddBelief srv2;
      std::stringstream s;
      s << "object(" << id << ", drone), name(" << id << ","<< message.sender <<")";
      srv2.request.belief_expression = s.str();
      srv2.request.multivalued = true;
      add_client.call(srv2);
      aerostack_msgs::AddBelief::Response response= srv2.response;
   }else{
    std::string sub = response.substitutions;
    std::vector<std::string> pairs = getPairs(sub);
    std::vector<std::string> subs = getSubstitutions(pairs);
    id = std::stoi(subs[0]);
  }
  aerostack_msgs::AddBelief::Request req;
  aerostack_msgs::AddBelief::Response res;
  double val_x=message.position.x;
  val_x=val_x*100;
  val_x=std::round(val_x);
  val_x=val_x/100;  
  double val_y=message.position.y;
  val_y=val_y*100;
  val_y=std::round(val_y);
  val_y=val_y/100;  
  double val_z=message.position.z;
  val_z=val_z*100;
  val_z=std::round(val_z);
  val_z=val_z/100;  
  if(val_x>=-0.01 && val_x<=0.01){
    val_x=0;
  }
  if(val_y>=-0.01 && val_y<=0.01){
    val_y=0;
  }
  if(val_z>=-0.01 && val_z<=0.01){
    val_z=0;
  }
  std::stringstream ss;
  ss << "position(" << id << ", (" <<val_x<< ", "<< val_y<< ", " << val_z << "))";
  req.belief_expression = ss.str();
  req.multivalued = false;
  add_client.call(req, res);
  if( !(last_positions.find(id) == last_positions.end())){
    //calculate velocity
    int32_t d_time=abs(int(message.time-last_positions[id].second));
    if(d_time>0.2){
    double x_now = val_x;
    double x_previous = last_positions[id].first.x;
    double d_x= x_now-x_previous; 
    double vel_x = d_x/(double)d_time;
    double y_now = val_y;
    double y_previous = last_positions[id].first.y;
    double d_y= y_now-y_previous; 
    double vel_y = d_y/(double)d_time;
    double z_now = val_z;
    double z_previous = last_positions[id].first.z;
    double d_z= z_now-z_previous; 
    double vel_z = d_z/(double)d_time;
    geometry_msgs::Point shared_vel;
    shared_vel.x=vel_x;
    shared_vel.y=vel_y;
    shared_vel.z=vel_z;
    geometry_msgs::Point shared_position;
    shared_position.x=x_now;
    shared_position.y=y_now;
    shared_position.z=z_now;
    geometry_msgs::Point own_position;
    own_position.x=last_positions[my_id].first.x;
    own_position.y=last_positions[my_id].first.y;
    own_position.z=last_positions[my_id].first.z;
    geometry_msgs::Point own_vel=vel;
    //it is verified that there is collision between both vectors
    if(collision_detected(shared_position , shared_vel , own_position , own_vel)){
      double angle = get_angle(shared_vel, own_vel);
      if (angle>=160){
      aerostack_msgs::QueryBelief srv;
      std::stringstream s;
      s << "frontal_collision_course(" << my_id << ", "<< id <<")";
      srv.request.query = s.str();
      query_client.call(srv);
      aerostack_msgs::QueryBelief::Response response= srv.response;
      if(!response.success){
        std::cout<<"collision detected";
        aerostack_msgs::AddBelief srv_collision;
        s.str(std::string());
        s << "frontal_collision_course(" << my_id << ", "<< id <<")";
        srv_collision.request.belief_expression = s.str();
        srv_collision.request.multivalued = false;
        add_client.call(srv_collision);
      }
      aerostack_msgs::RemoveBelief::Request req;
      aerostack_msgs::RemoveBelief::Response res;
      s.str(std::string());
      s << "collision_course(" << my_id << ", "<< id <<")";
      req.belief_expression = s.str();
      remove_client.call(req, res);
      }else{
      aerostack_msgs::QueryBelief srv;
      std::stringstream s;
      s << "collision_course(" << my_id << ", "<< id <<")";
      srv.request.query = s.str();
      query_client.call(srv);
      aerostack_msgs::QueryBelief::Response response= srv.response;
      if(!response.success){
        std::cout<<"collision detected";
        aerostack_msgs::AddBelief srv_collision;
        s.str(std::string());
        s << "collision_course(" << my_id << ", "<< id <<")";
        srv_collision.request.belief_expression = s.str();
        srv_collision.request.multivalued = false;
        add_client.call(srv_collision);
      }
      aerostack_msgs::RemoveBelief::Request req;
      aerostack_msgs::RemoveBelief::Response res;
      s.str(std::string());
      s << "frontal_collision_course(" << my_id << ", "<< id <<")";
      req.belief_expression = s.str();
      remove_client.call(req, res);
      }
    }else{
      aerostack_msgs::QueryBelief srv;
      std::stringstream s;
      s << "frontal_collision_course(" << my_id << ", "<< id <<")";
      srv.request.query = s.str();
      query_client.call(srv);
      aerostack_msgs::QueryBelief::Response response1= srv.response;
      if(response1.success){
      aerostack_msgs::RemoveBelief::Request req;
      aerostack_msgs::RemoveBelief::Response res;
      s.str(std::string());
      s << "frontal_collision_course(" << my_id << ", "<< id <<")";
      req.belief_expression = s.str();
      remove_client.call(req, res);
      }
      s.str(std::string());
      s << "collision_course(" << my_id << ", "<< id <<")";
      srv.request.query = s.str();
      query_client.call(srv);
      aerostack_msgs::QueryBelief::Response response2= srv.response;
      if(response2.success){
      aerostack_msgs::RemoveBelief::Request req;
      aerostack_msgs::RemoveBelief::Response res;
      s.str(std::string());
      s << "collision_course(" << my_id << ", "<< id <<")";
      req.belief_expression = s.str();
      remove_client.call(req, res);
      }
    }
    geometry_msgs::Point p;
    p.x=val_x;
    p.y=val_y;
    p.z=val_z;

    std::pair <geometry_msgs::Point, int32_t> pair (p, message.time);
    last_positions[id]=pair;
  }
}else{  
  //update last shared position 
  geometry_msgs::Point p;
  p.x=val_x;
  p.y=val_y;
  p.z=val_z;

  std::pair <geometry_msgs::Point, int32_t> pair (p, message.time);
  last_positions[id]=pair;
  }
}
}

void BeliefUpdaterProcess::poseCallback(const geometry_msgs::PoseStamped& pos) {
  std::string new_flight_state;
  if(pos.pose.position.z < 0.1) {
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
  Point new_pose(pos.pose.position.x, pos.pose.position.y, pos.pose.position.z);
  new_pose.roundTo(POSE_MIN_DISTANCE);
  if(current_pose.maxDifference(new_pose) > 0) {
    bool success = sendPose(new_pose);
    if(success) {
      current_pose = new_pose;
    }
    //check if its not the first time
    if( !(last_positions.find(my_id) == last_positions.end())){
      //calculate velocity
      int32_t d_time=abs(int(pos.header.stamp.sec-last_positions[my_id].second));
      if(d_time>0.1){
      double x_now = pos.pose.position.x;
      double x_previous = last_positions[my_id].first.x;
      double d_x= x_now-x_previous; 
      double vel_x = d_x/(double)d_time;
      double y_now = pos.pose.position.y;
      double y_previous = last_positions[my_id].first.y;
      double d_y= y_now-y_previous; 
      double vel_y = d_y/(double)d_time;
      double z_now = pos.pose.position.z;
      double z_previous = last_positions[my_id].first.z;
      double d_z= z_now-z_previous; 
      double vel_z = d_z/(double)d_time;
      vel.x=vel_x;
      vel.y=vel_y;
      vel.z=vel_z;
      //update last position of the own drone
      geometry_msgs::Point p;
      p.x=pos.pose.position.x;
      p.y=pos.pose.position.y;
      p.z=pos.pose.position.y;
      std::pair <geometry_msgs::Point, int32_t> pair (p, pos.header.stamp.sec);
      last_positions[my_id]=pair;
    }
    }else{  
    //update last position of the own drone
    geometry_msgs::Point p;
    p.x=pos.pose.position.x;
    p.y=pos.pose.position.y;
    p.z=pos.pose.position.y;
    std::pair <geometry_msgs::Point, int32_t> pair (p, pos.header.stamp.sec);
    last_positions[my_id]=pair;
    }
  }
}

void BeliefUpdaterProcess::batteryCallback(const  sensor_msgs::BatteryState& battery) {
    std::cout<<"new battery"<<std::endl;
  std::string new_battery_level;
  if(battery.percentage*100 < BATTERY_LOW_THRESHOLD) {
    new_battery_level = "LOW";
  } else if(battery.percentage*100 < BATTERY_MEDIUM_THRESHOLD) {
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
  aerostack_msgs::AddBelief::Request req;
  aerostack_msgs::AddBelief::Response res;

  std::stringstream ss;
  ss << "flight_state(" << my_id << ", " << flight_state << ")";
  req.belief_expression = ss.str();
  req.multivalued = false;

  add_client.call(req, res);

  return res.success;
}

bool BeliefUpdaterProcess::sendPose(Point pose) {
  aerostack_msgs::AddBelief::Request req;
  aerostack_msgs::AddBelief::Response res;
  if(pose.x==-0){
    pose.x=0;
  }
  if(pose.y==-0){
    pose.y=0;
  }
  if(pose.z==-0){
    pose.z=0;
  }
  std::stringstream ss;
  ss << "position(" << my_id << ", (" << pose.x << ", " << pose.y << ", " << pose.z << "))";
  req.belief_expression = ss.str();
  req.multivalued = false;
  add_client.call(req, res);
  return res.success;
}

bool BeliefUpdaterProcess::sendBatteryLevel(std::string level) {
  aerostack_msgs::AddBelief::Request req;
  aerostack_msgs::AddBelief::Response res;
  std::stringstream ss;
  ss << "battery_level(" << my_id << ", " << level << ")";
  req.belief_expression = ss.str();
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
