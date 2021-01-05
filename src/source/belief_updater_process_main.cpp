#include "belief_updater_process.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, ros::this_node::getName());

  BeliefUpdaterProcess belief_updater;
  belief_updater.ownSetUp();
  belief_updater.ownStart();

  ros::Rate r(100); // 100 hz
  while (ros::ok())
  {
   ros::spinOnce();
   r.sleep();
  }
}
