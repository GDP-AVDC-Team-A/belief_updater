#include "belief_updater_process.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, ros::this_node::getName());

  BeliefUpdaterProcess belief_updater;
  belief_updater.setUp();
  belief_updater.start();

  ros::spin();
}
