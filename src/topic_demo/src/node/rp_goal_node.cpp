#include <rp_goal.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "rp_goal_node");
  
  zev::RpGoal mG;

  ros::spin();
  return(0);
}