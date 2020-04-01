#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

void sub_od_odom_cb(const nav_msgs::Odometry::ConstPtr &rev_){
    nav_msgs::Odometry mVo_odom = *rev_;
    mVo_odom.pose.pose.position.x -= 0.1;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "tran_vo_odom");

  //实例化句柄，初始化node
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  
  ros::spin();
  return 0;
} 