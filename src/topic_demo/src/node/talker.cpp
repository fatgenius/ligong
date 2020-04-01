#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>

#include <vector>

void rang_pub_t(void);
void cmd_vel_pub_t(void);

int main(int argc, char **argv) {
  //用于解析ROS参数，第三个参数为本节点名
  ros::init(argc, argv, "talker");

  // rang_pub_t();
  cmd_vel_pub_t();
  
  return 0;
} 


void rang_pub_t(void) {
  //实例化句柄，初始化node
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

  //创建publisher
  ros::Publisher pub = nh.advertise<sensor_msgs::Range>("/sonar", 1);
  //定义发布的频率 
  ros::Rate loop_rate(10);

  sensor_msgs::Range mData;
  mData.header.frame_id = "sonar";
  mData.min_range = 0.5;
  mData.max_range = 1.5;
  mData.range = 0.0;
  mData.radiation_type = 1;
  mData.field_of_view = 0.2;

  double dStep = 0.01;

  while (ros::ok()) {
    mData.header.stamp = ros::Time::now();
    // mData.range += dStep;
    mData.range = (double)(rand() % 15) / 10.0;
    ROS_ERROR("range = %f", mData.range);
    // if(mData.range >= 1.5 || mData.range <= 0){
    //   dStep = -dStep;
    // }
    pub.publish(mData);

    loop_rate.sleep();
  }
}

void cmd_vel_pub_t(void) {
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Rate loop_rate(10);

  geometry_msgs::Twist mData;
  mData.linear.x = 0.1;
  mData.linear.y = 0.0;
  mData.linear.z = 0.0;

  mData.angular.x = 0.0;
  mData.angular.y = 0.0;
  mData.angular.z = 0.1;

  while (ros::ok()) {
    pub.publish(mData);
    loop_rate.sleep();
  }
}