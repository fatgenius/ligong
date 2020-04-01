//ROS头文件
#include <ros/ros.h>
//包含自定义msg产生的头文件
#include <topic_demo/gps.h>
//ROS标准msg头文件
#include <std_msgs/Float32.h>

#include <geometry_msgs/PoseStamped.h>

ros::Publisher pub;
int iIndex_t = 0;
int iNums;
std::vector<geometry_msgs::PoseStamped> goal;

void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr &goal_) {    
  std::cout << "\n x = " << goal_->pose.position.x << "\n y = " << goal_->pose.position.y <<
           "\n z = " << goal_->pose.position.x << "\n o_x = " << goal_->pose.orientation.x << 
           "\n o_y= " << goal_->pose.orientation.z << "\n o_z = " << goal_->pose.orientation.z << 
           "\n o_w = " << goal_->pose.orientation.w << "\n";

  
  iIndex_t = iIndex_t % iNums;
  std::cout << "\n---next_pos_index = " << iIndex_t << "---\n";
  goal[iIndex_t].header.stamp = ros::Time::now();
  pub.publish(goal[iIndex_t]);
  ++iIndex_t;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "listener");
  ros::NodeHandle private_nh("~");


  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<geometry_msgs::PoseStamped>("zev_reach", 1, gpsCallback);
  pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
   
  private_nh.getParam("nums", iNums);
  goal.resize(iNums);

  {
    auto &atGoal_index = goal[0];
    auto &atVeptr_pos = atGoal_index.pose.position;
    auto &atVeptr_or = atGoal_index.pose.orientation;

    private_nh.getParam("x", atVeptr_pos.x);
    private_nh.getParam("y",atVeptr_pos.y);
    private_nh.getParam("z", atVeptr_pos.z);

    private_nh.getParam("rx", atVeptr_or.x);
    private_nh.getParam("ry", atVeptr_or.y);
    private_nh.getParam("rz", atVeptr_or.z);
    private_nh.getParam("rw", atVeptr_or.w);

    atGoal_index.header.frame_id = "map";
  }

  {
    auto &atGoal_index = goal[1];
    auto &atVeptr_pos = atGoal_index.pose.position;
    auto &atVeptr_or = atGoal_index.pose.orientation;

    private_nh.getParam("x1", atVeptr_pos.x);
    private_nh.getParam("y1",atVeptr_pos.y);
    private_nh.getParam("z1", atVeptr_pos.z);

    private_nh.getParam("rx1", atVeptr_or.x);
    private_nh.getParam("ry1", atVeptr_or.y);
    private_nh.getParam("rz1", atVeptr_or.z);
    private_nh.getParam("rw1", atVeptr_or.w);

    atGoal_index.header.frame_id = "map";
  }

  {
    auto &atGoal_index = goal[2];
    auto &atVeptr_pos = atGoal_index.pose.position;
    auto &atVeptr_or = atGoal_index.pose.orientation;

    private_nh.getParam("x2", atVeptr_pos.x);
    private_nh.getParam("y2",atVeptr_pos.y);
    private_nh.getParam("z2", atVeptr_pos.z);

    private_nh.getParam("rx2", atVeptr_or.x);
    private_nh.getParam("ry2", atVeptr_or.y);
    private_nh.getParam("rz2", atVeptr_or.z);
    private_nh.getParam("rw2", atVeptr_or.w);

    atGoal_index.header.frame_id = "map";
  }
  
  //ros::spin()用于调用所有可触发的回调函数。将进入循环，不会返回，类似于在循环里反复调用ros::spinOnce()。
  ros::spin(); 
  return 0;
}

