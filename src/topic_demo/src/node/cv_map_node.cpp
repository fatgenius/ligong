#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros_map_use.h>

void sub_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose_);

zev::RosMapUse mMap("/home/ai/Downloads/AMR/mooc/src/ROS-Academy-for-Beginners-master/slam_sim_demo/maps/Software_Museum.pgm",0.05);

int main(int argc, char** argv){
    ros::init(argc, argv, "cv_map_node");

    ros::NodeHandle nh;
    auto mSub_pose = nh.subscribe<geometry_msgs::PoseStamped>("mv_pose_now", 1, sub_pose_cb);

    
    mMap.draw_robot_pose();
    while(cv::waitKey(1) != 'q'){
        ros::spinOnce();
    }
    return true;
}

void sub_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose_){
    mMap.draw_robot_pose(cv::Point(pose_->pose.position.x, pose_->pose.position.y));
}