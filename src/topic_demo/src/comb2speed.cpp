#include <comb2speed.h>

namespace zev{
    Comb2Speed::Comb2Speed(void){
        ros::NodeHandle nh;
        ros::NodeHandle n("~");

        mCombined_last.header.stamp = ros::Time::now();
        mCombined_last.pose.pose.position.x = 0.0;
        mCombined_last.pose.pose.position.y = 0.0;
        mCombined_last.pose.pose.position.z = 0.0;

        std::string strT;

        n.param<std::string>("combined_id", strT, "robot_pose_ekf/odom_combiined");
        mSub_combined = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(strT, 1, 
            boost::bind(&Comb2Speed::sub_combined_cb, this, _1));

        n.param<std::string>("speed_id", strT, "odom");
        mPub_odom = nh.advertise<nav_msgs::Odometry>(strT, 100);
    }

    Comb2Speed::~Comb2Speed(){}

    void Comb2Speed::sub_combined_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &rev_){     
        tf::Quaternion RQ2; 
        /*使用四元素计算旋转矩阵*/  
        tf::quaternionMsgToTF(rev_->pose.pose.orientation, RQ2);
        double dRoll, dPitch, dYaw;
        /*使用旋转矩阵计算欧拉角*/
        tf::Matrix3x3(RQ2).getRPY(dRoll, dPitch, dYaw);

        double dYaw_step = dYaw - dYaw_last;
        double dT = (rev_->header.stamp - mCombined_last.header.stamp).toSec();
        double dX = rev_->pose.pose.position.x - mCombined_last.pose.pose.position.x;
        double dY = rev_->pose.pose.position.y - mCombined_last.pose.pose.position.y;

        nav_msgs::Odometry odom;
        odom.header.stamp = rev_->header.stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        odom.twist.twist.linear.x = std::sqrt(dX * dX + dY * dY) / dT;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = dYaw_step / dT;
        mPub_odom.publish(odom);
        
        ROS_ERROR("cal_vx = %f  cal_vth =  %f", odom.twist.twist.linear.x, odom.twist.twist.angular.z);
        
        mCombined_last = *rev_;
        dYaw_last = dYaw;
    }
}