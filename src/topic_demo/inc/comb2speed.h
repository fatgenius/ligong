#ifndef _comb2speed_h_
#define _comb2speed_h_

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

namespace zev{
    class Comb2Speed{
    protected:
        /*接收combined, 发送速度odom*/
        ros::Subscriber mSub_combined;
        ros::Publisher mPub_odom;

        /*上一次combined*/
        geometry_msgs::PoseWithCovarianceStamped mCombined_last;
        double dYaw_last = 0.0;

    public:
        Comb2Speed(void);

        virtual ~Comb2Speed();
    
    protected:
        /*接收combined回调函数*/
        void sub_combined_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &rev_);
    };
}

#endif