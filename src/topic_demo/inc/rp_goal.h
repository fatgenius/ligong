#ifndef _rp_goal_h_
#define _rp_goal_h_

#include <ros/ros.h>
//ROS标准msg头文件
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h> 
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <iostream>
#include <fstream>
#include <thread>
#include <opencv2/opencv.hpp>

#include <math_tool.hpp>

#include <tinyxml.h>
#include <tinystr.h>

namespace zev{ 
enum EN_PUB_STATE {
    GetPos = 0,
    ClearMap,
    ReturnPath,
    Test,
};

enum EN_SUB_STATE {
    Reach = 0,
    RecovFaild,
};

enum EN_NOW_STATE {
    Atuo = 0,
    Free,
};

class RpGoal{
    protected:
        /*发布目标和接收目标topic*/
        ros::Publisher mPub_goal;
        ros::Subscriber mSub_pose;

        /*发布和接收标志topic*/
        ros::Publisher mPub_state;
        ros::Subscriber mSub_state;
        
        /*状态标志*/
        int iState = -1;

        /*当前坐标是否有效标志位*/
        bool bPose_ok = false;
        /*当前接收坐标*/
        geometry_msgs::PoseStamped mPose_now_sub;

        /*坐标集*/
        std::vector<geometry_msgs::PoseStamped> vePoses;
        /*当前坐标下标*/
        int iIndex_goals = -1;

        /*获取键值线程*/
        std::thread* mThread_key = nullptr;
        int iThread_key_state = -1;

        /*计数恢复失败次数*/
        int iRecov_faild = 0;

        /*写初始化坐标信息*/
        bool bFlag_init_pose = false;
        geometry_msgs::PoseWithCovarianceStamped mPose_init;
        ros::Publisher mPub_init_pose;

        /*平均坐标*/
        geometry_msgs::PoseStamped mPose_avge;
        bool bFlag_avge_pose = false;

    public:
        RpGoal(void);
        ~RpGoal(void);

    public:
        /*获取坐标*/
        inline void get_pos(void){
            pub_state(EN_PUB_STATE::GetPos);
        }

        /*发布测试标志*/
        inline void pub_test(void){
            std_msgs::Int8 mMsg;
            mMsg.data = EN_PUB_STATE::Test;
            mPub_state.publish(mMsg);
        }

        /*发布坐标
        *index_ ： 坐标下标(默认-1，当前坐标)
        */
        void pub_goal(const int& index_ = -1);

        /*PoseStamped类型转mat类型
        * pose_ : 输入geometry_msgs::PoseStamped
        * res_ : 输出cv::Mat
        */
        static bool PoseStamp2mat(const geometry_msgs::PoseStamped& pose_, cv::Mat& res_);

        /*mat类型转PoseStamped类型
        *src_ : 输入cv::Mat
        *pose_ : 输出geometry_msgs::PoseStamped
        */
        static bool mat2PoseStamp(const cv::Mat& src_, geometry_msgs::PoseStamped& pose_);

        /*初始化配置*/
        void reset(void);

        /*发布标志位*/
        void pub_state(const int& state_) const;

    protected:
        /*接收坐标回调函数
        * goal_ : 接收目标类型变量
        */
        void sub_goal_cb(const geometry_msgs::PoseStamped::ConstPtr &goal_);

        /*接收状态回调函数
        * msg_ : 接收目标类型变量
        */
        void sub_state_cb(const std_msgs::Int8::ConstPtr& msg_);

        /*获取键值和执行相应操作回调函数*/
        void get_key_cb(void);

        /*将坐标集写入xml文件*/
        bool write_xml(void) const;

        /*读取坐标集xml文件*/
        bool read_xml(void);

};
}
#endif
