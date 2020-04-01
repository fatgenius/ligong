#include <rp_goal.h>

namespace zev{

void RpGoal::sub_goal_cb(const geometry_msgs::PoseStamped::ConstPtr &goal_) {    
      
    mPose_now_sub.pose.position.x = goal_->pose.position.x;
    mPose_now_sub.pose.position.y = goal_->pose.position.y;
    mPose_now_sub.pose.position.z = goal_->pose.position.z;

    mPose_now_sub.pose.orientation.x = goal_->pose.orientation.x;
    mPose_now_sub.pose.orientation.y = goal_->pose.orientation.y;
    mPose_now_sub.pose.orientation.z = goal_->pose.orientation.z;
    mPose_now_sub.pose.orientation.w = goal_->pose.orientation.w;

    mPose_now_sub.header.frame_id = "map";
    // pub_test();
    bPose_ok = true;
    std::cout << "----get_pos successful----\n";
}

void RpGoal::sub_state_cb(const std_msgs::Int8::ConstPtr& msg_) {
    std::cout << "sub mv_state = " <<  (int)msg_->data << "\n";
    switch(msg_->data){
        case EN_SUB_STATE::Reach: {
            if(iRecov_faild > 0){
                iRecov_faild = 0;
                pub_goal();
            }
            else if(vePoses.size() > 1 && iState == EN_NOW_STATE::Atuo) {
                ++iIndex_goals;
                if(iIndex_goals >= vePoses.size()){
                    iIndex_goals = 0;
                }
                pub_goal();
            }
        }break;
        case EN_SUB_STATE::RecovFaild: {
            // pub_state(EN_PUB_STATE::ReturnPath);
            // ++iRecov_faild;
        }break;
        default:break;
    }
}

RpGoal::RpGoal(void){
    ros::NodeHandle nh;

    mThread_key = new std::thread(&RpGoal::get_key_cb, this);
    mThread_key->detach();
    // iThread_key_state = 1;

    mPub_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    mSub_pose = nh.subscribe<geometry_msgs::PoseStamped>("mv_pose_now", 1, boost::bind(&RpGoal::sub_goal_cb, this, _1));
    
    mPub_state = nh.advertise<std_msgs::Int8>("/mv_sub_state", 1);
    mSub_state = nh.subscribe<std_msgs::Int8>("mv_pub_state", 1, boost::bind(&RpGoal::sub_state_cb, this, _1));

    mPub_init_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/zev_initialpose", 1);
}

RpGoal::~RpGoal(){
    delete mThread_key;
}

void RpGoal::get_key_cb(void){
    while(ros::ok){
        switch(std::cin.get()){
            /*获取当前机器人坐标*/
            case 'g':{
                get_pos();
                std::cout <<"----pub get_pos_state----\n";
            }break;
            /*保存当前获取的机器人坐标到缓冲区*/
            case 's':{
                if(bPose_ok == true){
                    vePoses.push_back(mPose_now_sub);
                    bPose_ok = false;
                    std::cout << "----save "<< vePoses.size() << " robot_pose----\n";
                }
                else{
                    std::cout << "----pose invaild----\n";
                }
            }break;
            /*发送当前序号坐标作为机器人要达到的目标*/
            case 'd':{
                if(vePoses.empty() == false){
                    ++iIndex_goals;
                    if(iIndex_goals >= vePoses.size()){
                        iIndex_goals = 0;
                    }
                    pub_goal();
                    std::cout << "----pub robot_pose  "<< iIndex_goals << "  ----\n";
                }
                else{
                    std::cout << "----vePoses is empty----\n";
                }
            }break;
            /*将当前缓冲区信息写入xml*/
            case 'w':{
                if(write_xml()){
                    std::cout << "----write xml successful----\n";
                }
                else{
                    std::cout << "----write xml faild----\n";
                }
            }break;
            /*读取xml*/
            case 'r':{
                if(read_xml()) {
                    std::cout << "----read " << vePoses.size() << " poses from xml successful----\n";
                }
                else{
                    std::cout << "----read xml faild----\n";
                }
            }break;
            /*开启自动循环模式*/
            case 'a':{
                if(iState == EN_NOW_STATE::Atuo){
                    iState = EN_NOW_STATE::Free;
                    std::cout << "----free_model----\n";
                }
                else{
                    iState = EN_NOW_STATE::Atuo;
                    std::cout << "----atuo_model----\n";
                }
            }break;
            /*重新初始化rp_topic*/
            case 'q':{
                reset();
                std::cout << "----reset---\n";
            }break;
            /*发送清除地图标志*/
            case 'c':{ 
                pub_state(EN_PUB_STATE::ClearMap);
                std::cout << "----pub_ClearMap----\n";
            }break;
            /*退出按键操作*/
            case 'e':{
                std::cout << "-----exit key_operation-----\n";
                return;
            }break;
            /*将当前缓冲区坐标写入初始化坐标缓冲区*/
            case 'i':{
                if(bPose_ok == true){
                    mPose_init.pose.pose = mPose_now_sub.pose;
                    // mPose_init.pose.pose.position = mPose_now_sub.pose.position;
                    // mPose_init.pose.pose.orientation = mPose_now_sub.pose.orientation;
                    bPose_ok = false;
                    bFlag_init_pose = true;
                    std::cout << "----save init_robot_pose----\n";
                }
                else{
                    bFlag_init_pose = false;
                    std::cout << "----pose invaild----\n";
                }
            }break;
            /*发布初始化坐标*/
            case 'u':{
                if(bFlag_init_pose){
                    mPub_init_pose.publish(mPose_init);
                    std::cout << "----pub init_pose successful----\n";
                }
                else{
                    std::cout << "----init_pose invaild----\n";
                }
            }break;
            /*取队列里的所有坐标求平均存入平均变量中*/
            case 'f':{
                if(vePoses.empty() == false){
                    mPose_avge = vePoses[0];
                    for(int i = 1; i < vePoses.size(); ++i){
                        mPose_avge.pose.position.x += vePoses[i].pose.position.x;
                        mPose_avge.pose.position.y += vePoses[i].pose.position.y;
                        mPose_avge.pose.position.z += vePoses[i].pose.position.z;
                    }
                    mPose_avge.pose.position.x /= vePoses.size();
                    mPose_avge.pose.position.y /= vePoses.size();
                    mPose_avge.pose.position.z /= vePoses.size();
                    bFlag_avge_pose = true;
                    std::cout << "-----cal avge_pose use test_val = " << vePoses.size() << "-----\n";
                }
                else{
                    std::cout << "-----faild cal avge_pose test_val = Null-----\n";
                }
            }break;
            /*将计算的平均值写入坐标容器中*/
            case 'v':{
                if(bFlag_avge_pose){
                    vePoses.push_back(mPose_avge);
                    bFlag_avge_pose = false;
                    std::cout << "-----save avge_pose successful-----\n";
                }
                else{
                    std::cout << "-----avge_pose invaild-----\n";
                }
            }break;
            default:break;
        }
    }
}

void RpGoal::pub_goal(const int& index_){
    if(vePoses.empty() == false){
        if(index_ < 0){
            mPub_goal.publish(vePoses[iIndex_goals]);
        }
        else if(index_ < vePoses.size()){
            mPub_goal.publish(vePoses[index_]);
        }
    }
}

bool RpGoal::PoseStamp2mat(const geometry_msgs::PoseStamped& pose_, cv::Mat& res_){
    if(res_.rows != 1 || res_.cols != 7 || res_.type() != CV_32FC1){
        res_ = cv::Mat(1, 7, CV_32FC1);
    }

    res_.at<float>(0, 0) = pose_.pose.position.x;
    res_.at<float>(0, 1) = pose_.pose.position.y;
    res_.at<float>(0, 2) = pose_.pose.position.z;

    res_.at<float>(0, 3) = pose_.pose.orientation.x;
    res_.at<float>(0, 4) = pose_.pose.orientation.y;
    res_.at<float>(0, 5) = pose_.pose.orientation.z;
    res_.at<float>(0, 6) = pose_.pose.orientation.w;

    return true;
}

bool RpGoal::mat2PoseStamp(const cv::Mat& src_, geometry_msgs::PoseStamped& pose_){
    if(src_.rows !=1 || src_.cols != 7){
        return false;
    }
    pose_.pose.position.x = src_.at<float>(0, 0);
    pose_.pose.position.y = src_.at<float>(0, 1);
    pose_.pose.position.z = src_.at<float>(0, 2);

    pose_.pose.orientation.x = src_.at<float>(0, 3);
    pose_.pose.orientation.y = src_.at<float>(0, 4);
    pose_.pose.orientation.z = src_.at<float>(0, 5);
    pose_.pose.orientation.w = src_.at<float>(0, 6);
    return true;
}

void RpGoal::reset(void){
    iState = -1;
    bPose_ok = false;
    vePoses.clear();
    iIndex_goals = -1;
}

void RpGoal::pub_state(const int& state_) const {
    std_msgs::Int8 mMsg;
    mMsg.data = state_;
    mPub_state.publish(mMsg);
}

bool RpGoal::write_xml(void) const{
    cv::FileStorage mFs("/home/ai/Downloads/AMR/amr_lg_b/src/topic_demo/poses.xml", cv::FileStorage::WRITE);
    if(mFs.isOpened() == false){
        return false;
    }
    cv::Mat mT;
    mFs << "nums" << (int)vePoses.size();
    mFs << "poses" << "[:";
    for(auto &ai : vePoses){
        PoseStamp2mat(ai, mT);
        if(mT.empty()){
           mFs << "empty"; 
        }
        else{
            mFs << mT;
        }
    }
    mFs << "]";
    
    mFs << "init_pose_flag" << bFlag_init_pose;
    if(bFlag_init_pose) {
        geometry_msgs::PoseStamped mPose_t;
        mPose_t.pose = mPose_init.pose.pose;
        PoseStamp2mat(mPose_t, mT);
        mFs << "init_pose" << mT;
    }

    mFs.release();
    return true;
}

bool RpGoal::read_xml(void){
    cv::FileStorage mFs("/home/ai/Downloads/AMR/amr_lg_b/src/topic_demo/poses.xml", cv::FileStorage::READ);
    if(mFs.isOpened() == false){
        return false;
    }

    std::vector<cv::Mat> veT;
    mFs["poses"] >> veT;
    
    if(veT.size() != 0){
        vePoses.clear();
        geometry_msgs::PoseStamped mPs_t;
        mPs_t.header.frame_id = "map";
        for(auto &ai : veT){
            // std::cout << ai << '\n';
            mat2PoseStamp(ai, mPs_t);
            vePoses.push_back(mPs_t);
        }
    }

    mFs["init_pose_flag"] >> bFlag_init_pose; 
    if(bFlag_init_pose){
        cv::Mat mT;
        mFs["init_pose"] >> mT;
        geometry_msgs::PoseStamped mPose_t;
        mat2PoseStamp(mT, mPose_t);
        mPose_init.pose.pose = mPose_t.pose;
        mPose_init.header.frame_id = "map";
        mPose_init.pose.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                      0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942};
    }
    
    return true;
}

}