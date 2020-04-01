/*
        *类名 ： 数学和工具
        *作者 ：zev
        *联系方式：zev.huang@trojanjet.com
        *类创建日期 ： 2019/6/5
*版本 ： V1.0
Copyright (C), 2012-2018 , TrojanJet Intelligent Technologies LTD , All rights reserved
* 修订：
 * 2019.6.6：增加求距离函数
*/
#pragma once
#ifndef _math_tool_hpp_
#define _math_tool_hpp_

#ifndef ZEV_PI
#define ZEV_PI 3.1415926
#endif

// #define ZEV_LINUX
#define ZEV_OPENCV
// #define ZEV_QT

#include <sstream>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#ifdef ZEV_LINUX
#include <dirent.h>
#endif

#ifdef ZEV_OPENCV
#include <opencv2/opencv.hpp>
#endif

#ifdef ZEV_LINUX
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#endif

#ifdef ZEV_QT
#include <QStringList>
#endif

namespace ZevTYPE {
    typedef std::pair<double, long> PAIR_DL;

    typedef std::vector<double> VE_1D;
    typedef std::vector<VE_1D> VE_2D;
    typedef std::vector<PAIR_DL> VE_PAIR_DL;

#ifdef ZEV_OPENCV
    typedef std::vector<cv::Point2f> VE_CVP2F;
    typedef std::vector<cv::Point> VE_CVP;

    typedef std::vector<cv::Mat> VE_MAT;
#endif
}

namespace ZevEN{
    enum EN_LEVEL{
        LEVEL_MIN = -1,
        LEVEL_W,
        LEVEL_Q,
        LEVEL_B,
        LEVEL_S,
        LEVEL_G,
        LEVEL_P1,
        LEVEL_P2,
        LEVEL_P3,
        LEVEL_P4,
        LEVEL_P5,
        LEVEL_P6,
        LEVEL_MAX,
    };
}

namespace MathTool {
	/*数字转字符串
	 * numbers_ : 输入数字
	 */
	template<typename T> static std::string num_str(const T &numbers_) {
		std::string re_str;
		std::stringstream stream;
		stream << numbers_;
		stream >> re_str;
		return re_str;
	}

	/*字符串转数字
	 * 输入字符串
	 * */
	template<typename T> static T str_num(const std::string& str_) {
		T re_num = 0;
		std::stringstream stream(str_);
		stream >> re_num;
		return re_num;
	}

    /*获取文件名后缀
     * file_name_ : 文件名
     * */
    static std::string read_file_exd(const std::string& file_name_){
        for(auto ai = file_name_.size() - 2; ai >= 0; --ai){
            if(file_name_[ai] == '.'){
                return std::string(file_name_.begin() + ai + 1, file_name_.end());
            }
        }
        return std::string();
    }

    /*获取文件夹特定后缀文件名列表
     * path_ : 文件夹路径
     * exd_ : 文件名后缀
     * list_ : 输出文件列表
     * complete_ : 是否输出完整路径
     * */
    static bool get_exd_files_name(const std::string& path_, const std::string& exd_,
            std::vector<std::string>& list_, const bool& complete_ = false){
#ifdef ZEV_LINUX
        DIR *dir;
        dir = opendir(path_.c_str());
        struct dirent *ptr;
        while((ptr = readdir(dir)) != NULL) {
            /*跳过'.'和'..'两个目录*/
            if(ptr->d_name[0] == '.'){
                continue;
            }
            std::string str_t = ptr->d_name;
            if(read_file_exd(str_t) == exd_) {
                if(complete_){
                    str_t = path_ + '/' + str_t;
                }
                list_.push_back(str_t);
            }
        }
        closedir(dir);
        /*如果获取列表为空*/
        if(list_.empty()){
            return false;
        }
#endif
        return true;
    }

    /*一维double型向量将排序名次按位置输出
 * ve_src_ : 源向量
 * ve_res_ : 输出向量*/
    static bool get_ve_sort_seq(const ZevTYPE::VE_1D &ve_src_, ZevTYPE::VE_1D &ve_res_){
        if(ve_src_.size() < 2){
            return false;
        }
        /*创建查找标记向量*/
        ZevTYPE::VE_1D veFlag(ve_src_.size(), -1);
        ve_res_.resize(ve_src_.size());

        double dMix;
        double dMax;

        double dMix_index;
        double dMax_index;

        double dCount = 0;
        double dLen = ve_src_.size() - 1;

        bool bFlag_exit = true;
        while(1){
            bFlag_exit = false;
            /*遍历源向量*/
            for (int i = 0; i < ve_src_.size(); ++i) {
                if(veFlag[i] != -1){
                    continue;
                }
                if(bFlag_exit == false){
                    bFlag_exit = true;
                    dMix = ve_src_[i];
                    dMax = ve_src_[i];
                    dMix_index = i;
                    dMax_index = i;
                }
                else if(dMix >= ve_src_[i]){
                    dMix = ve_src_[i];
                    dMix_index = i;
                }
                else if(dMax <= ve_src_[i]){
                    dMax = ve_src_[i];
                    dMax_index = i;
                }
            }
            if(bFlag_exit == false){
                break;
            }
            veFlag[dMix_index] = dCount;
            veFlag[dMax_index] = dLen - dCount;
            ++dCount;
        }
        ve_res_ = veFlag;
        return true;
    }

    /*二维double型向量转为一维double型向量
     * ve_src_ : 源向量
     * ve_res_ : 输出向量*/
    static bool VE_2D_to_VE_1D(const ZevTYPE::VE_2D &ve_src_, ZevTYPE::VE_1D &ve_res_){
        ve_res_.clear();
        for(auto &ai : ve_src_){
            for(auto &aj : ai){
                ve_res_.push_back(aj);
            }
        }
        return true;
    }

    /*判断当前数字的级数*/
    template<typename  T> static T read_get_nums_level(const T& nums_){
        double dT_abs = std::abs(nums_), dT_level = 0;
        /*万级别的数字*/
        if(dT_abs / 10000 >= 1){
            dT_level = ZevEN::EN_LEVEL::LEVEL_W;
        }
        /*千级别的数字*/
        else if(dT_abs / 1000 >= 1){
            dT_level = ZevEN::EN_LEVEL::LEVEL_Q;
        }
        /*百级别的数字*/
        else if(dT_abs / 100 >= 1){
            dT_level = ZevEN::EN_LEVEL::LEVEL_B;
        }
        /*十级别的数字*/
        else if(dT_abs / 10 >= 1){
            dT_level = ZevEN::EN_LEVEL::LEVEL_S;
        }
        /*个级别的数字*/
        else if(dT_abs >= 1){
            dT_level = ZevEN::EN_LEVEL::LEVEL_G;
        }
        /*一位小数级别的数字*/
        else if(dT_abs / 0.1 >= 1){
            dT_level = ZevEN::EN_LEVEL::LEVEL_P1;
        }
        /*二位小数级别的数字*/
        else if(dT_abs / 0.01 >= 1){
            dT_level = ZevEN::EN_LEVEL::LEVEL_P2;
        }
        /*三位小数级别的数字*/
        else if(dT_abs / 0.001 >= 1){
            dT_level = ZevEN::EN_LEVEL::LEVEL_P3;
        }
        /*四位小数级别的数字*/
        else if(dT_abs / 0.0001 >= 1){
            dT_level = ZevEN::EN_LEVEL::LEVEL_P4;
        }
        /*五位小数级别的数字*/
        else if(dT_abs / 0.00001 >= 1){
            dT_level = ZevEN::EN_LEVEL::LEVEL_P5;
        }
        /*六位小数级别的数字*/
        else if(dT_abs / 0.000001 >= 1){
            dT_level = ZevEN::EN_LEVEL::LEVEL_P6;
        }

        if(nums_ < 0){
            dT_level = -dT_level;
        }
        return dT_level;
    }

#ifdef ZEV_LINUX
    /*获取文件文件数量
     * 文件夹路径
     * */
    static int get_files_nums(const char* dir_name) {
        // check the parameter !
        if(dir_name == NULL) {
//        std::cout << "dir_name is null !" << std::endl;
            return -1;
        }

        // check if dir_name is a valid dir
        struct stat s;
        lstat(dir_name, &s);
        if(!S_ISDIR( s.st_mode ) ) {
//        std::cout << "dir_name is not a valid directory !" << std::endl;
            return -1;
        }

        struct dirent * filename;    // return value for readdir()
        DIR * dir;                   // return value for opendir()
        dir = opendir(dir_name);
        if( NULL == dir ) {
//        std::cout << "Can not open dir" << dir_name << std::endl;
            return -1;
        }
//        std::cout << "Successfully opened the dir !" << std::endl;

        /* read all the files in the dir ~ */
        int iCount = 0;
        while( ( filename = readdir(dir) ) != NULL ) {
            // get rid of "." and ".."
            if( strcmp( filename->d_name , "." ) == 0 ||
                strcmp( filename->d_name , "..") == 0)
                continue;
            ++iCount;
//        std::cout << filename->d_name << std::endl;
        }
        return iCount;
    }
#endif

#ifdef ZEV_QT
    /*QT字符串列表转标准库字符串向量
     * Qstr_list_ : QT字符串列表
     * return : 标准库字符串向量
     * */
    static std::vector<std::string> QStrList2vecStr(const QStringList& Qstr_list_){
        std::vector<std::string> veStr_re(Qstr_list_.size());
        for(int i = 0; i < veStr_re.size(); ++i){
            veStr_re[i] = Qstr_list_[i].toStdString();
        }
        return veStr_re;
    }
#endif

#ifdef ZEV_OPENCV
    /*HSV颜色空间任意通道赋值统一的值
     * src_ : 原图像
     * index_ : 通道编号
     * val_ : 值
     * */
    static bool set_HSV_fixed(cv::Mat& src_, const int& index_, const uchar& val_){
        if(index_ > 2 || index_ < 0 || src_.empty()){
            return false;
        }
        cv::Mat mPipe[3];
        cv::split(src_, mPipe);
        mPipe[index_] = val_;
//        std::cout << mPipe[index_];
        cv::merge(mPipe, 3, src_);
        return true;
    }

    /*获取两点间的距离
    * p0_ : 点0
    * p0_ : 点1
    * */
    static double get_distance(const cv::Point& p0_, const cv::Point& p1_) {
        return sqrtf(powf((p0_.x - p1_.x), 2) + powf((p0_.y - p1_.y), 2));
    }

    /*获取对角线掩膜
     * res_ : 输入输出图像
     * */
    static bool get_diag_mask(cv::Mat& res_, const bool& tangent_ = true){
        if(res_.empty()){
            return false;
        }
        res_ = cv::Mat(res_.size(), CV_8UC1, cv::Scalar(0));
        cv::line(res_, cv::Point(0, 0), cv::Point(res_.cols, res_.rows), cv::Scalar(255));
        for (int i = 0; i < res_.rows; ++i) {
            auto atPtr = res_.ptr<uchar>(i);
            for (int j = 0; j < res_.cols; ++j) {
                if (*atPtr == 255) {
                    break;
                } else {
                    *atPtr = 255;
                }
                    atPtr++;
            }
        }
        if(tangent_ == false){
            res_ = ~res_;
        }
        return true;
    }

    /*获取单映矩阵并进行映射
     * src_ : 原图像
     * res_ : 输出图像
     * p_src_ : 原图像关键点集
     * p_res_ : 输出图像关键点集
     * size_ : 输出图像尺寸*/
    static bool get_Homography_warp(const cv::Mat& src_, cv::Mat& res_,
            const ZevTYPE::VE_CVP2F& p_src_, const ZevTYPE::VE_CVP2F& p_res_, const cv::Size& size_){

        /*获取单性矩阵*/
        cv::Mat h = cv::findHomography(p_src_, p_res_);
        /*透视变换*/
        cv::warpPerspective(src_, res_, h, size_);
        return true;
    }

    /*图像旋转任意角度
     * src_ : 原图像
     * res_ : 输出图像
     * angle_ : 旋转角度
     * scale_ : 放大倍数(默认1)
     * center_ : 旋转中心位置(默认原图像中心)
     * */
    static bool get_rotaed_mat(const cv::Mat& src_, cv::Mat& res_, const double& angle_, const double scale_ = 1,
            cv::Point center_ = cv::Point(-1, -1)){
        if(center_.x == -1){
            center_.x = src_.cols / 2;
            center_.y = src_.rows / 2;
        }
        cv::Mat mRot= getRotationMatrix2D(center_, angle_, scale_);
        warpAffine(src_, res_, mRot, src_.size());
        return true;
    }

    /*获取矩形区域顶点按从左至右的顺序排序
     * rota_ : 旋转矩形
     * ve_ : 点集向量
     * */
    template<typename T> static bool get_vertices(const cv::RotatedRect &rota_, std::vector<cv::Point_<T>>& ve_) {
        cv::Point2f corner[4];
        rota_.points(corner);
        ve_.resize(4);
        std::sort(corner, corner + 4, [](const cv::Point2f &x1, const cv::Point2f &x2){
            return x1.y < x2.y;
        });
        if(corner[0].x > corner[1].x){
            std::swap(corner[0], corner[1]);
        }
        if(corner[2].x > corner[3].x){
            std::swap(corner[2], corner[3]);
        }

        for(int i = 0; i < 4; ++i){
            ve_[i] = corner[i];
        }
        return true;
    }

    /*将视频图片按照每隔一定帧数保存图片
     * video_path_ : 视频地址
     * image_path_ : 保存图片地址
     * frame_ : 每隔几帧保存一张图片
     * image_exd_ : 保存图片后缀名*/
    static bool video_image(const std::string &video_path_, const std::string &image_path_, const int &frame_,
                            const std::string &image_exd_ = ".jpg"){
        cv::VideoCapture mCap(video_path_);
        if(mCap.isOpened() == false){
            return false;
        }

        int iCount_frame = 0;
        std::string strImage_path = image_path_ + '/';
        while(1){
            cv::Mat m;
            mCap >> m;
            if(m.empty()){
                break;
            }
            ++iCount_frame;
            if(iCount_frame % frame_ == 0){
                cv::imwrite(strImage_path + MathTool::num_str(iCount_frame) + image_exd_, m);
                std::cout << "当前保存帧数 = " << iCount_frame << "\n";
            }
        }
        mCap.release();
        std::cout << "总帧数 = " << iCount_frame << "\n\n";
        return true;
    }

#endif

};

#endif // _math_par_h_
