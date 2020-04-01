#ifndef _ros_map_use_h_
#define _ros_map_use_h_

#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>

namespace zev
{
class RosMapUse
{
protected:
    cv::Mat mSrc_map_;
    cv::Mat mDst_map_;

    double dResolution_map_;
    cv::Point mP_center;
    cv::Rect mRect_map;
    bool bFlag_init = false;

public:
    RosMapUse(const std::string &_str_map_path, const double &_resolution_map);
    ~RosMapUse();

    bool draw_robot_pose(const cv::Point &_offset = cv::Point(0, 0));

    void test(void);

protected:
    bool solut_map(const cv::Mat &_src);
};
} // namespace zev
#endif
