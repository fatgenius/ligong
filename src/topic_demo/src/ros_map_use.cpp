#include <ros_map_use.h>

namespace zev
{

RosMapUse::RosMapUse(const std::string &_str_map_path, const double &_resolution_map)
{
    if (_str_map_path.empty())
    {
        std::cerr << "_str_map_path is empty\n";
        exit(1);
    }
    else
    {
        mSrc_map_ = cv::imread(_str_map_path);
        if (mSrc_map_.empty())
        {
            std::cerr << "_str_map_path is vaild\n";
            exit(1);
        }
    }

    if (_resolution_map <= 0)
    {
        std::cerr << "_resolution_map less than zero\n";
        exit(1);
    }
    else
    {
        dResolution_map_ = _resolution_map;
    }

    if (solut_map(mSrc_map_) == false)
    {
        std::cerr << "solut_map faild\n";
        exit(1);
    }

    bFlag_init = true;
}

RosMapUse::~RosMapUse() {}

bool RosMapUse::solut_map(const cv::Mat &_src)
{
    if (_src.empty())
    {
        return false;
    }

    mRect_map.x = -1;
    mRect_map.y = -1;
    mRect_map.width = -1;
    mRect_map.height = -1;
    auto mdata_Zero = _src.at<cv::Vec3b>(0, 0);
    for (auto i = 0; i < _src.rows; ++i)
    {
        for (auto j = 0; j < _src.cols; ++j)
        {
            if (mdata_Zero != _src.at<cv::Vec3b>(i, j))
            {
                if (j < mRect_map.x || mRect_map.x == -1)
                {
                    mRect_map.x = j;
                }

                if (j > mRect_map.width || mRect_map.width == -1)
                {
                    mRect_map.width = j;
                }

                if (i < mRect_map.y || mRect_map.y == -1)
                {
                    mRect_map.y = i;
                }

                if (i > mRect_map.height || mRect_map.height == -1)
                {
                    mRect_map.height = i;
                }
            }
        }
    }

    mRect_map.width -= mRect_map.x;
    mRect_map.height -= mRect_map.y;
    mDst_map_ = _src(mRect_map).clone();

    mP_center = _src.size() / 2;
    mP_center -= cv::Point(mRect_map.x, mRect_map.y);

    return true;
}

bool RosMapUse::draw_robot_pose(const cv::Point &_offset)
{
    if (bFlag_init == false)
    {
        return false;
    }
    auto mPose_t = mP_center;
    mPose_t.x += _offset.x / dResolution_map_;
    mPose_t.y -= _offset.y / dResolution_map_;
    mDst_map_ = mSrc_map_(mRect_map).clone();
    cv::circle(mDst_map_, mPose_t, 2, cv::Scalar(0, 0, 255), 6);

    cv::imshow("map", mDst_map_);
    return true;
}

void RosMapUse::test(void)
{
    draw_robot_pose(cv::Point(1, 1));
    cv::imshow("map", mDst_map_);
    cv::waitKey(1);
}
} // namespace zev
