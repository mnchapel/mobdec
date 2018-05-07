/// @file   kltTracker.cpp
/// @author Marie-Neige Chapel
/// @date   2018/05/03



#include "kltTracker.h"



//---------------------------------------------------------------------------------------
void KltTracker::updateFeaturePointPositions(const std::vector<uint>& fp_idx,
                                             const std::vector<cv::Point2f>& fp_t,
                                             const std::vector<uchar>& status) noexcept
{
    uint count = 0;
    uint status_size = status.size();
    for(uint i=0; i<status_size; i++)
    {
        if(status[i] == 0/* || isOutOfImage()*/)
            continue;

        cv::Mat fp((cv::Point2d)fp_t[i]);
        data->updateFeaturePoint2DPosition(fp_idx[i], fp);
        count++;
    }

    std::cout << "Nb feature points updated: " << count << std::endl;
}



//---------------------------------------------------------------------------------------
void KltTracker::compute() noexcept
{
    cv::Mat img_t   = data->getImageAtT();
    cv::cvtColor(img_t, img_t, CV_RGB2GRAY);

    cv::Mat img_tm1 = data->getImageAtTM1();
    cv::cvtColor(img_tm1, img_tm1, CV_RGB2GRAY);

    std::vector<uint> fp_idx;
    std::vector<cv::Point2f> fp_t;
    std::vector<cv::Point2f> fp_tm1;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::Size win_size = cv::Size(21,21);
    int max_level = 3;
    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

    getFeaturePointAtTM1(fp_idx, fp_tm1);
    std::cout << fp_tm1.size() << std::endl;
    cv::calcOpticalFlowPyrLK(img_tm1, img_t, fp_tm1, fp_t, status, err, win_size, max_level, criteria);
    updateFeaturePointPositions(fp_idx, fp_t, status);
}



//---------------------------------------------------------------------------------------
std::string KltTracker::getComponentName() const noexcept
{
    return "KltTracker";
}



//---------------------------------------------------------------------------------------
void KltTracker::getFeaturePointAtTM1(std::vector<uint>& fp_idx,
                                      std::vector<cv::Point2f>& fp_tm1) const noexcept
{
    for(uint i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        if(!data->isFeaturePointFeatureTrackerOld(i))
            continue;

        cv::Point2f fp(data->getFeaturePointPosition2DAtTM1(i));
        fp_tm1.push_back(fp);
        fp_idx.push_back(i);
    }
}



//---------------------------------------------------------------------------------------
bool KltTracker::isEnoughTimeElapsed() const noexcept
{
    return true;
}



//---------------------------------------------------------------------------------------
void KltTracker::readFileData() noexcept
{
}
