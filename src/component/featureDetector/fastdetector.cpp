/// @file   fastDetector.cpp
/// @author Marie-Neige Chapel
/// @date   2018/05/03



#include "fastdetector.h"



//---------------------------------------------------------------------------------------
void FastDetector::compute() noexcept
{
    // Load image & convert to grayscale
    cv::Mat img_t   = data->getImageAtT();
    cv::cvtColor(img_t, img_t, CV_RGB2GRAY);

    std::vector<cv::KeyPoint> keypoints;
    int threshold = 20;
    std::vector<cv::Point2f> feature_points;
    cv::FAST(img_t, keypoints, threshold, true);
//    cv::KeyPointsFilter::retainBest(keypoints, 1000);
    cv::KeyPoint::convert(keypoints, feature_points, std::vector<int>());

    for(auto& fp: feature_points)
    {
        cv::Mat point(fp);
        data->addNewFeaturePoint(point);
    }

#ifndef NDEBUG
    DataWriter::writeFeaturePointPositionAge(data, getComponentName());
    DataWriter::paintFeaturePoint(data, getComponentName());
#endif
}



//---------------------------------------------------------------------------------------
std::string FastDetector::getComponentName() const noexcept
{
    return "FastDetector";
}



//---------------------------------------------------------------------------------------
bool FastDetector::isEnoughTimeElapsed() const noexcept
{
    return true;
}



//---------------------------------------------------------------------------------------
void FastDetector::readFileData() noexcept
{

}
