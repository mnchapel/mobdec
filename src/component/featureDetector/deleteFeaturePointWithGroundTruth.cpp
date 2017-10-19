/// @file   deleteFeaturePointWithGroundTruth.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <component/featureDetector/deleteFeaturePointWithGroundTruth.h>



//---------------------------------------------------------------------------------------
void DeleteFeaturePointWithGroundTruth::compute() noexcept
{
    std::vector<uint> id_moving_feature_point;
    findMovingFeaturePoint(id_moving_feature_point);
    deleteFeaturePoint(id_moving_feature_point);

#ifndef NDEBUG
    DataWriter::paintFeaturePoint(data, getComponentName());
#endif
}



//---------------------------------------------------------------------------------------
std::string DeleteFeaturePointWithGroundTruth::getComponentName() const noexcept
{
    return "DeleteFeaturePointWithGroundTruth";
}



//---------------------------------------------------------------------------------------
void DeleteFeaturePointWithGroundTruth::findMovingFeaturePoint(std::vector<uint>& id_moving_feature_point) noexcept
{
	char buffer[500];
	sprintf(buffer, CST(std::string,cst::TEMPLATE_GT_IMAGE_PATH).c_str(), data->getTime());
	DEBUG_COMPONENT_MSG("buffer " << buffer);
    cv::Mat ground_truth = cv::imread(buffer);

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        if(!data->isFeaturePoint(i))
            continue;

        cv::Mat feature_point = *(data->getFeaturePointPosition2DAtT(i));

        assert(ground_truth.rows>0 && ground_truth.cols>0);

        if(ground_truth.at<cv::Vec3b>(feature_point.at<double>(1), feature_point.at<double>(0)) != cv::Vec3b(0,0,0))
            id_moving_feature_point.push_back(i);
    }
    DEBUG_COMPONENT_MSG("nb point to delete " << id_moving_feature_point.size());
}



//---------------------------------------------------------------------------------------
void DeleteFeaturePointWithGroundTruth::deleteFeaturePoint(const std::vector<uint>& id_moving_feature_point) noexcept
{
    for(auto& i: id_moving_feature_point)
        data->deleteFeaturePoint(i);
}



//---------------------------------------------------------------------------------------
bool DeleteFeaturePointWithGroundTruth::isEnoughTimeElapsed() const noexcept
{
    return true;
}



//---------------------------------------------------------------------------------------
void DeleteFeaturePointWithGroundTruth::readFileData() noexcept
{

}






















