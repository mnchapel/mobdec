/// @file   falseNegativeSuppression.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <component/labeling/falseNegativeSuppression.h>
#include <array>
#include <fstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#define DIST_NEAR_CLUSTER 40
#define MIN_CHILD_SIZE 25



//---------------------------------------------------------------------------------------
void FalseNegativeSuppression::compute() noexcept
{
    computeBis();
}



//---------------------------------------------------------------------------------------
std::string FalseNegativeSuppression::getComponentName() const noexcept
{
    return "FalseNegativeSuppression";
}



//---------------------------------------------------------------------------------------
void FalseNegativeSuppression::computeBis() noexcept
{
    std::vector<uint> cluster_label_value;
    std::vector< std::vector<uint> > cluster_optical_flow_list;
    std::vector<uint> false_negative_label_cluster;

    computeOpticalFlowClusterList(cluster_optical_flow_list);
    computeFalseNegativeSuppress(cluster_label_value, cluster_optical_flow_list, false_negative_label_cluster);

#ifndef NDEBUG
    DataWriter::writeStaticFeaturePointPositionAtT(data, getComponentName());
    DataWriter::writeStaticFeaturePointPositionAtTMDelta(data, getComponentName());
    DataWriter::paintConfidenceValue(data, getComponentName());
    DataWriter::paintCluster(data, getComponentName(), cluster_optical_flow_list);
    DataWriter::writeLabel(data, getComponentName());
    DataWriter::writeConfidenceValue(data, getComponentName());
#endif
}



//---------------------------------------------------------------------------------------
void FalseNegativeSuppression::computeFalseNegativeSuppress(const std::vector<uint>& cluster_label_value,
                                                            const std::vector< std::vector<uint> >& cluster_optical_flow_list,
                                                            std::vector<uint>& false_negative_label_cluster) noexcept
{
    uint nb_cluster_optical_flow_list = cluster_optical_flow_list.size();
    for(int i=0; i<nb_cluster_optical_flow_list; i++) // for each optical flow cluster
    {
        if(isFalseNegativeCluster(cluster_optical_flow_list[i]))
        {
            resetConfidenceIndex(cluster_optical_flow_list[i]);
        }
    }
}



//---------------------------------------------------------------------------------------
void FalseNegativeSuppression::computeOpticalFlowCluster(std::vector<bool>& isInCluster,
                                                         std::vector<uint>& cluster) noexcept
{
    uint index_start = cluster[0]+1;

    bool change = true;
    while(change)
    {
        change = false;
        for(int i=index_start; i<CST(int,cst::NB_MAX_POINT); i++)
        {
            if(!data->isFeaturePointOpticalFlowOld(i)
            || isInCluster[i])
                continue;

            for(int j=0; j<cluster.size(); j++)
            {
                if(isOpticalFlowNearToCluster(i, cluster[j])
                && isPointNearToCluster(i, cluster[j]))
                {
                    cluster.push_back(i);
                    isInCluster[i] = true;
                    change = true;
                    break;
                }
            }
        }
    }
}



//---------------------------------------------------------------------------------------
void FalseNegativeSuppression::computeOpticalFlowClusterList(std::vector< std::vector<uint> >& cluster_list) noexcept
{
    std::vector<bool> isInCluster(CST(int,cst::NB_MAX_POINT), false);

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        if(!data->isFeaturePointOpticalFlowOld(i)
        || isInCluster[i])
            continue;

        std::vector<uint> cluster;
        cluster.push_back(i);
        isInCluster[i] = true;

        computeOpticalFlowCluster(isInCluster, cluster);

        cluster_list.push_back(cluster);
    }
}



//---------------------------------------------------------------------------------------
bool FalseNegativeSuppression::isEnoughTimeElapsed() const noexcept
{	
    if(data->getTimeElapsed() >= (CST(int,cst::DELTA)*2))
        return true;

    return false;
}



//---------------------------------------------------------------------------------------
bool FalseNegativeSuppression::isFalseNegativeCluster(const std::vector<uint>& cluster_optical_flow) noexcept
{
    bool is_false_negative = false;

    uint nb_static = 0;
    uint nb_moving = 0;

    uint nb_point_in_cluster = cluster_optical_flow.size();
    for(int i=0; i<nb_point_in_cluster; i++)
    {
        uint point_index = cluster_optical_flow[i];
        uint point_label = data->getFeaturePointLabel(point_index);

        if(point_label == Labeling::STATIC)
        {
            nb_static++;
        }
        else if(point_label == Labeling::MOVING)
        {
            nb_moving++;
        }
    }

    if(nb_static > 5 && nb_moving > 0)
        is_false_negative = true;

    return is_false_negative;
}



//---------------------------------------------------------------------------------------
bool FalseNegativeSuppression::isOpticalFlowNearToCluster(uint i,
														  uint j) noexcept
{
	cv::Mat optf_1 = data->getFeaturePointOpticalFlowTMDelta(i);
	cv::Mat optf_2 = data->getFeaturePointOpticalFlowTMDelta(j);

    double angle_diff = acos((optf_1.dot(optf_2))/(cv::norm(optf_1) * cv::norm(optf_2))) * 180./M_PI;
    double magnitude_diff = fabs(cv::norm(optf_1)-cv::norm(optf_2));

    if(angle_diff <= 2 && magnitude_diff <=2)
        return true;
    return false;
}



//---------------------------------------------------------------------------------------
bool FalseNegativeSuppression::isPointNearToCluster(uint index_1,
                                                    uint index_2) noexcept
{
	cv::Mat point_1 = *(data->getFeaturePointPosition2DAtT(index_1));
	cv::Mat point_2 = *(data->getFeaturePointPosition2DAtT(index_2));

    double square_dist = cv::norm(point_1-point_2);

    if(square_dist <= (DIST_NEAR_CLUSTER))
        return true;
    return false;
}



//---------------------------------------------------------------------------------------
void FalseNegativeSuppression::readFileData() noexcept
{

}



//---------------------------------------------------------------------------------------
void FalseNegativeSuppression::resetConfidenceIndex(const std::vector<uint>& cluster_optical_flow) noexcept
{
    uint nb_feature_point = cluster_optical_flow.size();
    for(int i=0; i<nb_feature_point; i++)
    {
        uint index_feature_point = cluster_optical_flow[i];

        if(data->getFeaturePointLabel(index_feature_point) == MOVING)
        {
            double confidence_index = data->getFeaturePointConfidenceValue(index_feature_point);
            data->updateConfidenceValue(index_feature_point, confidence_index*-1);
        }
    }
}






