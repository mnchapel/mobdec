/// @file   reconstruction3dTriangulation.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <component/reconstruction/reconstruction3dTriangulation.h>



//---------------------------------------------------------------------------------------
void Reconstruction3dTriangulation::compute() noexcept
{
    std::vector<uint> feature_id;
    std::vector<cv::Point2d> feature_point_t;
    std::vector<cv::Point2d> feature_point_tmdelta;
    cv::Mat triangulate_point;

    getFeaturePointReconstructionOld(feature_id, feature_point_t, feature_point_tmdelta);

	triangulate(feature_point_t, feature_point_tmdelta, triangulate_point);
	updateData(feature_id, triangulate_point);
}



//---------------------------------------------------------------------------------------
void Reconstruction3dTriangulation::computeProjectionMatrix(cv::Mat& projection_matrix) noexcept
{
    const cv::Mat& intrinsic_parameter = data->getIntrinsicParameters();
    const cv::Mat& rotation    = data->getRotationAtT();
    const cv::Mat& translation = data->getTranslationAtT();

    cv::Mat extrinsic_parameter;
    cv::hconcat(rotation, translation, extrinsic_parameter);

    projection_matrix = intrinsic_parameter * extrinsic_parameter;
}



//---------------------------------------------------------------------------------------
void Reconstruction3dTriangulation::computeProjectionMatrixIdentity(cv::Mat& projection_matrix_identity) noexcept
{
    const cv::Mat& intrinsic_parameter = data->getIntrinsicParameters();
    cv::Mat extrinsic_parameter_identity = cv::Mat::eye(3,4,CV_64F);

    projection_matrix_identity = intrinsic_parameter * extrinsic_parameter_identity;
}



//---------------------------------------------------------------------------------------
std::string Reconstruction3dTriangulation::getComponentName() const noexcept
{
    return "Reconstruction3dTriangulation";
}



//---------------------------------------------------------------------------------------
void Reconstruction3dTriangulation::getFeaturePointReconstructionOld(std::vector<uint>& feature_id,
									     	 	 	   	   	   	   	 std::vector<cv::Point2d>& feature_point_t,
																	 std::vector<cv::Point2d>& feature_point_tmdelta) const noexcept
{
    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        if(!data->isFeaturePointOld(i)
        || !data->hasTMDeltaMatching(i))
            continue;

        cv::Point2d point_t(*(data->getFeaturePointPosition2DAtT(i)));
        cv::Point2d point_tmdelta(data->getFeaturePointPosition2DAtTMDelta(i));

        feature_id.push_back(i);
        feature_point_t.push_back(point_t);
        feature_point_tmdelta.push_back(point_tmdelta);
    }

    assert(feature_id.size() == feature_point_t.size());
    assert(feature_point_t.size() == feature_point_tmdelta.size());
}



//---------------------------------------------------------------------------------------
std::string Reconstruction3dTriangulation::getPhaseReturn() const noexcept
{
	return "NextPhase";
}



//---------------------------------------------------------------------------------------
bool Reconstruction3dTriangulation::isEnoughTimeElapsed() const noexcept
{
    if(data->getTimeElapsed() >= CST(int,cst::DELTA))
        return true;

    return false;
}



//---------------------------------------------------------------------------------------
void Reconstruction3dTriangulation::readFileData() noexcept
{

}



//---------------------------------------------------------------------------------------
void Reconstruction3dTriangulation::triangulate(const std::vector<cv::Point2d>& feature_point_t,
												const std::vector<cv::Point2d>& feature_point_tmdelta,
												cv::Mat& triangulate_point) noexcept
{
    cv::Mat projection_matrix;
    cv::Mat projection_matrix_identity;

    computeProjectionMatrixIdentity(projection_matrix_identity);
    computeProjectionMatrix(projection_matrix);

    cv::triangulatePoints(projection_matrix_identity, projection_matrix, feature_point_t, feature_point_tmdelta, triangulate_point);

    assert(feature_point_t.size() == triangulate_point.cols);
}



//---------------------------------------------------------------------------------------
void Reconstruction3dTriangulation::updateData(const std::vector<uint>& feature_id,
											   const cv::Mat& triangulate_point) noexcept
{
    for(int j=0; j<triangulate_point.cols; j++)
    {
        uint index = feature_id[j];

        cv::Point3d point;
        point.x = triangulate_point.at<double>(0,j)/triangulate_point.at<double>(3,j);
        point.y = triangulate_point.at<double>(1,j)/triangulate_point.at<double>(3,j);
        point.z = triangulate_point.at<double>(2,j)/triangulate_point.at<double>(3,j);

        double distance = cv::norm(point);
        data->updateFeaturePointDistanceToCamera(index, distance);
    }
}



















