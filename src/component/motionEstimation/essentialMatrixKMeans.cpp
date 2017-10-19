/// @file   essentialMatrixKMeans.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <component/motionEstimation/essentialMatrixKMeans.h>

#define K 6



//---------------------------------------------------------------------------------------
void EssentialMatrixKMeans::compute() noexcept
{
    break_life_cycle = false;

    srand(time(NULL));

    std::vector<cv::Point2f> static_feature_point_t;
    std::vector<cv::Point2f> static_feature_point_tmdelta;
    std::vector<cv::Point2f> selected_point_t;
    std::vector<cv::Point2f> selected_point_tmdelta;

	getStaticFeaturePoint(static_feature_point_t, static_feature_point_tmdelta);
	cv::Mat point(static_feature_point_t);

	selectPointForEssentialMatrixNister((cv::Mat)point.t(), selected_point_t);
	findCorrespondingTMDelta(static_feature_point_t, static_feature_point_tmdelta, selected_point_t, selected_point_tmdelta);
	computeExtrinsicParameters(selected_point_t, selected_point_tmdelta);

#ifndef NDEBUG
    DataWriter::paintConfidenceValue(data, getComponentName());
    DataWriter::writeDistanceToCamera(data, getComponentName());
#endif
}



//---------------------------------------------------------------------------------------
std::string EssentialMatrixKMeans::getComponentName() const noexcept
{
    return "EssentialMatrixKMeans";
}



//---------------------------------------------------------------------------------------
std::string EssentialMatrixKMeans::getPhaseReturn() const noexcept
{
    return "CombiningGeometricConstraintOpticalFlow";
}



//---------------------------------------------------------------------------------------
void EssentialMatrixKMeans::computeExtrinsicParameters(const std::vector<cv::Point2f>& feature_point_t,
											     	   const std::vector<cv::Point2f>& feature_point_tmdelta) noexcept
{
    cv::Mat rotation;
    cv::Mat translation;

//    for(int i=0; i<feature_point_t.size(); i++)
//    {
//    	DEBUG_COMPONENT_MSG(feature_point_t[i] << " " << feature_point_tmdelta[i]);
//    }

    cv::Mat essential_matrix = computeEssentialMatrix(feature_point_t, feature_point_tmdelta);
    computeRotationTranslation(essential_matrix, feature_point_t, feature_point_tmdelta, rotation, translation);

#ifndef NDEBUG
    DataWriter::writeExtrinsicParameters(data, getComponentName(), rotation, translation);
#endif
}



//---------------------------------------------------------------------------------------
cv::Mat EssentialMatrixKMeans::computeEssentialMatrix(const std::vector<cv::Point2f>& feature_point_t,
											    	  const std::vector<cv::Point2f>& feature_point_tmdelta) noexcept
{
    double focal = data->getIntrinsicParameters().at<double>(0,0);
    cv::Point2d camera_principal_point(data->getImageCol()/2, data->getImageRow()/2);

    cv::Mat essential_matrix = cv::findEssentialMat(feature_point_t, feature_point_tmdelta, focal, camera_principal_point, cv::RANSAC);

    return essential_matrix;
}



//---------------------------------------------------------------------------------------
void EssentialMatrixKMeans::computeRotationTranslation(const cv::Mat& essential_matrix,
											     	   const std::vector<cv::Point2f>& feature_point_t,
													   const std::vector<cv::Point2f>& feature_point_tmdelta,
													   cv::Mat& rotation,
													   cv::Mat& translation) noexcept
{
    double focal = data->getIntrinsicParameters().at<double>(0,0);
    cv::Point2d camera_principal_point(data->getImageCol()/2, data->getImageRow()/2);

    cv::recoverPose(essential_matrix, feature_point_t, feature_point_tmdelta, rotation, translation, focal, camera_principal_point);

    data->updateExtrinsicParameters(rotation, translation);

#ifndef NDEBUG
    cv::Mat rot(3,1,CV_64F);
    cv::Rodrigues(rotation, rot);
    DEBUG_COMPONENT_MSG("rotation "
              	  	    << rot.at<double>(0)*180./M_PI << " "
						<< rot.at<double>(1)*180./M_PI << " "
						<< rot.at<double>(2)*180./M_PI);

    DEBUG_COMPONENT_MSG("translation "
              	  	    << translation.at<double>(0) << " "
						<< translation.at<double>(1) << " "
						<< translation.at<double>(2));
#endif
}



//---------------------------------------------------------------------------------------
void EssentialMatrixKMeans::findCorrespondingTMDelta(const std::vector<cv::Point2f>& static_feature_point_t,
													 const std::vector<cv::Point2f>& static_feature_point_tmdelta,
													 const std::vector<cv::Point2f>& kmeans_centers_t,
													 std::vector<cv::Point2f>& kmeans_centers_tmdelta) const noexcept
{
	for(auto elt: kmeans_centers_t)
	{
		auto it = std::find(static_feature_point_t.begin(), static_feature_point_t.end(), elt);
		if(it!=static_feature_point_t.end())
		{
			auto pos = std::distance(static_feature_point_t.begin(), it);
			kmeans_centers_tmdelta.push_back(static_feature_point_tmdelta[pos]);
		}
		else
			std::cerr << "\tcenter is not in the list" << std::endl;
	}
}



//---------------------------------------------------------------------------------------
void EssentialMatrixKMeans::getStaticFeaturePoint(std::vector<cv::Point2f>& static_feature_point_t,
												  std::vector<cv::Point2f>& static_feature_point_tmdelta) const noexcept
{
    std::vector<uint> static_feature_point = data->getStaticPointIndex();

    for(int i=0; i<static_feature_point.size(); i++)
	{
		uint feature_point_id = static_feature_point[i];
		if(!data->isFeaturePointOpticalFlowOld(feature_point_id))
			continue;

		cv::Point2f point_t(*(data->getFeaturePointPosition2DAtT(feature_point_id)));
		cv::Point2f point_tmdelta(data->getFeaturePointPosition2DAtTMDelta(feature_point_id));

		static_feature_point_t.push_back(point_t);
		static_feature_point_tmdelta.push_back(point_tmdelta);
    }

    DEBUG_COMPONENT_MSG("Nb point to static " << static_feature_point_t.size());
}



//---------------------------------------------------------------------------------------
bool EssentialMatrixKMeans::isEnoughTimeElapsed() const noexcept
{
	if(data->getTimeElapsed() >= CST(int,cst::DELTA))
		return true;
	return false;
}



//---------------------------------------------------------------------------------------
void EssentialMatrixKMeans::readFileData() noexcept
{

}



//---------------------------------------------------------------------------------------
void EssentialMatrixKMeans::selectPointForEssentialMatrixNister(const cv::Mat& static_point_t,
																std::vector<cv::Point2f>& kmean_centers) noexcept
{
	cv::Mat labels;
	cv::Mat centers;

	// Compute kmeans
	cv::kmeans(static_point_t,
			   K,
			   labels,
			   cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 10, 1.0),
			   3,
			   cv::KMEANS_PP_CENTERS,
			   centers);

	// Get point in the list
	std::vector<int> near_id(centers.rows);
	std::vector<double> least_distance(centers.rows, 100);
	for(int i=0; i<labels.rows; i++)
	{
		int label = labels.at<int>(i);
		double dist = cv::norm(centers.at<cv::Point2f>(label) - static_point_t.at<cv::Point2f>(i));
		if(dist < least_distance[label])
		{
			least_distance[label] = dist;
			near_id[label] = i;
		}
	}

	// Fill kmean_centers
	kmean_centers.resize(K);
	for(int i=1; i<K; i++)
	{
		cv::Point2f point(static_point_t.at<cv::Vec2f>(near_id[i])[0], static_point_t.at<cv::Vec2f>(near_id[i])[1]);
		kmean_centers[i-1] = point;
	}
	cv::Point2f point(static_point_t.at<cv::Vec2f>(near_id[0])[0], static_point_t.at<cv::Vec2f>(near_id[0])[1]);
	kmean_centers[K-1] = point;


#ifndef NDEBUG
    	DataWriter::paintKMeansClusterCenters(data, getComponentName(), kmean_centers);
    	DataWriter::paintKMeansCluster(data, getComponentName(), K, static_point_t, labels);
#endif

}



















