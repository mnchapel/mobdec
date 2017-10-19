/// @file   essentialMatrix.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <component/motionEstimation/essentialMatrix.h>



//---------------------------------------------------------------------------------------
void EssentialMatrix::compute() noexcept
{
    break_life_cycle = false;

    srand(time(NULL));

    std::vector<cv::Point2d> static_feature_point_t;
    std::vector<cv::Point2d> static_feature_point_tmdelta;

	getStaticFeaturePoint(static_feature_point_t, static_feature_point_tmdelta);

	computeExtrinsicParameters(static_feature_point_t, static_feature_point_tmdelta);

#ifndef NDEBUG
    DataWriter::paintConfidenceValue(data, getComponentName());
    DataWriter::writeDistanceToCamera(data, getComponentName());
#endif
}



//---------------------------------------------------------------------------------------
std::string EssentialMatrix::getComponentName() const noexcept
{
    return "EssentialMatrix";
}



//---------------------------------------------------------------------------------------
std::string EssentialMatrix::getPhaseReturn() const noexcept
{
	return "CombiningGeometricConstraintOpticalFlow";
}



//---------------------------------------------------------------------------------------
void EssentialMatrix::computeExtrinsicParameters(const std::vector<cv::Point2d>& feature_point_t,
											     const std::vector<cv::Point2d>& feature_point_tmdelta) noexcept
{
    cv::Mat rotation;
    cv::Mat translation;

//    std::vector<cv::Point2d> fp_t;
//    std::vector<cv::Point2d> fp_tmdelta;
//    for(int i=0; i<6; i++)
//    {
//    	fp_t.push_back(feature_point_t[i]);
//    	fp_tmdelta.push_back(feature_point_tmdelta[i]);
//    }

    cv::Mat essential_matrix = computeEssentialMatrix(feature_point_t, feature_point_tmdelta);
    computeRotationTranslation(essential_matrix, feature_point_t, feature_point_tmdelta, rotation, translation);

#ifndef NDEBUG
    DataWriter::writeExtrinsicParameters(data, getComponentName(), rotation, translation);
#endif
}



//---------------------------------------------------------------------------------------
cv::Mat EssentialMatrix::computeEssentialMatrix(const std::vector<cv::Point2d>& feature_point_t,
											    const std::vector<cv::Point2d>& feature_point_tmdelta) noexcept
{
    double focal = data->getIntrinsicParameters().at<double>(0,0);
    cv::Point2d camera_principal_point(data->getImageCol()/2, data->getImageRow()/2);

    cv::Mat essential_matrix = cv::findEssentialMat(feature_point_t, feature_point_tmdelta, focal, camera_principal_point, cv::LMEDS);

    return essential_matrix;
}



//---------------------------------------------------------------------------------------
void EssentialMatrix::computeRotationTranslation(const cv::Mat& essential_matrix,
											     const std::vector<cv::Point2d>& feature_point_t,
											     const std::vector<cv::Point2d>& feature_point_tmdelta,
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
bool EssentialMatrix::isEnoughTimeElapsed() const noexcept
{
	if(data->getTimeElapsed() >= CST(int,cst::DELTA))
		return true;
	return false;
}



//---------------------------------------------------------------------------------------
void EssentialMatrix::getStaticFeaturePoint(std::vector<cv::Point2d>& static_feature_point_t,
											std::vector<cv::Point2d>& static_feature_point_tmdelta) const noexcept
{
//    auto static_point_index_begin = data->getStaticPointIndexBegin();
//    auto static_point_index_end = data->getStaticPointIndexEnd();
    std::vector<uint> static_feature_point = data->getStaticPointIndex();

//    for(int i=0; i<NB_MAX_POINT; i++)
//    {
//        if(!data->isFeaturePointOld(i)
//         ||!data->isFeaturePointStatic(i))
//            continue;
//
//        if(!isTooNearToOtherFeaturePoint(i))
//        	feature_point_static_id.push_back(i);
//    }

//    assert(feature_point_static_id.size()>6);

//    std::cout << "total static point " << feature_point_static_id.size() << std::endl;


//    static_feature_point.push_back(962);
//    static_feature_point.push_back(4381);
//    static_feature_point.push_back(4629);
//    static_feature_point.push_back(804);
//    static_feature_point.push_back(3059);
//    static_feature_point.push_back(3903);

//    uint nb_point = 6;
//    std::cout << "nb point static " << nb_point << std::endl;
//    random_unique(static_feature_point.begin(), static_feature_point.end(), nb_point);

//	for(size_t i=0; i<nb_point; i++)
    for(int i=0; i<static_feature_point.size(); i++)
	{
		uint feature_point_id = static_feature_point[i];
		if(!data->isFeaturePointOpticalFlowOld(feature_point_id))
			continue;

//		assert(!data->hasTMDeltaMatching(feature_point_id));

		cv::Point2f point_t(*(data->getFeaturePointPosition2DAtT(feature_point_id)));
		cv::Point2f point_tmdelta(data->getFeaturePointPosition2DAtTMDelta(feature_point_id));

		static_feature_point_t.push_back(point_t);
		static_feature_point_tmdelta.push_back(point_tmdelta);
    }

    DEBUG_COMPONENT_MSG("Nb point to static " << static_feature_point_t.size());
}



//---------------------------------------------------------------------------------------
void EssentialMatrix::readFileData() noexcept
{

}



//---------------------------------------------------------------------------------------
bool EssentialMatrix::isTooNearToOtherFeaturePoint(uint index) const noexcept
{
	for(int j=0; j<CST(int,cst::NB_MAX_POINT); j++)
	{
		if(data->isFeaturePointStatic(j))
			continue;

		cv::Mat feature_point_static = *(data->getFeaturePointPosition2DAtT(index));
		cv::Mat feature_point_other  = *(data->getFeaturePointPosition2DAtT(j));

		if(cv::norm(feature_point_static - feature_point_other) < 15)
			return true;
	}

	return false;
}



//---------------------------------------------------------------------------------------
std::vector<uint>::iterator EssentialMatrix::random_unique(std::vector<uint>::iterator begin,
														   std::vector<uint>::iterator end,
														   std::size_t num_random) const noexcept
{
	std::size_t left = std::distance(begin, end);
	while(num_random--)
	{
		std::vector<uint>::iterator r = begin;
		std::advance(r, rand()%left);

		if(!data->isFeaturePointOpticalFlowOld(*r))
		{
			num_random++;
			continue;
		}

		std::swap(*begin, *r);
		++begin;
		--left;
	}

	return begin;
}



















