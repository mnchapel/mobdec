/// @file   kmeansOpticalFlow.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <component/clustering/kmeansOpticalFlow.h>



//---------------------------------------------------------------------------------------
void KMeansOpticalFlow::compute() noexcept
{
	selectPointForEssentialMatrixNister();
}



//---------------------------------------------------------------------------------------
std::string KMeansOpticalFlow::getComponentName() const noexcept
{
    return "KMeansOpticalFlow";
}



//---------------------------------------------------------------------------------------
void KMeansOpticalFlow::getStaticFeaturePointOld(cv::Mat& feature_point) const noexcept
{
	std::vector<cv::Point2f> fp_tmp;

	for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
	{
		if(!data->isFeaturePointOpticalFlowOld(i)
		|| !data->isFeaturePointStatic(i))
			continue;

		cv::Point2f point_t(*(data->getFeaturePointPosition2DAtT(i)));
		fp_tmp.push_back(point_t);
	}

	feature_point = cv::Mat(fp_tmp);

	DEBUG_COMPONENT_MSG("nb static points " << fp_tmp.size());
	DEBUG_COMPONENT_MSG("nb static feature points " << feature_point.size());
}



//---------------------------------------------------------------------------------------
bool KMeansOpticalFlow::isEnoughTimeElapsed() const noexcept
{
	if(data->getTimeElapsed() > CST(int,cst::DELTA))
		return true;
	return false;
}



//---------------------------------------------------------------------------------------
void KMeansOpticalFlow::readFileData() noexcept
{
}



//---------------------------------------------------------------------------------------
void KMeansOpticalFlow::selectPointForEssentialMatrixNister() noexcept
{
	cv::Mat feature_point;
	int K = 6;
	cv::Mat labels;
	cv::Mat centers;

	getStaticFeaturePointOld(feature_point);

	cv::kmeans(feature_point.t(), K, labels, cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 10, 1.0), 3, cv::KMEANS_PP_CENTERS, centers);

	DEBUG_COMPONENT_MSG(centers.size());

#ifndef NDEBUG
    	DataWriter::paintKMeansClusterCenters(data, getComponentName(), centers);
#endif

}
