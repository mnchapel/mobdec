/// @file   data.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <core/data.h>



//---------------------------------------------------------------------------------------
Data::Data(Video& video,
		   Camera& camera) noexcept
	: video{video}
    , camera{camera}
{
		start_time = CST(int,cst::START_TIME);
		time = start_time;
		feature_point.resize(CST(int,cst::NB_MAX_POINT));
		this->video.readImage(time);
}



//---------------------------------------------------------------------------------------
void Data::addNewFeaturePoint(cv::Mat& position_2d) noexcept
{
	uint free_index = findFirstFeaturePointFreeSpace();

	assert(free_index != uint(-1));

	feature_point[free_index] = FeaturePoint(position_2d);
}



//---------------------------------------------------------------------------------------
void Data::clearStaticFeaturePoint() noexcept
{
	static_feature_point.clear();
}



//---------------------------------------------------------------------------------------
void Data::deleteFeaturePoint(uint index) noexcept
{
	feature_point[index].deletePoint();
}



//---------------------------------------------------------------------------------------
void Data::featurePointReturnToSave(uint id,
									bool has_match,
									cv::Mat& position_2d,
									double distance_to_camera) noexcept
{
	feature_point[id].returnTo2DPositionSavedAtTMDelta(has_match, position_2d, distance_to_camera);

	if(feature_point[id].getLabel() == 'S'
	&& !has_match)
	{
//		std::cout << "remove " << id << std::endl;
		removeStaticPoint(id);
		deleteFeaturePoint(id);
	}
	if(!has_match)
		deleteFeaturePoint(id);
}



//---------------------------------------------------------------------------------------
uint Data::findFirstFeaturePointFreeSpace() noexcept
{
    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
        if(feature_point[i].getAge() == 0)
            return i;

    return -1;
}



//---------------------------------------------------------------------------------------
uint Data::getEndInitialisationTime() const noexcept
{
	return end_initialisation_time;
}



//---------------------------------------------------------------------------------------
uint Data::getEndProcessTime() const noexcept
{
	return end_process_time;
}



//---------------------------------------------------------------------------------------
uint Data::getFeaturePointAge(uint index) const noexcept
{
	return feature_point[index].getAge();
}



//---------------------------------------------------------------------------------------
double Data::getFeaturePointConfidenceValue(uint index) const noexcept
{
	return feature_point[index].getConfidenceValue();
}



//---------------------------------------------------------------------------------------
double Data::getFeaturePointDistanceToCameraAtT(uint index) const noexcept
{
	return feature_point[index].getDistanceToCameraAtT();
}



//---------------------------------------------------------------------------------------
double Data::getFeaturePointDistanceToCameraAtTMDelta(uint index) const noexcept
{
	return feature_point[index].getDistanceToCameraAtTMDelta();
}



//---------------------------------------------------------------------------------------
double Data::getFeaturePointLabel(uint index) const noexcept
{
	return feature_point[index].getLabel();
}



//---------------------------------------------------------------------------------------
cv::Mat Data::getFeaturePointOpticalFlowTM1(uint index) const noexcept
{
	return feature_point[index].getOpticalFlowTM1();
}



//---------------------------------------------------------------------------------------
cv::Mat Data::getFeaturePointOpticalFlowTMDelta(uint index) const noexcept
{
	return feature_point[index].getOpticalFlowTMDelta();
}



//---------------------------------------------------------------------------------------
std::vector<cv::Mat>::const_iterator Data::getFeaturePointPosition2DAtT(uint id) const noexcept
{
	return feature_point[id].get2DPositionAtT();
}



// TODO remove
//---------------------------------------------------------------------------------------
std::vector<cv::Mat>::const_iterator Data::testFunc(uint id) const noexcept
{
	return feature_point[id].testFunc();
}



//---------------------------------------------------------------------------------------
const cv::Mat& Data::getFeaturePointPosition2DAtTM1(uint id) const noexcept
{
	return feature_point[id].get2DPositionAtTM1();
}



//---------------------------------------------------------------------------------------
const cv::Mat& Data::getFeaturePointPosition2DAtTMDelta(uint id) const noexcept
{
	return feature_point[id].get2DPositionAtTMDelta();
}



////---------------------------------------------------------------------------------------
//const cv::Mat Data::getFeaturePointPosition3DAt(uint t
//											   ,uint id) const noexcept
//{
//	const cv::Mat& intrinsic = camera.getIntrinsicParameters();
//
//	cv::Mat feature_point_homog(3,1,CV_64F);
//	feature_point_homog.at<double>(0) = feature_point[id].get2DPositionAt(t).at<double>(0);
//	feature_point_homog.at<double>(1) = feature_point[id].get2DPositionAt(t).at<double>(1);
//	feature_point_homog.at<double>(2) = 1.;
//
//	cv::Mat homogeneous_point = intrinsic.inv() * feature_point_homog;
//	homogeneous_point /= cv::norm(homogeneous_point);
//
//	return homogeneous_point * feature_point[id].getDistanceToCameraAt(t);
//}



//---------------------------------------------------------------------------------------
std::vector<cv::Mat>::const_iterator Data::getFeaturePointPosition3DAt(uint t
											   	  	  	  	  	  	  ,uint id) const noexcept
{
	return feature_point[id].get3DPositionAt(t);
}



//---------------------------------------------------------------------------------------
const cv::Mat Data::getFeaturePointPosition3DAtT_Old(uint id) const noexcept
{
	const cv::Mat& intrinsic = camera.getIntrinsicParameters();

	cv::Mat feature_point_homog(3,1,CV_64F);
	cv::Mat fp = *(feature_point[id].get2DPositionAtT());
	feature_point_homog.at<double>(0) = fp.at<double>(0);
	feature_point_homog.at<double>(1) = fp.at<double>(1);
	feature_point_homog.at<double>(2) = 1.;

	cv::Mat homogeneous_point = intrinsic.inv() * feature_point_homog;
	homogeneous_point /= cv::norm(homogeneous_point);

	return homogeneous_point * feature_point[id].getDistanceToCameraAtT();
}



//---------------------------------------------------------------------------------------
std::vector<cv::Mat>::const_iterator Data::getFeaturePointPosition3DAtT(uint id) const noexcept
{
	return feature_point[id].get3DPositionAtT();
}



//---------------------------------------------------------------------------------------
std::vector<cv::Mat>::const_iterator Data::getFeaturePointPosition3DAtTM1(uint id) const noexcept
{
	return feature_point[id].get3DPositionAtTM1();
}



//---------------------------------------------------------------------------------------
std::vector<cv::Mat>::const_iterator Data::getFeaturePointPosition3DAtTMDelta(uint id) const noexcept
{
	return feature_point[id].get3DPositionAtTMDelta();
}



//---------------------------------------------------------------------------------------
const FeaturePoint& Data::getFeaturePoint(uint id) const noexcept
{
	return feature_point[id];
}



//---------------------------------------------------------------------------------------
const cv::Mat& Data::getImageAtT() const noexcept
{
	return video.getImageAtT();
}



//---------------------------------------------------------------------------------------
const cv::Mat& Data::getImageAtTM1() const noexcept
{
	return video.getImageAtTM1();
}



//---------------------------------------------------------------------------------------
const uint Data::getImageCol() const noexcept
{
	return video.getImageCol();
}



//---------------------------------------------------------------------------------------
const uint Data::getImageRow() const noexcept
{
	return video.getImageRow();
}



//---------------------------------------------------------------------------------------
const cv::Mat& Data::getIntrinsicParameters() const noexcept
{
	return camera.getIntrinsicParameters();
}



//---------------------------------------------------------------------------------------
uint Data::getLastGoodReconstruction() const noexcept
{
	return last_good_reconstruction;
}



//---------------------------------------------------------------------------------------
uint Data::getNbStaticPoint() const noexcept
{
	return static_feature_point.size();
}



//---------------------------------------------------------------------------------------
double Data::getReconstructionQualityAtT() const noexcept
{
	return reconstruction_quality.back();
}



//---------------------------------------------------------------------------------------
const cv::Mat& Data::getRotationAtT() const noexcept
{
    return camera.getRotation(utils::indexAtT(time));
}



//---------------------------------------------------------------------------------------
std::vector<uint> Data::getStaticPointIndex() const noexcept
{
	return static_feature_point;
}



//---------------------------------------------------------------------------------------
std::vector<uint>::iterator Data::getStaticPointIndexBegin() noexcept
{
	return static_feature_point.begin();
}



//---------------------------------------------------------------------------------------
std::vector<uint>::iterator Data::getStaticPointIndexEnd() noexcept
{
	return static_feature_point.end();
}



//---------------------------------------------------------------------------------------
const std::vector<uint>& Data::getSaveTime() const noexcept
{
	return save_time;
}



//---------------------------------------------------------------------------------------
uint Data::getStartTime() const noexcept
{
	return start_time;
}



//---------------------------------------------------------------------------------------
uint Data::getTime() const noexcept
{
    return time;
}



//---------------------------------------------------------------------------------------
uint Data::getTimeElapsed() const noexcept
{
    return time - start_time;
}



//---------------------------------------------------------------------------------------
const cv::Mat& Data::getTranslationAtT() const noexcept
{
    return camera.getTranslation(utils::indexAtT(time));
}



//---------------------------------------------------------------------------------------
bool Data::hasPosition3DComputedAtTM1(uint feature_point_id) const noexcept
{
	return feature_point[feature_point_id].hasPosition3DComputedAtTM1();
}



//---------------------------------------------------------------------------------------
bool Data::hasPosition3DComputedAtTMDelta(uint feature_point_id) const noexcept
{
	return feature_point[feature_point_id].hasPosition3DComputedAtTMDelta();
}



//---------------------------------------------------------------------------------------
bool Data::hasTMDeltaMatching(uint id) const noexcept
{
	return feature_point[id].hasValidTMDeltaMatch();
//	const cv::Mat& pos = feature_point[id].get2DPositionAtTMDelta();
//	if(pos.at<double>(0) == 0
//	&& pos.at<double>(1) == 0)
//		return false;
//	return true;
}



//---------------------------------------------------------------------------------------
bool Data::isFeaturePoint(uint id) const noexcept
{
	if(feature_point[id].getAge()>0)
		return true;
	return false;
}



//---------------------------------------------------------------------------------------
bool Data::isFeaturePointOld(uint id) const noexcept
{
	if(feature_point[id].getAge()> CST(int,cst::DELTA))
		return true;
	return false;
}



//---------------------------------------------------------------------------------------
bool Data::isFeaturePointOpticalFlowOld(uint id) const noexcept
{
	if(feature_point[id].getAge()>CST(int,cst::DELTA))
		return true;
	return false;
}



//---------------------------------------------------------------------------------------
bool Data::isFeaturePointReconstructionOld(uint id) const noexcept
{
	if(feature_point[id].getAge()>CST(int,cst::DELTA))
		return true;
	return false;
}



//---------------------------------------------------------------------------------------
bool Data::isFeaturePointScaleOld(uint id) const noexcept
{
	if(feature_point[id].getAge()>(CST(int,cst::DELTA)+1))
		return true;
	return false;
}



//---------------------------------------------------------------------------------------
bool Data::isFeaturePointTwoReconstructionsOld(uint id) const noexcept
{
	if(feature_point[id].getAge()>(CST(int,cst::DELTA)*2))
		return true;
	return false;
}



//---------------------------------------------------------------------------------------
bool Data::isFeaturePointStatic(uint index) const noexcept
{
    if(isFeaturePointOld(index) && (feature_point[index].getLabel() == Labeling::STATIC))
        return true;

    return false;
}



//---------------------------------------------------------------------------------------
void Data::nextFrame() noexcept
{
	time++;
	video.readImage(time);

	for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
	{
		if(!isFeaturePoint(i))
			continue;

        updateFeaturePointAge(i);
	}
}



//---------------------------------------------------------------------------------------
void Data::removeStaticPoint(uint feature_point_id) noexcept
{
	uint i;
	for(i=0; i<static_feature_point.size(); i++)
	{
		if(static_feature_point[i] == feature_point_id)
			break;
	}

	if(i<static_feature_point.size())
		static_feature_point.erase(static_feature_point.begin()+i);
}



//---------------------------------------------------------------------------------------
void Data::saveCurrentState() noexcept
{
	uint frame = time-CST(int,cst::DELTA);
	std::cout << "save time " << frame << std::endl;
	save_time.push_back(frame);
}



//---------------------------------------------------------------------------------------
void Data::setLastGoodReconstruction(uint frame_num) noexcept
{
	std::cout << __FUNCTION__ << " " << frame_num << std::endl;
	last_good_reconstruction = frame_num;
}



//---------------------------------------------------------------------------------------
void Data::updateConfidenceValue(uint index,
								 double bonus_malus) noexcept
{
	feature_point[index].updateConfidenceValue(bonus_malus);
	char label = feature_point[index].getLabel();

	if(label == 'S')
		static_feature_point.push_back(index);
}



//---------------------------------------------------------------------------------------
void Data::updateExtrinsicParameters(cv::Mat& rotation,
							   	   	 cv::Mat& translation) noexcept
{
    camera.updateExtrinsicParameters(utils::indexAtT(time), rotation, translation);
}



//---------------------------------------------------------------------------------------
void Data::updateFeaturePoint2DPosition(uint index,
										const cv::Mat& position_2d) noexcept
{
	feature_point[index].update2DPosition(position_2d);
}



//---------------------------------------------------------------------------------------
void Data::updateFeaturePointAge(uint index) noexcept
{
	feature_point[index].updateAge();
}



//---------------------------------------------------------------------------------------
void Data::updateFeaturePointDistanceToCamera(uint index,
											  double distance_to_camera) noexcept
{
	const cv::Mat& intrinsic = camera.getIntrinsicParameters();

	cv::Mat feature_point_homog(3,1,CV_64F);
	cv::Mat fp = *(feature_point[index].get2DPositionAtT());
	feature_point_homog.at<double>(0) = fp.at<double>(0);
	feature_point_homog.at<double>(1) = fp.at<double>(1);
	feature_point_homog.at<double>(2) = 1.;

	cv::Mat homogeneous_point = intrinsic.inv() * feature_point_homog;
	homogeneous_point /= cv::norm(homogeneous_point);

	cv::Mat position_3d = homogeneous_point * distance_to_camera;

	feature_point[index].updateDistanceToCamera(distance_to_camera);
	feature_point[index].update3DPosition(position_3d);
}



//---------------------------------------------------------------------------------------
void Data::updateReconstructionQuality(double quality) noexcept
{
//	if(reconstruction_quality.size() == time-end_initialisation_time)
//	{
//		std::cout << "new quality" << std::endl;
	std::cout << __FUNCTION__ << " " << quality << std::endl;
		reconstruction_quality.push_back(quality);
//	}
////	else
//	{
//		std::cout << "update last quality" << std::endl;
//		reconstruction_quality[reconstruction_quality.size()-1] = quality;
//	}
}

