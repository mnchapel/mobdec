/// @file   featurePoint.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <core/featurePoint.h>



//---------------------------------------------------------------------------------------
FeaturePoint::FeaturePoint() noexcept
	: age(0)
	, confidence_value(0)
	, label(Labeling::UNLABELED)
{
	distance_to_camera.resize(CST(int,cst::NB_ROWS));
	distance_to_camera_saved.resize(CST(int,cst::NB_ROWS));
	test = std::vector<cv::Mat>(CST(int,cst::NB_ROWS), cv::Mat::zeros(2,1,CV_64F));
	position_2d = std::vector<cv::Mat>(CST(int,cst::NB_ROWS), cv::Mat::zeros(2,1,CV_64F));
	position_3d = std::vector<cv::Mat>(CST(int,cst::NB_ROWS), cv::Mat::zeros(3,1,CV_64F));
    has_position_3d = std::vector<bool>(CST(int,cst::NB_ROWS), false);

    this->distance_to_camera.assign(CST(int,cst::NB_ROWS),0);
}



//---------------------------------------------------------------------------------------
FeaturePoint::FeaturePoint(cv::Mat& position_2d) noexcept
	: age(1)
	, confidence_value(0)
	, label(Labeling::UNLABELED)
{
	distance_to_camera.resize(CST(int,cst::NB_ROWS));
	this->position_2d = std::vector<cv::Mat>(CST(int,cst::NB_ROWS), cv::Mat::zeros(2,1,CV_64F));
	this->position_3d = std::vector<cv::Mat>(CST(int,cst::NB_ROWS), cv::Mat::zeros(3,1,CV_64F));
	has_position_3d = std::vector<bool>(CST(int,cst::NB_ROWS), false);

    this->distance_to_camera.assign(CST(int,cst::NB_ROWS), 0);
	this->position_2d[0] = position_2d;
}



//---------------------------------------------------------------------------------------
void FeaturePoint::deletePoint() noexcept
{
	age = 0;
	confidence_value = 0;
    label = UNLABELED;

	position_2d.clear();
	position_3d.clear();

	position_2d = std::vector<cv::Mat>(CST(int,cst::NB_ROWS), cv::Mat::zeros(2,1,CV_64F));
	position_3d = std::vector<cv::Mat>(CST(int,cst::NB_ROWS), cv::Mat::zeros(3,1,CV_64F));

    distance_to_camera.assign(CST(int,cst::NB_ROWS), 0);
}



//---------------------------------------------------------------------------------------
const cv::Mat& FeaturePoint::get2DPositionAt(uint t) const noexcept
{
	uint old_age = (age+CST(int,cst::NB_ROWS)-t-1)%CST(int,cst::NB_ROWS);
	assert(old_age>=0 && old_age<CST(int,cst::NB_ROWS));

	return position_2d[old_age];
}



//---------------------------------------------------------------------------------------
std::vector<cv::Mat>::const_iterator FeaturePoint::get2DPositionAtT() const noexcept
{
    return position_2d.begin()+utils::indexAtT(age);
}



// TODO remove
//---------------------------------------------------------------------------------------
std::vector<cv::Mat>::const_iterator FeaturePoint::testFunc() const noexcept
{
    return test.begin()+utils::indexAtT(age);
}



//---------------------------------------------------------------------------------------
const cv::Mat& FeaturePoint::get2DPositionAtTM1() const noexcept
{
    return position_2d[utils::indexAtTM1(age)];
}



//---------------------------------------------------------------------------------------
const cv::Mat& FeaturePoint::get2DPositionAtTMDelta() const noexcept
{
    return position_2d[utils::indexAtTMDelta(age)];
}



//---------------------------------------------------------------------------------------
std::vector<cv::Mat>::const_iterator FeaturePoint::get3DPositionAtT() const noexcept
{
    return position_3d.begin()+utils::indexAtT(age);
}



//---------------------------------------------------------------------------------------
std::vector<cv::Mat>::const_iterator FeaturePoint::get3DPositionAtTM1() const noexcept
{
    return position_3d.begin()+utils::indexAtTM1(age);
}



//---------------------------------------------------------------------------------------
std::vector<cv::Mat>::const_iterator FeaturePoint::get3DPositionAtTMDelta() const noexcept
{
    return position_3d.begin()+utils::indexAtTMDelta(age);
}



//---------------------------------------------------------------------------------------
std::vector<cv::Mat>::const_iterator FeaturePoint::get3DPositionAt(uint t) const noexcept
{
	uint old_age = (age+CST(int,cst::NB_ROWS)-t-1)%CST(int,cst::NB_ROWS);
	assert(old_age>=0 && old_age<CST(int,cst::NB_ROWS));

	return position_3d.begin()+old_age;
}



//---------------------------------------------------------------------------------------
uint FeaturePoint::getAge() const noexcept
{
	return age;
}



//---------------------------------------------------------------------------------------
double FeaturePoint::getConfidenceValue() const noexcept
{
	return confidence_value;
}



//---------------------------------------------------------------------------------------
double FeaturePoint::getDistanceToCameraAt(uint t) const noexcept
{
	uint index = (age+CST(int,cst::NB_ROWS)-t-1)%CST(int,cst::NB_ROWS);
	assert(index>=0 && index<CST(int,cst::NB_ROWS));

	return distance_to_camera[index];
}



//---------------------------------------------------------------------------------------
double FeaturePoint::getDistanceToCameraAtT() const noexcept
{
    return distance_to_camera[utils::indexAtT(age)];
}



//---------------------------------------------------------------------------------------
double FeaturePoint::getDistanceToCameraAtTM1() const noexcept
{
    return distance_to_camera[utils::indexAtTM1(age)];
}



//---------------------------------------------------------------------------------------
double FeaturePoint::getDistanceToCameraAtTMDelta() const noexcept
{
    return distance_to_camera[utils::indexAtTMDelta(age)];
}



//---------------------------------------------------------------------------------------
char FeaturePoint::getLabel() const noexcept
{
	return label;
}



//---------------------------------------------------------------------------------------
cv::Mat FeaturePoint::getOpticalFlowTM1() const noexcept
{
    return position_2d[utils::indexAtT(age)] - position_2d[utils::indexAtTM1(age)];
}



//---------------------------------------------------------------------------------------
cv::Mat FeaturePoint::getOpticalFlowTMDelta() const noexcept
{
    return position_2d[utils::indexAtT(age)] - position_2d[utils::indexAtTMDelta(age)];
}



//---------------------------------------------------------------------------------------
bool FeaturePoint::hasPosition3DComputedAtTM1() const noexcept
{
    return has_position_3d[utils::indexAtTM1(age)];
}



//---------------------------------------------------------------------------------------
bool FeaturePoint::hasPosition3DComputedAtTMDelta() const noexcept
{
    return has_position_3d[utils::indexAtTMDelta(age)];
}



//---------------------------------------------------------------------------------------
bool FeaturePoint::hasValidTMDeltaMatch() const noexcept
{
	return valid_tmdelta_match;
}



//---------------------------------------------------------------------------------------
void FeaturePoint::returnTo2DPositionSavedAtTMDelta(bool has_match,
													const cv::Mat& position_2d_old,
													double distance_to_camera_old) noexcept
{
    position_2d[utils::indexAtTMDelta(age)] = position_2d_old;
    distance_to_camera[utils::indexAtTMDelta(age)] = distance_to_camera_old;

	if(!has_match)
	{
//		age = DELTA;
//		label = UNCERTAIN;
		valid_tmdelta_match = false;
        has_position_3d[utils::indexAtT(age)] = false;
        has_position_3d[utils::indexAtTMDelta(age)] = false;
	}
}



//---------------------------------------------------------------------------------------
void FeaturePoint::update2DPosition(const cv::Mat& position_2d) noexcept
{
    this->position_2d[utils::indexAtT(age)] = position_2d;

	if(age>CST(int,cst::DELTA))
		valid_tmdelta_match = true;
}



//---------------------------------------------------------------------------------------
void FeaturePoint::update3DPosition(const cv::Mat& position_3d) noexcept
{
    this->position_3d[utils::indexAtT(age)] = position_3d;
    has_position_3d[utils::indexAtT(age)] = true;
}



//---------------------------------------------------------------------------------------
void FeaturePoint::updateAge() noexcept
{
	age++;
}



//---------------------------------------------------------------------------------------
void FeaturePoint::updateConfidenceValue(double bonus_malus) noexcept
{
	confidence_value += bonus_malus;

	confidence_value = (confidence_value < -1.)?-1:confidence_value;
	confidence_value = (confidence_value >  1.)? 1:confidence_value;

    this->confidence_value  = confidence_value;

    if(age > CST(int, cst::DELTA))
    {
		if(confidence_value >= CST(int,cst::LABEL_ON_CV_THRESHOLD))
			label = STATIC;
		else if(confidence_value <= (-1*CST(int,cst::LABEL_ON_CV_THRESHOLD)))
			label = MOVING;
		else
			label = UNCERTAIN;
    }
}



//---------------------------------------------------------------------------------------
void FeaturePoint::updateDistanceToCamera(double distance_to_camera) noexcept
{
//	distance_to_camera_saved = this->distance_to_camera[utils::indexAtT(age)];
    this->distance_to_camera[utils::indexAtT(age)] = distance_to_camera;
}








