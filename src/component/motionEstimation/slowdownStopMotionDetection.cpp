/// @file   slowdownStopMotionDetection.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <component/motionEstimation/slowdownStopMotionDetection.h>



//---------------------------------------------------------------------------------------
void SlowdownStopMotionDetection::compute() noexcept
{
    std::string state = "ras";
    break_life_cycle = false;

    if(t != data->getTime()) // first time in SlowdownStopMotionDetection
    {
        t = data->getTime();
        n = 0;

        correctSlowDownStopMotion();

        // Save feature point
//        	saveFeaturePoint();
    }
    else // life cycle broke
    {
        std::cout << "break life cycle detected" << std::endl;
//      selectFeaturePointHistoric();
    }

//  // Slow down or stop motion detected
//  if(!isOpticalFlowGood())
//  {
//      std::cout << "slowdown or stop detected !" << std::endl;
//      state = "slowdown or stop detected";
//
//      selectFeaturePointHistoric();
//
//      for(int i=0; i<NB_MAX_POINT; i++)
//          data->featurePointReturnToSave(i);
//  }

#ifndef NDEBUG
    DataWriter::paintConfidenceValue(data, getComponentName());
    DataWriter::paintOpticalFlow(data, getComponentName());
    DataWriter::writeState(data, getComponentName(), state);
#endif
}



//---------------------------------------------------------------------------------------
void SlowdownStopMotionDetection::correctSlowDownStopMotion() noexcept
{
	std::vector<uint> save_time = data->getSaveTime();
	uint i = save_time.size();
	uint last_time = data->getTime();

	if(isOpticalFlowGood()
	|| save_time.empty())
	{
		data->saveCurrentState();
		return;
	}

	std::cout << "Not good optical flow" << std::endl;
	std::cout << "Slow down or stop motion detected" << std::endl;

	do
	{
//		last_time = save_time.back();
//		save_time.pop_back();
		last_time--;

		std::vector<cv::Mat> position_2d_last;
		std::vector<double> distance_to_camera_last;
		std::vector<uint> age_last;

		DataReader::readFeaturePoint(last_time,
									 position_2d_last,
									 age_last,
									 data,
									 "LdofOptF");

		DataReader::readDistanceToCameraAfterScale(last_time,
												   distance_to_camera_last,
												   data,
												   "Scale");

		// Replace t-delta t
		for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
		{
			uint diff_age = data->getFeaturePointAge(i) - age_last[i];
			bool has_match = (diff_age != (data->getTime() - last_time))?false:true;

//			if(i==286)
//				std::cout << data->getFeaturePointAge(i) << " " << age_last[i] << " " << data->getTime() << " " << last_time << " " << has_match << std::endl;

			data->featurePointReturnToSave(i, has_match, position_2d_last[i], distance_to_camera_last[i]);

//			if(i==286)
//				std::cout << data->getFeaturePointAge(i) << std::endl;
		}

	} while(!isOpticalFlowGood()
		 && (last_time != data->getStartTime()));
//		 && !save_time.empty());

	std::cout << "Last time used " << last_time << std::endl;
}



//---------------------------------------------------------------------------------------
std::string SlowdownStopMotionDetection::getComponentName() const noexcept
{
    return "SlowdownStopMotionDetection";
}



//---------------------------------------------------------------------------------------
std::string SlowdownStopMotionDetection::getPhaseReturn() const noexcept
{
	return "CombiningGeometricConstraintOpticalFlow";
}



//---------------------------------------------------------------------------------------
bool SlowdownStopMotionDetection::isEnoughTimeElapsed() const noexcept
{
    if(data->getTimeElapsed() > CST(int,cst::DELTA))
        return true;
    return false;
}



//---------------------------------------------------------------------------------------
bool SlowdownStopMotionDetection::isOpticalFlowGood() noexcept
{
	double sum_magnitude = 0.;
	uint count_feature_point = 0;

    auto static_point_index_begin = data->getStaticPointIndexBegin();
    auto static_point_index_end = data->getStaticPointIndexEnd();

	for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
	{
		if(!data->isFeaturePointStatic(i))
			continue;

		sum_magnitude += cv::norm(*(data->getFeaturePointPosition2DAtT(i)) - data->getFeaturePointPosition2DAtTMDelta(i));
		count_feature_point++;
	}

	double mean_magnitude = sum_magnitude/count_feature_point;

	std::cout << "mean magnitude " << mean_magnitude << std::endl;

	if(mean_magnitude < 20)
		return false;

	return true;
}



//---------------------------------------------------------------------------------------
bool SlowdownStopMotionDetection::isOpticalFlowGood(const std::vector<cv::Mat>& position_2d) noexcept
{
	double sum_magnitude = 0.;
	uint count_feature_point = 0;

	for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
	{
		if(!data->isFeaturePointStatic(i))
			continue;

		sum_magnitude += cv::norm(*(data->getFeaturePointPosition2DAtT(i)) - position_2d[i]);
		count_feature_point++;
	}

	double mean_magnitude = sum_magnitude/count_feature_point;

	if(mean_magnitude < 15)
		return false;

	return true;
}



//---------------------------------------------------------------------------------------
void SlowdownStopMotionDetection::readFileData() noexcept
{

}



//---------------------------------------------------------------------------------------
void SlowdownStopMotionDetection::saveFeaturePoint() noexcept
{
	const std::vector<uint>& save_time = data->getSaveTime();

	if(save_time.size() == 0)
	{
		data->saveCurrentState();
		return;
	}

	uint last_time = save_time.back();

	std::vector<cv::Mat> position_2d_last;
	std::vector<uint> age_last;

	DataReader::readFeaturePoint(last_time,
								 position_2d_last,
								 age_last,
								 data,
								 "LdofOptF");

	if(isOpticalFlowGood(position_2d_last))
		data->saveCurrentState();
}



//---------------------------------------------------------------------------------------
void SlowdownStopMotionDetection::selectFeaturePointHistoric() noexcept
{
	const std::vector<uint>& save_time = data->getSaveTime();

	std::vector<cv::Mat> position_2d_last;
	std::vector<double> distance_to_camera_last;
	std::vector<uint> age_last;
	uint last_time;
	uint i = save_time.size()-n;

	std::cout << "save_time size " << save_time.size() << std::endl;

	if(i==0)
	{
		break_life_cycle = true;
		return;
	}

	while(i > 0)
	{
		position_2d_last.clear();
		distance_to_camera_last.clear();
		age_last.clear();
		i--;
		n++;
		last_time = save_time[i];
		std::cout << "test " << last_time << " frame" << std::endl;
		DataReader::readFeaturePoint(last_time,
									 position_2d_last,
									 age_last,
									 data,
									 "LdofOptF");

		if(isOpticalFlowGood(position_2d_last))
			break;
	}

	if(isOpticalFlowGood(position_2d_last))
	{
		std::cout << "good optical flow" << std::endl;
		std::cout << "i choosen " << i << " last_time " << last_time << std::endl;

		DataReader::readDistanceToCameraAfterScale(last_time,
												   distance_to_camera_last,
												   data,
												   "Scale");

		// Replace t-delta t
		for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
			data->featurePointReturnToSave(i, age_last[i], position_2d_last[i], distance_to_camera_last[i]);
	}
	else
		std::cout << "not a good optical flow" << std::endl;

}



















