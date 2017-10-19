/// @file   scale.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <component/reconstruction/scale.h>



//---------------------------------------------------------------------------------------
void Scale::compute() noexcept
{
    computeScale();

#ifndef NDEBUG
    DataWriter::writeDistanceToCamera(data, getComponentName());
    DataWriter::writeBlenderFile(data, getComponentName());
#endif
}



//---------------------------------------------------------------------------------------
std::string Scale::getComponentName() const noexcept
{
    return "Scale";
}



//---------------------------------------------------------------------------------------
void Scale::computeScale() noexcept
{
	double median_scale = computeMedianScale();
	DEBUG_COMPONENT_MSG(median_scale);

	computeDistanceScale(median_scale);

#ifndef NDEBUG
    DataWriter::writeMedianScale(data, getComponentName(), median_scale);
#endif
}



//---------------------------------------------------------------------------------------
void Scale::computeDistanceScale(double scale) noexcept
{
    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        if(!data->isFeaturePointOld(i))
            continue;

        double distance = data->getFeaturePointDistanceToCameraAtT(i);
        data->updateFeaturePointDistanceToCamera(i, distance*scale);
    }
}



//---------------------------------------------------------------------------------------
double Scale::computeMedianScale() noexcept
{
    std::vector<double> scale;

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        if(!data->isFeaturePointScaleOld(i)
        || !data->isFeaturePointStatic(i))
            continue;

        double s = computeMedianScaleForAPoint(i);
		scale.push_back(s);
    }

    assert(scale.size() > 0);
    std::sort(scale.begin(), scale.end());

    return scale[scale.size()/2];
}



//---------------------------------------------------------------------------------------
double Scale::computeMedianScaleForAPoint(uint index) noexcept
{
    std::vector<double> scale;

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        if(!data->isFeaturePointScaleOld(i)
        || !data->isFeaturePointStatic(i)
        || i == index)
            continue;

        double distance_ptp_t   = cv::norm(*data->getFeaturePointPosition3DAtT(i)   - *data->getFeaturePointPosition3DAtT(index));
        double distance_ptp_tm1 = cv::norm(*data->getFeaturePointPosition3DAtTM1(i) - *data->getFeaturePointPosition3DAtTM1(index));

		scale.push_back(distance_ptp_tm1/distance_ptp_t);
    }

    assert(scale.size() > 0);
    std::sort(scale.begin(), scale.end());

    return scale[scale.size()/2];
}



//---------------------------------------------------------------------------------------
double Scale::computeMedianScaleForAPointOnRandomPoint(uint index) noexcept
{
	std::vector<bool> is_used(CST(int,cst::NB_MAX_POINT), false);
	std::vector<double> scale;
	uint count = 0;

	uint nb_static_point = data->getNbStaticPoint();

	if(nb_static_point>CST(int,cst::SCALE_NB_SAMPLE_POINT))
	{
		std::vector<uint> static_point_index = data->getStaticPointIndex();
		random_unique(static_point_index.begin(), static_point_index.end(), CST(int,cst::SCALE_NB_SAMPLE_POINT));

		for(int i=0; i<CST(int,cst::SCALE_NB_SAMPLE_POINT); i++)
		{
			uint feature_point_id = static_point_index[i];

			if(!data->isFeaturePointScaleOld(feature_point_id)
			|| !data->isFeaturePointStatic(feature_point_id)
			|| feature_point_id == index
			|| is_used[feature_point_id])
				continue;

			double distance_ptp_t   = cv::norm(*data->getFeaturePointPosition3DAtT(feature_point_id)   - *data->getFeaturePointPosition3DAtT(index));
			double distance_ptp_tm1 = cv::norm(*data->getFeaturePointPosition3DAtTM1(feature_point_id) - *data->getFeaturePointPosition3DAtTM1(index));

			if(distance_ptp_t == 0)
				DEBUG_COMPONENT_MSG("error distance ");

			scale.push_back(distance_ptp_tm1/distance_ptp_t);
			is_used[feature_point_id] = true;
			count++;
		}
	}
	else
	{
		for(int i=0; i<nb_static_point; i++)
		{
			uint feature_point_id = i;

			if(!data->isFeaturePointScaleOld(feature_point_id)
			|| !data->isFeaturePointStatic(feature_point_id)
			|| feature_point_id == index
			|| is_used[feature_point_id])
				continue;

			double distance_ptp_t   = cv::norm(*data->getFeaturePointPosition3DAtT(feature_point_id)   - *data->getFeaturePointPosition3DAtT(index));
			double distance_ptp_tm1 = cv::norm(*data->getFeaturePointPosition3DAtTM1(feature_point_id) - *data->getFeaturePointPosition3DAtTM1(index));

			scale.push_back(distance_ptp_tm1/distance_ptp_t);
			is_used[feature_point_id] = true;
			count++;
		}
	}

	assert(scale.size() > 0);
	std::sort(scale.begin(), scale.end());

	return scale[scale.size()/2];
}



//---------------------------------------------------------------------------------------
double Scale::computeMedianScaleOnRandomPoint() noexcept
{

    std::vector<double> scale;
    std::vector<bool> is_used(CST(int,cst::NB_MAX_POINT), false);
    uint count = 0;

	uint nb_static_point = data->getNbStaticPoint();

//	if(nb_static_point>SCALE_NB_SAMPLE_POINT)
//	{
		std::vector<uint> static_point_index = data->getStaticPointIndex();
		DEBUG_COMPONENT_MSG("nb static point " << static_point_index.size());

		uint nb_point = (CST(int,cst::SCALE_NB_SAMPLE_POINT)>static_point_index.size())?static_point_index.size():CST(int,cst::SCALE_NB_SAMPLE_POINT);

		if(static_point_index.size() > CST(int,cst::SCALE_NB_SAMPLE_POINT))
			random_unique(static_point_index.begin(), static_point_index.end(), CST(int,cst::SCALE_NB_SAMPLE_POINT));

		for(int i=0; i<nb_point; i++)
		{
			uint feature_point_id = static_point_index[i];

			if(!data->isFeaturePointScaleOld(feature_point_id)
			|| !data->isFeaturePointStatic(feature_point_id)
			|| is_used[feature_point_id])
				continue;

			double s = computeMedianScaleForAPointOnRandomPoint(feature_point_id);
			scale.push_back(s);
			is_used[feature_point_id] = true;
			count++;
		}
//	}
//	else
//	{
//		for(int i=0; i<nb_static_point; i++)
//		{
//			uint feature_point_id = i;
//
//			if(!data->isFeaturePointScaleOld(feature_point_id)
//			|| !data->isFeaturePointStatic(feature_point_id)
//			|| is_used[feature_point_id])
//				continue;
//
//			double s = computeMedianScaleForAPointOnRandomPoint(feature_point_id); // TODO: Not the right function called here
//			scale.push_back(s);
//			is_used[feature_point_id] = true;
//			count++;
//		}
//	}

    assert(scale.size() > 0);
    std::sort(scale.begin(), scale.end());

    return scale[scale.size()/2];
}



//---------------------------------------------------------------------------------------
bool Scale::isEnoughTimeElapsed() const noexcept
{
    if(data->getTimeElapsed() > CST(int,cst::DELTA))
        return true;
    return false;
}



//---------------------------------------------------------------------------------------
std::vector<uint>::iterator Scale::random_unique(std::vector<uint>::iterator begin,
												 std::vector<uint>::iterator end,
												 std::size_t num_random) noexcept
{
	std::size_t left = std::distance(begin, end);
	while(num_random--)
	{
		std::vector<uint>::iterator r = begin;
		std::advance(r, rand()%left);
		std::swap(*begin, *r);
		++begin;
		--left;
	}

	return begin;
}



//---------------------------------------------------------------------------------------
void Scale::readFileData() noexcept
{

}


















