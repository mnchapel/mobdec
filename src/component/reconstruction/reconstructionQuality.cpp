/// @file   reconstructionQuality.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <component/reconstruction/reconstructionQuality.h>



//---------------------------------------------------------------------------------------
void ReconstructionQuality::compareToOtherStaticFeaturePoint(uint index_i,
														     std::vector<uint>& valid_distance,
															 double& mean_error_distance_static,
															 double& mean_error_distance_move) const noexcept
{

    std::vector<uint> index_list;
    double sum_distance_moving = 0.;
    uint count_distance_moving = 0;

    double sum_distance_static = 0.;
    uint count_distance_static = 0;

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
    	uint feature_point_id = i;

        if(!data->isFeaturePointStatic(feature_point_id)
        || !data->isFeaturePointOld(feature_point_id)
		|| !data->isFeaturePointTwoReconstructionsOld(feature_point_id)
		|| index_i == feature_point_id)
            continue;

        if(isPointStable(index_i, feature_point_id, sum_distance_moving, count_distance_moving, sum_distance_static, count_distance_static))
        {
            valid_distance[index_i]++;
            index_list.push_back(feature_point_id);
        }
    }

    if(count_distance_static == 0)
    	mean_error_distance_static = 0;
    else
    	mean_error_distance_static = sum_distance_static/count_distance_static;


    if(count_distance_moving == 0)
    	mean_error_distance_move = 0;
    else
    	mean_error_distance_move = sum_distance_moving/count_distance_moving;



//	std::vector<bool> is_used(NB_MAX_POINT, false);
//	uint count = 0;
//
//    double sum_distance_moving = 0.;
//    uint count_distance_moving = 0;
//
//    double sum_distance_static = 0.;
//    uint count_distance_static = 0;
//
//
//	std::vector<uint> static_point_index = data->getStaticPointIndex();
//
//	random_unique(static_point_index.begin(), static_point_index.end(), SCALE_NB_SAMPLE_POINT);
//
//    auto static_point_index_begin = static_point_index.begin();
//    auto static_point_index_end   = static_point_index.end();
//
//	for(auto it=static_point_index_begin; it!=static_point_index_end; ++it)
//    {
//		uint feature_point_id = *it;
//
//        if(!data->isFeaturePointStatic(feature_point_id)
//        || !data->isFeaturePointScaleOld(feature_point_id)
//		|| !data->hasTMDeltaMatching(index_i)
//		|| index_i == feature_point_id
//		|| is_used[feature_point_id])
//            continue;
//
//        if(isPointStable(index_i, feature_point_id, sum_distance_moving, count_distance_moving, sum_distance_static, count_distance_static))
//            valid_distance[index_i]++;
//
//		is_used[feature_point_id] = true;
//		count++;
//    }
//
//	if(count_distance_static == 0)
//		mean_error_distance_static = 0;
//	else
//		mean_error_distance_static = sum_distance_static/count_distance_static;
//
//	if(count_distance_moving == 0)
//		mean_error_distance_move = 0;
//	else
//		mean_error_distance_move   = sum_distance_moving/count_distance_moving;
}



//---------------------------------------------------------------------------------------
void ReconstructionQuality::compute() noexcept
{
    break_life_cycle = false;

    std::cout << "last good reconstruction " << data->getLastGoodReconstruction() << std::endl;
    if(!isGoodReconstruction())
    {
        std::cout << "not a good reconstruction" << std::endl;
//    		break_life_cycle = true;
    }
//    else
//        data->setLastGoodReconstruction(data->getTime());

#ifndef NDEBUG
    DataWriter::writeDistanceToCamera(data, getComponentName());
    DataWriter::writeBlenderFile(data, getComponentName());
#endif
}



//---------------------------------------------------------------------------------------
uint ReconstructionQuality::computePtPDistanceOnValidPercent(std::vector<uint>& index_list,
															 std::vector<uint>& valid_distance,
															 std::vector<double>& mean_error_distance_static,
															 std::vector<double>& mean_error_distance_move) const noexcept
{
    std::vector<uint> valid_distance_old_point;
    std::vector< std::vector<uint> > valid_index(CST(int,cst::NB_MAX_POINT), std::vector<uint>());

    uint value_max = 0;

    for(int index_i=0; index_i<CST(int,cst::NB_MAX_POINT); index_i++)
    {
        if(!data->isFeaturePointOld(index_i)
         ||!data->isFeaturePointStatic(index_i)
		 ||!data->isFeaturePointTwoReconstructionsOld(index_i))
            continue;

        compareToOtherStaticFeaturePoint(index_i, valid_distance, mean_error_distance_static[index_i], mean_error_distance_move[index_i]);

        valid_distance_old_point.push_back(valid_distance[index_i]);
        index_list.push_back(index_i);

        if(valid_distance[index_i] > value_max)
            value_max = valid_distance[index_i];
    }

    std::cout << "value_max " << value_max << std::endl;
    uint nb_min_valid_point_to_bonus = value_max * CST(int,cst::PERCENT_PC_TO_BE_STATIC);

    return nb_min_valid_point_to_bonus;


//	auto static_feature_point_begin = data->getStaticPointIndexBegin();
//	auto static_feature_point_end   = data->getStaticPointIndexEnd();
//
//    uint value_max = 0;
////    for(int index_i=0; index_i<NB_MAX_POINT; index_i++)
//    for(auto it = static_feature_point_begin; it!=static_feature_point_end; ++it)
//    {
//    	uint index_i = *it;
//
////    	std::cout << data->hasPosition3DComputedAtTM1(index_i) << std::endl;
//        if(!data->isFeaturePointScaleOld(index_i)
//        || !data->isFeaturePointStatic(index_i)
//		|| !data->hasTMDeltaMatching(index_i)
//		|| !data->hasPosition3DComputedAtTM1(index_i))
//            continue;
//
//        compareToOtherStaticFeaturePoint(index_i, valid_distance, mean_error_distance_static[index_i], mean_error_distance_move[index_i]);
//
//        index_list.push_back(index_i);
//
//        if(valid_distance[index_i] > value_max)
//            value_max = valid_distance[index_i];
//    }
//
//    uint nb_min_valid_point_to_bonus = value_max * 0.5;
//
//    return nb_min_valid_point_to_bonus;
}



//---------------------------------------------------------------------------------------
uint ReconstructionQuality::computePtPDistanceOnValidPercentTMDelta(std::vector<uint>& index_list,
															 	 	std::vector<uint>& valid_distance,
																	std::vector<double>& mean_error_distance_static,
																	std::vector<double>& mean_error_distance_move) const noexcept
{
//    std::vector< std::vector<uint> > valid_index(NB_MAX_POINT, std::vector<uint>());
//
//    uint value_max = 0;
//    for(int index_i=0; index_i<NB_MAX_POINT; index_i++)
//    {
//        if(!data->isFeaturePointOld(index_i)
//        || !data->isFeaturePointStatic(index_i)
//		|| !data->hasTMDeltaMatching(index_i))
//            continue;
//
//        compareToOtherStaticFeaturePointTMDelta(index_i, valid_distance, valid_index, mean_error_distance_static[index_i], mean_error_distance_move[index_i]);
//
//        index_list.push_back(index_i);
//
//        if(valid_distance[index_i] > value_max)
//            value_max = valid_distance[index_i];
//    }
//
//    uint nb_min_valid_point_to_bonus = value_max * 0.4;
//
//    return nb_min_valid_point_to_bonus;

	return 0;
}



//---------------------------------------------------------------------------------------
std::string ReconstructionQuality::getComponentName() const noexcept
{
    return "ReconstructionQuality";
}



//---------------------------------------------------------------------------------------
std::string ReconstructionQuality::getPhaseReturn() const noexcept
{
    return "EssentialMatrix";
}



//---------------------------------------------------------------------------------------
bool ReconstructionQuality::isEnoughTimeElapsed() const noexcept
{
    if(data->getTimeElapsed() > CST(int,cst::DELTA))
        return true;

    return false;
}



//---------------------------------------------------------------------------------------
bool ReconstructionQuality::isGoodReconstruction() const noexcept
{
	uint count_static = 0;
	uint count_valid  = 0;

	std::vector<uint> index_list;
	std::vector<uint> valid_distance(CST(int,cst::NB_MAX_POINT), 0);
	std::vector<double> mean_error_distance_static(CST(int,cst::NB_MAX_POINT), 0);
	std::vector<double> mean_error_distance_move(CST(int,cst::NB_MAX_POINT), 0);

	uint nb_min_valid_point_to_bonus = computePtPDistanceOnValidPercent(index_list, valid_distance, mean_error_distance_static, mean_error_distance_move);

	for(auto index: index_list)
	{
		if(valid_distance[index] > nb_min_valid_point_to_bonus)
			count_valid++;
		count_static++;
	}

	double percent_valid;
	if(count_static == 0)
		percent_valid = 0;
	else
		percent_valid = count_valid /(double)count_static;

	std::cout << "count_valid   " << count_valid   << std::endl;
	std::cout << "count_static  " << count_static  << std::endl;
	std::cout << "percent_valid " << percent_valid << std::endl;

	data->updateReconstructionQuality(percent_valid);

	if(percent_valid<0.5)
		return false;

	return true;
}



//---------------------------------------------------------------------------------------
bool ReconstructionQuality::isPointStable(uint index_i,
										  uint index_j,
										  double& sum_distance_moving,
										  uint& count_distance_moving,
										  double& sum_distance_static,
										  uint& count_distance_static) const noexcept
{
//		uint last_good_reconstruction = data->getTime() - data->getLastGoodReconstruction();

		cv::Point3d fp_t_i(*data->getFeaturePointPosition3DAtT(index_i));
		cv::Point3d fp_t_j(*data->getFeaturePointPosition3DAtT(index_j));
		cv::Point3d fp_t = fp_t_i - fp_t_j;
		double distance_ptp_t  = sqrt(fp_t.x*fp_t.x + fp_t.y*fp_t.y);

//		cv::Point3d fp_i(*data->getFeaturePointPosition3DAt(last_good_reconstruction, index_i));
//		cv::Point3d fp_j(*data->getFeaturePointPosition3DAt(last_good_reconstruction, index_j));
		cv::Point3d fp_i(*data->getFeaturePointPosition3DAtTMDelta(index_i));
		cv::Point3d fp_j(*data->getFeaturePointPosition3DAtTMDelta(index_j));
		cv::Point3d fp = fp_i - fp_j;
		double distance_ptp_tm  = sqrt(fp.x*fp.x + fp.y*fp.y);

	    double diff = fabs(distance_ptp_t - distance_ptp_tm);

	    if(diff < CST(int,cst::MAX_PTP_ERROR_STATIC))
	    {
	        sum_distance_static += diff;
	        count_distance_static++;

	        return true;
	    }

	    sum_distance_moving += diff;
	    count_distance_moving++;

	    return false;
}



//---------------------------------------------------------------------------------------
std::vector<uint>::iterator ReconstructionQuality::random_unique(std::vector<uint>::iterator begin,
												 	 	 	 	 std::vector<uint>::iterator end,
																 std::size_t num_random) const noexcept
{
	std::size_t left = std::distance(begin, end);
	while(num_random--)
	{
		std::vector<uint>::iterator r(begin);
		std::advance(r, rand()%left);
		std::swap(*begin, *r);
		++begin;
		--left;
	}

	return begin;
}



//---------------------------------------------------------------------------------------
void ReconstructionQuality::readFileData() noexcept
{

}
