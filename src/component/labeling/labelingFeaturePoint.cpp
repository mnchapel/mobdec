/// @file   labelingFeaturePoint.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <component/labeling/labelingFeaturePoint.h>



//---------------------------------------------------------------------------------------
void LabelingFeaturePoint::compute() noexcept
{
    std::vector<uint> index_list;
    std::vector<uint> valid_distance(CST(int,cst::NB_MAX_POINT), 0);
    std::vector<double> mean_error_distance_static(CST(int,cst::NB_MAX_POINT), 0);
    std::vector<double> mean_error_distance_move(CST(int,cst::NB_MAX_POINT), 0);

    uint nb_min_valid_point_to_bonus = computePtPDistanceOnValidPercent(index_list, valid_distance, mean_error_distance_static, mean_error_distance_move);
    DEBUG_COMPONENT_MSG("nb_min_valid_point_to_bonus " << nb_min_valid_point_to_bonus);
    updateConfidenceValue(nb_min_valid_point_to_bonus, index_list, valid_distance, mean_error_distance_static, mean_error_distance_move);

#ifndef NDEBUG
    DataWriter::paintConfidenceValue(data, getComponentName());
    DataWriter::writeLabel(data, getComponentName());
    DataWriter::writeConfidenceValue(data, getComponentName());
#endif
}



//---------------------------------------------------------------------------------------
std::string LabelingFeaturePoint::getComponentName() const noexcept
{
    return "LabelingFeaturePoint";
}



//---------------------------------------------------------------------------------------
void LabelingFeaturePoint::compareToOtherStaticFeaturePoint(uint index_i,
															std::vector<uint>& valid_distance,
															std::vector< std::vector<uint> >& valid_index,
															double& mean_error_distance_static,
															double& mean_error_distance_move) noexcept
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
		|| index_i == feature_point_id)
            continue;

        if(isPointStable(index_i, feature_point_id, sum_distance_moving, count_distance_moving, sum_distance_static, count_distance_static))
        {
            valid_distance[index_i]++;
            valid_index[index_i].push_back(feature_point_id);
            index_list.push_back(feature_point_id);
        }
    }

    mean_error_distance_static = sum_distance_static/count_distance_static;
    mean_error_distance_move   = sum_distance_moving/count_distance_moving;

#ifndef NDEBUG
//    DataWriter::writeValidDistanceIndexForAPoint(data, getComponentName(), index_i, index_list);
#endif
}



//---------------------------------------------------------------------------------------
uint LabelingFeaturePoint::computePtPDistanceOnValidPercent(std::vector<uint>& index_list,
															std::vector<uint>& valid_distance,
	    													std::vector<double>& mean_error_distance_static,
															std::vector<double>& mean_error_distance_move) noexcept
{
    std::vector<uint> valid_distance_old_point;
    std::vector< std::vector<uint> > valid_index(CST(int,cst::NB_MAX_POINT), std::vector<uint>());

    uint value_max = 0;

    for(int index_i=0; index_i<CST(int,cst::NB_MAX_POINT); index_i++)
    {
        if(!data->isFeaturePointOld(index_i))
            continue;

        compareToOtherStaticFeaturePoint(index_i, valid_distance, valid_index, mean_error_distance_static[index_i], mean_error_distance_move[index_i]);

        valid_distance_old_point.push_back(valid_distance[index_i]);
        index_list.push_back(index_i);

        if(valid_distance[index_i] > value_max)
            value_max = valid_distance[index_i];
    }

    DEBUG_COMPONENT_MSG("value_max " << value_max);
    DEBUG_COMPONENT_MSG("percent_pc_to_be_static " << CST(double,cst::PERCENT_PC_TO_BE_STATIC));
    uint nb_min_valid_point_to_bonus = value_max * CST(double,cst::PERCENT_PC_TO_BE_STATIC);

#ifndef NDEBUG
//    DataWriter::writeMaxNbValidPointToBonus(data, getComponentName(), value_max);
//
//    if(valid_distance_old_point.size()>0)
//    {
//        std::sort(valid_distance_old_point.begin(), valid_distance_old_point.end());
//    }
//    DataWriter::writePointTrust(data, getComponentName(), 0, 0);
//    DataWriter::writeValidDistance(data, getComponentName(), valid_distance);
//    DataWriter::writeValidDistanceIndex(data, getComponentName(), index_list);
//    DataWriter::writeValidDistanceSorted(data, getComponentName(), valid_distance_old_point);
#endif

    return nb_min_valid_point_to_bonus;
}



//---------------------------------------------------------------------------------------
bool LabelingFeaturePoint::isEnoughTimeElapsed() const noexcept
{
	if(data->getTimeElapsed() >= (CST(int,cst::DELTA)*2))
        return true;
    return false;
}



//---------------------------------------------------------------------------------------
bool LabelingFeaturePoint::isPointStable(uint index_i,
                                    	 uint index_j,
										 double& sum_distance_moving,
										 uint& count_distance_moving,
										 double& sum_distance_static,
										 uint& count_distance_static) noexcept
{
    double distance_ptp_t   	= cv::norm((*data->getFeaturePointPosition3DAtT(index_i))           - (*data->getFeaturePointPosition3DAtT(index_j)));
    double distance_ptp_tmdelta = cv::norm((*data->getFeaturePointPosition3DAtTMDelta(index_i)) - (*data->getFeaturePointPosition3DAtTMDelta(index_j)));

    double diff = fabs(distance_ptp_t - distance_ptp_tmdelta);

//    if(index_i == 967)
//    	std::cout << "index_i " << index_i << " " << cv::Point3d((*data->getFeaturePointPosition3DAtT(index_i))) << " " << cv::Point3d((*data->getFeaturePointPosition3DAtTMDelta(index_i))) << std::endl;

    if(diff < CST(double,cst::MAX_PTP_ERROR_STATIC))
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
void LabelingFeaturePoint::readFileData() noexcept
{

}



//---------------------------------------------------------------------------------------
void LabelingFeaturePoint::updateBonusConfidenceIndex(uint index,
                                                 	  double mean_error_distance_static) noexcept
{
//	double bonus = 1./(CST(int,cst::NB_FRAME_ACCUMULATION)*2)*cos(mean_error_distance_static*(M_PI/CST(double,cst::MAX_PTP_ERROR_STATIC)))+1./(CST(int,cst::NB_FRAME_ACCUMULATION)*2);
    double bonus = std::exp(-1*mean_error_distance_static*mean_error_distance_static/(CST(double, cst::MAX_PTP_ERROR_STATIC)*CST(double, cst::MAX_PTP_ERROR_STATIC)))/CST(double, cst::NB_FRAME_ACCUMULATION);
    data->updateConfidenceValue(index, bonus);
}



//---------------------------------------------------------------------------------------
void LabelingFeaturePoint::updateConfidenceValue(uint nb_min_valid_point_to_bonus,
												 const std::vector<uint>& index_list,
												 const std::vector<uint>& valid_distance,
												 const std::vector<double>& mean_error_distance_static,
												 const std::vector<double>& mean_error_distance_move) noexcept
{
	data->clearStaticFeaturePoint();
    std::vector<bool> hasBeenUpdated(CST(int,cst::NB_MAX_POINT), false);

    // Update bonus
    for(int i=0; i<index_list.size(); i++)
    {
        uint index = index_list[i];

        if(valid_distance[index] >= nb_min_valid_point_to_bonus && data->isFeaturePointTwoReconstructionsOld(index))
        {
            hasBeenUpdated[index] = true;
            updateBonusConfidenceIndex(index, mean_error_distance_static[index]);
        }
    }

    // Update malus
    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        if(!data->isFeaturePointOld(i) || !data->isFeaturePointTwoReconstructionsOld(i))
            continue;

        if(!hasBeenUpdated[i])
        {
            hasBeenUpdated[i] = true;
            updateMalusConfidenceIndex(i, mean_error_distance_move[i]);
        }
    }
}



//---------------------------------------------------------------------------------------
void LabelingFeaturePoint::updateMalusConfidenceIndex(uint index,
                                                 	  double mean_error_distance_move) noexcept
{
    if(mean_error_distance_move > 0.5)
        mean_error_distance_move = 0.5;

//    double malus = 1./(CST(int,cst::NB_FRAME_ACCUMULATION)*2)*cos((mean_error_distance_move-CST(double,cst::MAX_PTP_ERROR_STATIC))*(M_PI/CST(double,cst::MAX_PTP_ERROR_MOVING)))-1./(CST(int,cst::NB_FRAME_ACCUMULATION)*2);
    double malus = -1 * std::exp(-1*pow((mean_error_distance_move-1.),2)/(CST(double, cst::MAX_PTP_ERROR_MOVING)*CST(double, cst::MAX_PTP_ERROR_MOVING)))/CST(double, cst::NB_FRAME_ACCUMULATION);
    data->updateConfidenceValue(index, malus);
}















