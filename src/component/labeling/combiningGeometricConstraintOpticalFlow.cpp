/// @file   combiningGeometricConstraintOpticalFlow.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <component/labeling/combiningGeometricConstraintOpticalFlow.h>

#define DIST_NEAR_CLUSTER 40



//---------------------------------------------------------------------------------------
void CombiningGeometricConstraintOpticalFlow::compareToOtherStaticFeaturePoint(uint index_i,
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




//	std::vector<bool> is_used(NB_MAX_POINT, false);
//	uint count = 0;
//
//    std::vector<uint> index_list;
//    double sum_distance_moving = 0.;
//    uint count_distance_moving = 0;
//
//    double sum_distance_static = 0.;
//    uint count_distance_static = 0;
//
//    auto static_point_index_begin = data->getStaticPointIndexBegin();
//    auto static_point_index_end = data->getStaticPointIndexEnd();
//	std::vector<uint> static_point_index = data->getStaticPointIndex();
////	random_unique(static_point_index.begin(), static_point_index.end(), SCALE_NB_SAMPLE_POINT);
//
//	for(auto it=static_point_index_begin; it!=static_point_index_end; ++it)
//	{
//		uint feature_point_id = *it;
//
//        if(!data->isFeaturePointStatic(feature_point_id)
//        || !data->isFeaturePointTwoReconstructionsOld(feature_point_id)
//		|| !data->hasTMDeltaMatching(feature_point_id)
//		|| !data->hasPosition3DComputedAtTMDelta(feature_point_id)
//		|| index_i == feature_point_id
//		|| is_used[feature_point_id])
//            continue;
//
//        if(isPointStable(index_i, feature_point_id, sum_distance_moving, count_distance_moving, sum_distance_static, count_distance_static))
//        {
//            valid_distance[index_i]++;
//            valid_index[index_i].push_back(feature_point_id);
//            index_list.push_back(feature_point_id);
//        }
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
//
//#ifndef NDEBUG
////    DataWriter::writeValidDistanceIndexForAPoint(data, getComponentName(), index_i, index_list);
//#endif
}



//---------------------------------------------------------------------------------------
void CombiningGeometricConstraintOpticalFlow::compute() noexcept
{
	std::vector<std::vector<uint>> cluster_feature_point_id_list;
	std::vector<double> cluster_bonus_malus_value;
    std::vector<double> bonus_malus_value(CST(int,cst::NB_MAX_POINT), 0);

    std::vector<uint> index_list;
    std::vector<uint> valid_distance(CST(int,cst::NB_MAX_POINT), 0);
    std::vector<double> mean_error_distance_static(CST(int,cst::NB_MAX_POINT), 0);
    std::vector<double> mean_error_distance_move(CST(int,cst::NB_MAX_POINT), 0);

    createOpticalFlowCluster(cluster_feature_point_id_list);
    uint nb_min_valid_point_to_bonus = computePtPDistanceOnValidPercent(index_list, valid_distance, mean_error_distance_static, mean_error_distance_move);
    std::cout << "nb_min_valid_point_to_bonus " << nb_min_valid_point_to_bonus << std::endl;
    data->clearStaticFeaturePoint();
    updateConfidenceValue(nb_min_valid_point_to_bonus,
                          cluster_feature_point_id_list,
                          index_list,
                          valid_distance,
                          mean_error_distance_static,
                          mean_error_distance_move,
                          cluster_bonus_malus_value,
                          bonus_malus_value);

#ifndef NDEBUG
    DataWriter::paintCluster(data, getComponentName(), cluster_feature_point_id_list);
    DataWriter::paintConfidenceValue(data, getComponentName());
    DataWriter::paintTrendClusterBonusMalus(data, getComponentName(), cluster_feature_point_id_list, cluster_bonus_malus_value);
    DataWriter::paintTrendBonusMalus(data, getComponentName(), cluster_feature_point_id_list, bonus_malus_value);
//    DataWriter::writeCluster(data, getComponentName(), cluster_feature_point_id_list);
    DataWriter::writeConfidenceValue(data, getComponentName());
    DataWriter::writeLabel(data, getComponentName());
//    DataWriter::writeTrendClusterBonusMalus(data, getComponentName(), cluster_feature_point_id_list, cluster_bonus_malus_value);
//    DataWriter::writeTrendBonusMalus(data, getComponentName(), cluster_feature_point_id_list, bonus_malus_value);
#endif
}



//---------------------------------------------------------------------------------------
std::string CombiningGeometricConstraintOpticalFlow::getComponentName() const noexcept
{
    return "CombiningGeometricConstraintOpticalFlow";
}



//---------------------------------------------------------------------------------------
void CombiningGeometricConstraintOpticalFlow::computeAnOpticalFlowCluster(std::vector<bool>& is_in_cluster,
                                                           	   	   	   	  std::vector<uint>& cluster) noexcept
{
    uint index_start = cluster[0]+1;

    bool change = true;
    while(change)
    {
        change = false;
        for(int i=index_start; i<CST(int,cst::NB_MAX_POINT); i++)
        {
            if(!data->isFeaturePointOpticalFlowOld(i)
//            || !data->hasPosition3DComputedAtTMDelta(i) // TODO ne pas oublier
            || is_in_cluster[i])
                continue;

            for(int j=0; j<cluster.size(); j++)
            {
                if(isOpticalFlowNearToCluster(i, cluster[j])
                && isPointNearToCluster(i, cluster[j]))
                {
                    cluster.push_back(i);
                    is_in_cluster[i] = true;
                    change = true;
                    break;
                }
            }
        }
    }
}



//---------------------------------------------------------------------------------------
uint CombiningGeometricConstraintOpticalFlow::computePtPDistanceOnValidPercent(std::vector<uint>& index_list,
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

    std::cout << "value_max " << value_max << std::endl;
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





//    std::vector<uint> valid_distance_old_point;
//    std::vector< std::vector<uint> > valid_index(NB_MAX_POINT, std::vector<uint>());
//
//    uint value_max = 0;
//
//    for(int index_i=0; index_i<NB_MAX_POINT; index_i++)
//    {
//        if(!data->isFeaturePointTwoReconstructionsOld(index_i)
//		|| !data->hasTMDeltaMatching(index_i))
////		|| !data->hasPosition3DComputedAtTMDelta(index_i))
//            continue;
//
//        compareToOtherStaticFeaturePoint(index_i, valid_distance, valid_index, mean_error_distance_static[index_i], mean_error_distance_move[index_i]);
//
//        valid_distance_old_point.push_back(valid_distance[index_i]);
//        index_list.push_back(index_i);
//
//        if(valid_distance[index_i] > value_max)
//            value_max = valid_distance[index_i];
//    }
//
//    uint nb_min_valid_point_to_bonus = value_max * PERCENT_PC_TO_BE_STATIC;
//
//#ifndef NDEBUG
////    DataWriter::writeMaxNbValidPointToBonus(data, getComponentName(), value_max);
////
////    if(valid_distance_old_point.size()>0)
////    {
////        std::sort(valid_distance_old_point.begin(), valid_distance_old_point.end());
////    }
////    DataWriter::writePointTrust(data, getComponentName(), 0, 0);
////    DataWriter::writeValidDistance(data, getComponentName(), valid_distance);
////    DataWriter::writeValidDistanceIndex(data, getComponentName(), index_list);
////    DataWriter::writeValidDistanceSorted(data, getComponentName(), valid_distance_old_point);
//#endif
//
//    return nb_min_valid_point_to_bonus;
}



//---------------------------------------------------------------------------------------
void CombiningGeometricConstraintOpticalFlow::createOpticalFlowCluster(std::vector<std::vector<uint>>& cluster_feature_point_id_list) noexcept
{
    std::vector<bool> is_in_cluster(CST(int,cst::NB_MAX_POINT), false);

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        if(!data->isFeaturePointOpticalFlowOld(i)
//        || !data->hasPosition3DComputedAtTMDelta(i)
        || is_in_cluster[i])
            continue;

        std::vector<uint> cluster;
        cluster.push_back(i);
        is_in_cluster[i] = true;

        computeAnOpticalFlowCluster(is_in_cluster, cluster);

        cluster_feature_point_id_list.push_back(cluster);
    }
}



//---------------------------------------------------------------------------------------
bool CombiningGeometricConstraintOpticalFlow::isEnoughTimeElapsed() const noexcept
{
    if(data->getTimeElapsed() >= (CST(int,cst::DELTA)*2))
        true;
    else
        false;
}



//---------------------------------------------------------------------------------------
bool CombiningGeometricConstraintOpticalFlow::isOpticalFlowNearToCluster(uint i,
														  	  	  	  	 uint j) noexcept
{
	cv::Mat optf_1 = data->getFeaturePointOpticalFlowTMDelta(i);
	cv::Mat optf_2 = data->getFeaturePointOpticalFlowTMDelta(j);

    double angle_diff = acos((optf_1.dot(optf_2))/(cv::norm(optf_1) * cv::norm(optf_2))) * 180./M_PI;
    double magnitude_diff = fabs(cv::norm(optf_1)-cv::norm(optf_2));

    if(angle_diff <= 2 && magnitude_diff <=2)
        return true;
    return false;
}



//---------------------------------------------------------------------------------------
bool CombiningGeometricConstraintOpticalFlow::isPointNearToCluster(uint index_1,
                                                    			   uint index_2) noexcept
{
	cv::Mat point_1 = *(data->getFeaturePointPosition2DAtT(index_1));
	cv::Mat point_2 = *(data->getFeaturePointPosition2DAtT(index_2));

    double square_dist = cv::norm(point_1-point_2);

    if(square_dist <= (DIST_NEAR_CLUSTER))
        return true;
    return false;
}



//---------------------------------------------------------------------------------------
bool CombiningGeometricConstraintOpticalFlow::isPointStable(uint index_i,
                                    	 	 	 	 	 	uint index_j,
															double& sum_distance_moving,
															uint& count_distance_moving,
															double& sum_distance_static,
															uint& count_distance_static) noexcept
{
    double distance_ptp_t 		= cv::norm(*data->getFeaturePointPosition3DAtT(index_i)       - *data->getFeaturePointPosition3DAtT(index_j));
    double distance_ptp_tmdelta = cv::norm(*data->getFeaturePointPosition3DAtTMDelta(index_i) - *data->getFeaturePointPosition3DAtTMDelta(index_j));

//	cv::Point3d fp_t_i(*data->getFeaturePointPosition3DAtT(index_i));
//	cv::Point3d fp_t_j(*data->getFeaturePointPosition3DAtT(index_j));
//	cv::Point3d fp_t = fp_t_i - fp_t_j;
//	double distance_ptp_t  = sqrt(fp_t.x*fp_t.x + fp_t.y*fp_t.y);
//
//	cv::Point3d fp_tmdelta_i(*data->getFeaturePointPosition3DAtTMDelta(index_i));
//	cv::Point3d fp_tmdelta_j(*data->getFeaturePointPosition3DAtTMDelta(index_j));
//	cv::Point3d fp_tmdelta = fp_tmdelta_i - fp_tmdelta_j;
//	double distance_ptp_tmdelta  = sqrt(fp_tmdelta.x*fp_tmdelta.x + fp_tmdelta.y*fp_tmdelta.y);

    double diff = fabs(distance_ptp_t - distance_ptp_tmdelta);

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
std::vector<uint>::iterator CombiningGeometricConstraintOpticalFlow::random_unique(std::vector<uint>::iterator begin,
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
void CombiningGeometricConstraintOpticalFlow::readClusterFile(std::vector<std::vector<uint>>& cluster_feature_point_id_list) noexcept
{
    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_DATA_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + getComponentName() + "/cluster_%03d.txt";
    sprintf(buffer, path.c_str(), data->getTime());

    std::ifstream file;
    file.open(buffer);

    uint nb_cluster;
    uint cluster_id;
    uint feature_point_id;

    file >> nb_cluster;

    cluster_feature_point_id_list.resize(nb_cluster);

    while(!file.eof())
    {
    	file >> cluster_id;
    	file >> feature_point_id;

    	cluster_feature_point_id_list[cluster_id].push_back(feature_point_id);
    }

    file.close();
}



//---------------------------------------------------------------------------------------
void CombiningGeometricConstraintOpticalFlow::readFileData() noexcept
{

}



//---------------------------------------------------------------------------------------
double CombiningGeometricConstraintOpticalFlow::computeBonusMalusValue(uint nb_min_valid_point_to_bonus,
																	   uint valid_distance,
																	   double mean_error_distance_static,
																	   double mean_error_distance_move) noexcept
{
	double value;

	if(valid_distance >= nb_min_valid_point_to_bonus) // BONUS
	{
//		value = 1./(NB_FRAME_ACCUMULATION*2)*cos(mean_error_distance_static*(M_PI/MAX_PTP_ERROR_STATIC))+1./(NB_FRAME_ACCUMULATION*2);
		value = std::exp(-1*mean_error_distance_static*mean_error_distance_static/(CST(double,cst::MAX_PTP_ERROR_STATIC)*CST(double,cst::MAX_PTP_ERROR_STATIC)))/CST(int,cst::NB_FRAME_ACCUMULATION); // Old
	}
	else // MALUS
	{
		if(mean_error_distance_move > CST(double,cst::MAX_PTP_ERROR_MOVING))
			mean_error_distance_move = CST(double,cst::MAX_PTP_ERROR_MOVING);

//		value = 1./(NB_FRAME_ACCUMULATION*2)*cos((mean_error_distance_move-MAX_PTP_ERROR_STATIC)*(M_PI/0.4))-1./(NB_FRAME_ACCUMULATION*2);
		value = -1 * std::exp(-1*pow((mean_error_distance_move-1.),2)/(CST(double,cst::MAX_PTP_ERROR_MOVING)*CST(double,cst::MAX_PTP_ERROR_MOVING)))/CST(int,cst::NB_FRAME_ACCUMULATION); // Old
	}

	return value;
}



//---------------------------------------------------------------------------------------
void CombiningGeometricConstraintOpticalFlow::updateConfidenceValue(uint nb_min_valid_point_to_bonus,
																	const std::vector<std::vector<uint>>& cluster_feature_point_id_list,
												 	 	 	 	 	const std::vector<uint>& index_list,
																	const std::vector<uint>& valid_distance,
																	const std::vector<double>& mean_error_distance_static,
																	const std::vector<double>& mean_error_distance_move,
																	std::vector<double>& cluster_bonus_malus_value,
																	std::vector<double>& bonus_malus_value) noexcept
{

//	double reconstruction_quality_t = data->getReconstructionQualityAtT();
//	double reconstruction_weight = reconstruction_quality_t/PERCENT_PC_TO_BE_STATIC;
//	reconstruction_weight = (reconstruction_weight>1.)?1.:reconstruction_weight;
//	std::cout << "reconstruction_weigth " << reconstruction_weight << std::endl;

	double reconstruction_quality_t = 0;
	double reconstruction_weight = 0;

//	reconstruction_quality_t = data->getReconstructionQualityAtT();
//	reconstruction_weight = (reconstruction_quality_t>0.3)?1.:0.7;

	// TODO : danger
	if(reconstruction_quality_t == 0 )
		reconstruction_weight = 1.;

	std::cout << "reconstruction_weigth " << reconstruction_weight << std::endl;

	uint nb_cluster = cluster_feature_point_id_list.size();
	for(int i=0; i<nb_cluster; i++)
	{
		double cluster_value = 0.;
		double sum_value = 0.;
		uint count_value = 0;

		// Compute bonus/malus value for the ith cluster
		uint nb_point_in_cluster = cluster_feature_point_id_list[i].size();
		for(int j=0; j<nb_point_in_cluster; j++)
		{
			uint point_index = cluster_feature_point_id_list[i][j];

			if(data->isFeaturePointTwoReconstructionsOld(point_index))
			{
				double bonus_malus = computeBonusMalusValue(nb_min_valid_point_to_bonus,
															valid_distance[point_index],
															mean_error_distance_static[point_index],
															mean_error_distance_move[point_index]);

				sum_value += bonus_malus;
				bonus_malus_value[point_index] = bonus_malus;

				count_value++;
			}
		}

		cluster_value = sum_value/count_value * reconstruction_weight;

		if(count_value == 0)
			cluster_value = 0;

		cluster_bonus_malus_value.push_back(cluster_value);

		// Update confidence value
		for(int j=0; j<nb_point_in_cluster; j++)
		{
			uint point_index = cluster_feature_point_id_list[i][j];
			data->updateConfidenceValue(point_index, cluster_value); // Mean bonus/malus value
		}
	}
}












