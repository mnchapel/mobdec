/// @file   combiningGeometricConstraintOpticalFlow.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// C++
#include <ctime>

// MoBDec
#include <core/dataWriter.h>
#include <core/phase.h>



/// @brief The LabelingProcess class
class CombiningGeometricConstraintOpticalFlow : public Phase{

//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

    /// @brief Default constructor
	CombiningGeometricConstraintOpticalFlow() noexcept = default;



	/// @brief Copy constructor
    CombiningGeometricConstraintOpticalFlow(const CombiningGeometricConstraintOpticalFlow&) noexcept = delete;



	/// @brief Move constructor
	CombiningGeometricConstraintOpticalFlow(CombiningGeometricConstraintOpticalFlow&&) noexcept = default;



	/// @brief Copy assignment operator
    CombiningGeometricConstraintOpticalFlow& operator=(const CombiningGeometricConstraintOpticalFlow&) noexcept = delete;



	/// @brief Move assignment operator
	CombiningGeometricConstraintOpticalFlow& operator=(CombiningGeometricConstraintOpticalFlow&&) noexcept = default;



    /// @brief Destructor.
    ~CombiningGeometricConstraintOpticalFlow() noexcept = default;



    /// @brief Get the component name.
    std::string getComponentName() const noexcept override;



//-----------------------------------------------------------------------------
// PROTECTED MEMBER FUNCTION
protected:



    /// @brief
    ///
    /// @param index_i
    /// @param valid_distance
    /// @param valid_index
    /// @param mean_error_distance_static
    /// @param mean_error_distance_move
    void compareToOtherStaticFeaturePoint(uint index_i,
    							   	      std::vector<uint>& valid_distance,
										  std::vector< std::vector<uint> >& valid_index,
										  double& mean_error_distance_static,
										  double& mean_error_distance_move) noexcept;

    /// @brief compute
    void compute() noexcept override;



    /// @brief
    ///
    /// @param is_in_cluster:
    /// @param cluster:
    void computeAnOpticalFlowCluster(std::vector<bool>& is_in_cluster,
								     std::vector<uint>& cluster) noexcept;



    /// @brief
    ///
    /// @param index_list
    /// @param valid_distance
    /// @param mean_error_distance_static
    /// @param mean_error_distance_move
    ///
    /// @return
    uint computePtPDistanceOnValidPercent(std::vector<uint>& index_list,
    									  std::vector<uint>& valid_distance,
										  std::vector<double>& mean_error_distance_static,
										  std::vector<double>& mean_error_distance_move) noexcept;



    /// @brief
    ///
    /// @param cluster_feature_point_id_list:
    void createOpticalFlowCluster(std::vector<std::vector<uint>>& cluster_feature_point_id_list) noexcept;
    
    
    
    /// @brief .
    ///
    /// @return true if enough time elapsed to run the phase, false else.
    bool isEnoughTimeElapsed() const noexcept override;



    /// @brief
    ///
    /// @param i:
    /// @param j:
    ///
    /// @return
    bool isOpticalFlowNearToCluster(uint i,
									uint j) noexcept;



    /// @brief
    ///
    /// @param index_1:
    /// @param index_2:
    ///
    /// @return
    bool isPointNearToCluster(uint index_1,
						      uint index_2) noexcept;



    /// @brief
    ///
    /// @param index_i
    /// @param index_j
    /// @param sum_distance_moving
    /// @param count_distance_moving
    /// @param sum_distance_static
    /// @param count_distance_static
    ///
    /// @return
    bool isPointStable(uint index_i,
    				   uint index_j,
					   double& sum_distance_moving,
					   uint& count_distance_moving,
					   double& sum_distance_static,
					   uint& count_distance_static) noexcept;



    /// @brief
    ///
    /// @param begin
    /// @param end
    /// @param num_random
    ///
    /// @return
    std::vector<uint>::iterator random_unique(std::vector<uint>::iterator begin,
    							 	 	 	  std::vector<uint>::iterator end,
											  std::size_t num_random) noexcept;



    /// @brief
    ///
    /// @param cluster_feature_point_id_list
    void readClusterFile(std::vector<std::vector<uint>>& cluster_feature_point_id_list) noexcept;



    /// @brief
    void readFileData() noexcept override;



    /// @brief
    ///
    /// @param index
    /// @param mean_error_distance_static
    void updateBonusConfidenceIndex(uint index,
                                    double mean_error_distance_static) noexcept;



	/// @brief
	///
	/// @param nb_min_valid_point_to_bonus
	/// @param valid_distance
	/// @param mean_error_distance_static
	/// @param mean_error_distance_move
	double computeBonusMalusValue(uint nb_min_valid_point_to_bonus,
								  uint valid_distance,
								  double mean_error_distance_static,
								  double mean_error_distance_move) noexcept;



    /// @brief
    ///
    /// @param nb_min_valid_point_to_bonus
    /// @param cluster_feature_point_id_list
    /// @param index_list
    /// @param valid_distance
    /// @param mean_error_distance_static
    /// @param mean_error_distance_move
    /// @param cluster_bonus_malus_value
    /// @param bonus_malus_value
    void updateConfidenceValue(uint nb_min_valid_point_to_bonus,
							   const std::vector<std::vector<uint>>& cluster_feature_point_id_list,
    		 	 	 	 	   const std::vector<uint>& index_list,
							   const std::vector<uint>& valid_distance,
							   const std::vector<double>& mean_error_distance_static,
							   const std::vector<double>& mean_error_distance_move,
							   std::vector<double>& cluster_bonus_malus_value,
							   std::vector<double>& bonus_malus_value) noexcept;



    /// @brief
    ///
    /// @param index
    /// @param mean_error_distance_move
    void updateMalusConfidenceIndex(uint index,
                                    double mean_error_distance_move) noexcept;


};





















