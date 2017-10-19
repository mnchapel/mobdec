/// @file   labelingFeaturePoint.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



//OpenCV
#include <opencv2/surface_matching.hpp>

// C++
#include <ctime>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdlib.h>
#include <vector>

// MoBDec
#include <core/dataWriter.h>
#include <core/phase.h>



/// @brief The LabelingProcess class
class LabelingFeaturePoint : public Phase {

//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

    /// @brief Default constructor
	LabelingFeaturePoint() noexcept = default;



	/// @brief Copy constructor
    LabelingFeaturePoint(const LabelingFeaturePoint&) noexcept = delete;



	/// @brief Move constructor
	LabelingFeaturePoint(LabelingFeaturePoint&&) noexcept = default;



	/// @brief Copy assignment operator
    LabelingFeaturePoint& operator=(const LabelingFeaturePoint&) noexcept = delete;



	/// @brief Move assignment operator
	LabelingFeaturePoint& operator=(LabelingFeaturePoint&&) noexcept = default;



    /// @brief Destructor.
    ~LabelingFeaturePoint() noexcept = default;



    /// @brief Get the component name.
    std::string getComponentName() const noexcept override;



//-----------------------------------------------------------------------------
// PROTECTED MEMBER FUNCTION
protected:

    /// @brief compute
    void compute() noexcept override;



    /// @brief
    ///
    /// @param index_i:
    /// @param valid_distance:
    /// @param valid_index:
    /// @param mean_error_distance_static
    /// @param mean_error_distance_move
    void compareToOtherStaticFeaturePoint(uint index_i,
                                          std::vector<uint>& valid_distance,
										  std::vector< std::vector<uint> >& valid_index,
                                          double& mean_error_distance_static,
                                          double& mean_error_distance_move) noexcept;



    /// @brief computePtPDistanceOnValidPercent
    ///
    /// @param index_list:
    /// @param valid_distance:
    /// @param mean_error_distance_static:
    /// @param mean_error_distance_move:
    ///
    /// @return
    uint computePtPDistanceOnValidPercent(std::vector<uint>& index_list,
    									  std::vector<uint>& valid_distance,
										  std::vector<double>& mean_error_distance_static,
										  std::vector<double>& mean_error_distance_move) noexcept;
                                          
                                          
                                          
    /// @brief .
    ///
    /// @return true if enough time elapsed to run the phase, false else.
    bool isEnoughTimeElapsed() const noexcept override;



    
    /// @brief checkDifferencePtPDistance
    ///
    /// @param index_i
    /// @param index_j
    /// @param sum_distance:
    /// @param count_distance:
    ///
    /// @return
    bool isPointStable(uint index_i,
                       uint index_j,
                       double &sum_distance_moving,
                       uint &count_distance_moving,
                       double &sum_distance_static,
                       uint &count_distance_static) noexcept;



    /// @brief
    void readFileData() noexcept override;



    /// @brief updateBonusConfidenceIndex
    ///
    /// @param index
    /// @param mean_error_distance_static
    void updateBonusConfidenceIndex(uint index,
                                    double mean_error_distance_static) noexcept;



    /// @brief
    ///
    /// @param uint nb_min_valid_point_to_bonus:
    /// @param valid_distance:
    /// @param mean_error_distance_static:
    /// @param mean_error_distance_move:
    void updateConfidenceValue(uint nb_min_valid_point_to_bonus,
			 	 	 	 	   const std::vector<uint>& index_list,
			 	 	 	 	   const std::vector<uint>& valid_distance,
							   const std::vector<double>& mean_error_distance_static,
							   const std::vector<double>& mean_error_distance_move) noexcept;



    /// @brief updateMalusConfidenceIndex
    ///
    /// @param index
    /// @param mean_error_distance_move
    void updateMalusConfidenceIndex(uint index,
                                    double mean_error_distance_move) noexcept;


};
