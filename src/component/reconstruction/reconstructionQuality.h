/// @file   reconstructionQuality.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// C++
#include <chrono>
#include <ctime>
#include <iostream>
#include <stdlib.h>
#include <vector>

// MoBDec
#include <core/dataWriter.h>
#include <core/phase.h>



/// @brief Scale
class ReconstructionQuality : public Phase{

//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

    /// @brief Default constructor
	ReconstructionQuality() noexcept = default;



	/// @brief Copy constructor
    ReconstructionQuality(const ReconstructionQuality&) noexcept = delete;



	/// @brief Move constructor
	ReconstructionQuality(ReconstructionQuality&&) noexcept = default;



	/// @brief Copy assignment operator
    ReconstructionQuality& operator=(const ReconstructionQuality&) noexcept = delete;



	/// @brief Move assignment operator
	ReconstructionQuality& operator=(ReconstructionQuality&&) noexcept = default;



    /// @brief Destructor.
    ~ReconstructionQuality() noexcept = default;



    /// @brief Get the component name.
    std::string getComponentName() const noexcept override;



    /// @brief
    ///
    /// @return
    std::string getPhaseReturn() const noexcept override;



//-----------------------------------------------------------------------------
// PROTECTED MEMBER FUNCTION
protected:

    /// @brief
    void compareToOtherStaticFeaturePoint(uint index_i,
										  std::vector<uint>& valid_distance,
										  double& mean_error_distance_static,
										  double& mean_error_distance_move) const noexcept;



    /// @brief compute
    void compute() noexcept override;



    /// @brief
    ///
    /// @param
    uint computePtPDistanceOnValidPercent(std::vector<uint>& index_list,
    									  std::vector<uint>& valid_distance,
										  std::vector<double>& mean_error_distance_static,
										  std::vector<double>& mean_error_distance_move) const noexcept;



    /// @brief
    ///
    /// @param
    uint computePtPDistanceOnValidPercentTMDelta(std::vector<uint>& index_list,
    									  	     std::vector<uint>& valid_distance,
												 std::vector<double>& mean_error_distance_static,
												 std::vector<double>& mean_error_distance_move) const noexcept;



    /// @brief .
    ///
    /// @return true if enough time elapsed to run the phase, false else.
    bool isEnoughTimeElapsed() const noexcept override;



    /// @brief
    ///
    /// @return
    bool isGoodReconstruction() const noexcept;



    /// @brief
    bool isPointStable(uint index_i,
    				   uint index_j,
					   double& sum_distance_moving,
					   uint& count_distance_moving,
					   double& sum_distance_static,
					   uint& count_distance_static) const noexcept;



    /// @brief
    ///
    /// @param
    std::vector<uint>::iterator random_unique(std::vector<uint>::iterator begin,
    										  std::vector<uint>::iterator end,
											  std::size_t num_random) const noexcept;



    /// @brief
    void readFileData() noexcept override;

};
