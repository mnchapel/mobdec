/// @file   deleteFeaturePointWithGroundTruth.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// C++
#include <ctime>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <stdlib.h>
#include <thread>
#include <vector>

// MoBDec
#include <core/dataWriter.h>
#include <core/phase.h>



/// @brief The Camera class
class DeleteFeaturePointWithGroundTruth : public Phase{

//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

    /// @brief Default constructor
	DeleteFeaturePointWithGroundTruth() noexcept = default;



	/// @brief Copy constructor
    DeleteFeaturePointWithGroundTruth(const DeleteFeaturePointWithGroundTruth&) noexcept = delete;



	/// @brief Move constructor
	DeleteFeaturePointWithGroundTruth(DeleteFeaturePointWithGroundTruth&&) noexcept = default;



	/// @brief Copy assignment operator
    DeleteFeaturePointWithGroundTruth& operator=(const DeleteFeaturePointWithGroundTruth&) noexcept = delete;



	/// @brief Move assignment operator
	DeleteFeaturePointWithGroundTruth& operator=(DeleteFeaturePointWithGroundTruth&&) noexcept = default;



    /// @brief Destructor.
    ~DeleteFeaturePointWithGroundTruth() noexcept = default;



    /// @brief Get the component name.
    std::string getComponentName() const noexcept override;



    /// @brief
    void readFileData() noexcept override;



//-----------------------------------------------------------------------------
// PROTECTED MEMBER FUNCTION
protected:

    /// @brief compute
    void compute() noexcept override;



    /// @brief findMovingFeaturePoint
    ///
    /// @param id_moving_feature_point
    void findMovingFeaturePoint(std::vector<uint>& id_moving_feature_point) noexcept;



    /// @brief deleteFeaturePoint
    ///
    /// @param id_moving_feature_point
    void deleteFeaturePoint(const std::vector<uint> &id_moving_feature_point) noexcept;
    
    
    
    /// @brief .
    ///
    /// @return true if enough time elapsed to run the phase, false else.
    bool isEnoughTimeElapsed() const noexcept override;

};
