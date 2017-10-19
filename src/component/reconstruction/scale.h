/// @file   scale.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// C++
#include <chrono>
#include <ctime>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdlib.h>
#include <vector>

// MoBDec
#include <core/dataWriter.h>
#include <core/phase.h>



/// @brief Scale
class Scale : public Phase{

//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

    /// @brief Default constructor
    Scale() noexcept = default;



	/// @brief Copy constructor
    Scale(const Scale&) noexcept = delete;



	/// @brief Move constructor
    Scale(Scale&&) noexcept = default;



	/// @brief Copy assignment operator
    Scale& operator=(const Scale&) noexcept = delete;



	/// @brief Move assignment operator
    Scale& operator=(Scale&&) noexcept = default;



    /// @brief Destructor.
    ~Scale() noexcept = default;



    /// @brief Get the component name.
    std::string getComponentName() const noexcept override;



//-----------------------------------------------------------------------------
// PROTECTED MEMBER FUNCTION
protected:

    /// @brief compute
    void compute() noexcept override;



    /// @brief
    void computeScale() noexcept;



    /// @brief computeDistanceScale
    ///
    /// @param scale
    void computeDistanceScale(double scale) noexcept;



    /// @brief computeMedianScale
    ///
    /// @return
    double computeMedianScale() noexcept;



    /// @brief computeMedianScaleForAPoint
    ///
    /// @param index
    ///
    /// @return
    double computeMedianScaleForAPoint(uint index) noexcept;



    /// @brief Compute the median scale for a point on SCALE_NB_SAMPLE_POINT points.
    ///
    /// @param index: the index of the feature point.
    ///
    /// @return the median scale.
    double computeMedianScaleForAPointOnRandomPoint(uint index) noexcept;



    /// @brief Compute the median scale on SCALE_NB_SAMPLE_POINT points.
    ///
    /// @return the median scale.
    double computeMedianScaleOnRandomPoint() noexcept;



    /// @brief .
    ///
    /// @return true if enough time elapsed to run the phase, false else.
    bool isEnoughTimeElapsed() const noexcept override;



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
    void readFileData() noexcept override;

};
