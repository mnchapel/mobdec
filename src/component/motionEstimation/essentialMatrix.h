/// @file   essentialMatrix.h
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



/// @brief EssentialMatrixInitialization
class EssentialMatrix : public Phase{

//-----------------------------------------------------------------------------
// PROTECTED MEMBER DATA
protected:

	//!
	uint n=0;

	//!
	uint t=0;



//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

    /// @brief Default constructor
	EssentialMatrix() = default;



    /// @brief Copy constructor
    EssentialMatrix(const EssentialMatrix&) = delete;



	/// @brief Move constructor
    EssentialMatrix(EssentialMatrix&&) = default;



	/// @brief Copy assignment operator
    EssentialMatrix& operator=(const EssentialMatrix&) = delete;



	/// @brief Move assignment operator
    EssentialMatrix& operator=(EssentialMatrix&&) = default;



    /// @brief Destructor.
    ~EssentialMatrix() = default;



    /// @brief Get the component name.
    std::string getComponentName() const noexcept override;



    /// @brief
    ///
    /// @return
    std::string getPhaseReturn() const noexcept override;



    /// @brief
    void readFileData() noexcept override;



//-----------------------------------------------------------------------------
// PROTECTED MEMBER FUNCTION
protected:

    /// @brief compute
    void compute() noexcept override;



    /**
     * @brief computeExtrinsicParameters
     *
     * @param feature_point_t
     * @param feature_point_tmdelta
     */
    void computeExtrinsicParameters(const std::vector<cv::Point2d>& feature_point_t,
			   	   	   	   	   	   	const std::vector<cv::Point2d>& feature_point_tmdelta) noexcept;



    /**
     * @brief computeBundleAdjustment
     */
    void computeBundleAdjustment() noexcept;



    /**
     * @brief computeEssentialMatrix
     *
     * @param feature_point_t
     * @param feature_point_tmdelta
     *
     * @return
     */
    cv::Mat computeEssentialMatrix(const std::vector<cv::Point2d> &feature_point_t,
                                   const std::vector<cv::Point2d> &feature_point_tmdelta) noexcept;



    /**
     * @brief computeRotationTranslation
     *
     * @param essential_matrix
     * @param feature_point_t
     * @param feature_point_tmdelta
     * @param rotation,
     * @param translation
     */
    void computeRotationTranslation(const cv::Mat &essential_matrix,
                                    const std::vector<cv::Point2d>& feature_point_t,
                                    const std::vector<cv::Point2d>& feature_point_tmdelta,
                                    cv::Mat& rotation,
                                    cv::Mat& translation) noexcept;



    /**
     * @brief getAllFeaturePointAtTTMDelta
     *
     * @param feature_point_t
     * @param feature_point_tmdelta
     */
    void getAllFeaturePointAtTTMDelta(std::vector<cv::Point2d> &feature_point_t,
                                      std::vector<cv::Point2d> &feature_point_tmdelta) noexcept;



    /// @brief
    ///
    /// @param static_feature_point_t:
    /// @param static_feature_point_tmdelta:
    void getStaticFeaturePoint(std::vector<cv::Point2d>& static_feature_point_t,
					    	   std::vector<cv::Point2d>& static_feature_point_tmdelta) const noexcept;
                               
                               
                               
    /// @brief .
    ///
    /// @return true if enough time elapsed to run the phase, false else.
    bool isEnoughTimeElapsed() const noexcept override;



    /// @brief
    bool isTooNearToOtherFeaturePoint(uint index) const noexcept;



    /// @brief
    ///
    /// @param begin
    /// @param end
    /// @param num_random
    ///
    /// @return
    std::vector<uint>::iterator random_unique(std::vector<uint>::iterator begin,
											  std::vector<uint>::iterator end,
											  std::size_t num_random) const noexcept;

};
