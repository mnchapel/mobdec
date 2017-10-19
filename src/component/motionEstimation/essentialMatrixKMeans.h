/// @file   essentialMatrixKMeans.h
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

// OpenCV
#include <opencv2/calib3d.hpp>

// MoBDec
#include <core/dataWriter.h>
#include <core/phase.h>



/// @brief EssentialMatrixInitialization
class EssentialMatrixKMeans : public Phase{

//-----------------------------------------------------------------------------
// PROTECTED MEMBER DATA
protected:



//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

    /// @brief Default constructor
	EssentialMatrixKMeans() = default;



    /// @brief Copy constructor
    EssentialMatrixKMeans(const EssentialMatrixKMeans&) = delete;



	/// @brief Move constructor
	EssentialMatrixKMeans(EssentialMatrixKMeans&&) = default;



	/// @brief Copy assignment operator
    EssentialMatrixKMeans& operator=(const EssentialMatrixKMeans&) = delete;



	/// @brief Move assignment operator
	EssentialMatrixKMeans& operator=(EssentialMatrixKMeans&&) = default;



    /// @brief Destructor.
    ~EssentialMatrixKMeans() = default;



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
    void computeExtrinsicParameters(const std::vector<cv::Point2f>& feature_point_t,
			   	   	   	   	   	   	const std::vector<cv::Point2f>& feature_point_tmdelta) noexcept;



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
    cv::Mat computeEssentialMatrix(const std::vector<cv::Point2f> &feature_point_t,
                                   const std::vector<cv::Point2f> &feature_point_tmdelta) noexcept;



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
                                    const std::vector<cv::Point2f>& feature_point_t,
                                    const std::vector<cv::Point2f>& feature_point_tmdelta,
                                    cv::Mat& rotation,
                                    cv::Mat& translation) noexcept;



    /// @brief
    ///
    /// @param static_feature_point_t:
    /// @param static_feature_point_tmdelta:
    /// @param kmeans_centers_t:
    /// @param kmeans_centers_tmdelta
    void findCorrespondingTMDelta(const std::vector<cv::Point2f>& static_feature_point_t,
    							  const std::vector<cv::Point2f>& static_feature_point_tmdelta,
    							  const std::vector<cv::Point2f>& kmeans_centers_t,
								  std::vector<cv::Point2f>& kmeans_centers_tmdelta) const noexcept;



    /**
     * @brief getAllFeaturePointAtTTMDelta
     *
     * @param feature_point_t
     * @param feature_point_tmdelta
     */
    void getAllFeaturePointAtTTMDelta(std::vector<cv::Point2f> &feature_point_t,
                                      std::vector<cv::Point2f> &feature_point_tmdelta) noexcept;



    /// @brief
    ///
    /// @param static_feature_point_t:
    /// @param static_feature_point_tmdelta:
    void getStaticFeaturePoint(std::vector<cv::Point2f>& static_feature_point_t,
					    	   std::vector<cv::Point2f>& static_feature_point_tmdelta) const noexcept;
                               
                               
                               
    /// @brief .
    ///
    /// @return true if enough time elapsed to run the phase, false else.
    bool isEnoughTimeElapsed() const noexcept override;



    /// @brief
    ///
    /// @param static_point_t:
    /// @param kmean_centers:
    void selectPointForEssentialMatrixNister(const cv::Mat& static_point_t,
											 std::vector<cv::Point2f>& kmean_centers) noexcept;

};
