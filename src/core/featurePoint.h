/// @file   featurePoint.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// C++
#include <cstdlib>

// MoBDec
#include <core/labeling.h>
#include <core/utils.h>

// OpenCV
#include <opencv2/opencv.hpp>



/// @class FeaturePoint
/// @brief
class FeaturePoint {

//-----------------------------------------------------------------------------
// PRIVATE MEMBER DATA
private:

	/// Age of the feature point (number of frame).
	uint age = 0;

	/// The confidence value of the feature point.
	double confidence_value = 0;

	/// The descriptor vector of the feature point.
	cv::Mat descriptor;

	/// Distances between the camera and the feature point.
	std::vector<double> distance_to_camera;//[CST(int,cst::NB_ROWS)];

	/// A previous 3D position in the image saved.
	std::vector<double> distance_to_camera_saved;//[CST(int,cst::NB_ROWS)];

	///
	bool valid_tmdelta_match = false;

	///
	std::vector<bool> has_position_3d;

	/// The label of the feature point (STATIC, MOVING, UNCERTAIN, UNLABELED).
	char label = Labeling::UNLABELED;

	/// 2D position in the image.
	std::vector<cv::Mat> position_2d;

	// TODO remove
	std::vector<cv::Mat> test;

	/// 3D position in the space.
	std::vector<cv::Mat> position_3d;



//-----------------------------------------------------------------------------
// PUBLIC FUNCTION DATA
public:

    /// @brief Default constructor
	FeaturePoint() noexcept;



	/// @brief Constructor.
	///
	/// @param position_2d:
	FeaturePoint(cv::Mat& position_2d) noexcept;



	/// @brief Copy constructor
	FeaturePoint(const FeaturePoint&) = default;



	/// @brief Move constructor
	FeaturePoint(FeaturePoint&&) = default;



	/// @brief Copy assignment operator
	FeaturePoint& operator=(const FeaturePoint&) = default;



	/// @brief Move assignment operator
	FeaturePoint& operator=(FeaturePoint&&) = default;



    /// @brief Destructor.
    ~FeaturePoint() noexcept = default;



	/// @brief
	void deletePoint() noexcept;



	/// @brief
	///
	/// @param t
	///
	/// @return
	const cv::Mat& get2DPositionAt(uint t) const noexcept;



	/// @brief
	///
	/// @return the 2D position
	std::vector<cv::Mat>::const_iterator get2DPositionAtT() const noexcept;



	// TODO remove
	std::vector<cv::Mat>::const_iterator testFunc() const noexcept;



	/// @brief
	///
	/// @return the 2D position
	const cv::Mat& get2DPositionAtTM1() const noexcept;



	/// @brief
	///
	/// @return the 2D position
	const cv::Mat& get2DPositionAtTMDelta() const noexcept;



	/// @brief
	///
	/// @return the 3D position
	std::vector<cv::Mat>::const_iterator get3DPositionAtT() const noexcept;



	/// @brief
	///
	/// @return the 3D position
	std::vector<cv::Mat>::const_iterator get3DPositionAtTM1() const noexcept;



	/// @brief
	///
	/// @return the 3D position
	std::vector<cv::Mat>::const_iterator get3DPositionAtTMDelta() const noexcept;



	/// @brief
	///
	/// @param
	///
	/// @return
	std::vector<cv::Mat>::const_iterator get3DPositionAt(uint t) const noexcept;



	/// @brief
	///
	/// @return
	uint getAge() const noexcept;



	/// @brief
	///
	/// @return
	double getConfidenceValue() const noexcept;



	/// @brief
	///
	/// @param t:
	///
	/// @return
	double getDistanceToCameraAt(uint t) const noexcept;



	/// @brief
	///
	/// @return
	double getDistanceToCameraAtT() const noexcept;



	/// @brief
	///
	/// @return
	double getDistanceToCameraAtTM1() const noexcept;



	/// @brief
	///
	/// @return
	double getDistanceToCameraAtTMDelta() const noexcept;



	/// @brief
	///
	/// @return
	char getLabel() const noexcept;



	/// @brief
	///
	/// @return
	cv::Mat getOpticalFlowTM1() const noexcept;



	/// @brief
	///
	/// @return
	cv::Mat getOpticalFlowTMDelta() const noexcept;



	/// @brief
	///
	/// @return
	bool hasPosition3DComputedAtTM1() const noexcept;



	/// @brief
	///
	/// @return
	bool hasPosition3DComputedAtTMDelta() const noexcept;



	/// @brief
	///
	/// @return
	bool hasValidTMDeltaMatch() const noexcept;



	/// @brief Replace the last 2D position by the 2D position saved.
	///
	/// @param has_match
	/// @param position_2d_old
	/// @param distance_to_camera_old
	void returnTo2DPositionSavedAtTMDelta(bool has_match,
										  const cv::Mat& position_2d_old,
										  double distance_to_camera_old) noexcept;



	/// @brief
	///
	/// @param position_2d
	void update2DPosition(const cv::Mat& position_2d) noexcept;



	/// @brief
	///
	/// @param position_3d
	void update3DPosition(const cv::Mat& position_3d) noexcept;



	/// @brief
	void updateAge() noexcept;



	/// @brief
	///
	/// @param bonus_malus:
	void updateConfidenceValue(double bonus_malus) noexcept;



	/// @brief
	///
	/// @param distance_to_camera
    void updateDistanceToCamera(double distance_to_camera) noexcept;

};
