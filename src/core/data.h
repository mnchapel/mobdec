/// @file   data.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// C++
#include <array>

// MoBDec
#include <core/camera.h>
#include <core/featurePoint.h>
#include <core/utils.h>
#include <core/video.h>

// OpenCV


/// @brief
class Data {

//-----------------------------------------------------------------------------
// PRIVATE MEMBER DATA
private:

	/// Parameters of the camera.
	Camera camera;

	///
	uint end_initialisation_time = 0;

	///
	uint end_process_time = 0;

	///
	std::vector<FeaturePoint> feature_point;

	///
	std::vector<uint> static_feature_point;

	///
	uint last_good_reconstruction = 0;

	///
	std::vector<double> reconstruction_quality;

	///
	std::vector<uint> save_time;

	///
    uint start_time = 0;

	/// The current time.
	uint time = 0;

	///
	Video video;



//-----------------------------------------------------------------------------
// PUBLIC FUNCTION DATA
public:

	/// @brief Default constructor
	Data() = default;



	/// @brief Constructor
	///
	/// @param video:
	/// @param camera:
	Data(Video& video,
		 Camera& camera) noexcept;



	/// @brief Copy constructor
	Data(const Data&) = default;



	/// @brief Move constructor
	Data(Data&&) = default;



	/// @brief Copy assignment operator
	Data& operator=(const Data&) = default;



	/// @brief Move assignment operator
	Data& operator=(Data&&) = default;



	/// @brief Destructor
	~Data() = default;



	/// @brief Add a new feature point.
	///
	/// @param position_2d: the 2d position of the feature point in the current image.
    void addNewFeaturePoint(cv::Mat& position_2d) noexcept;



	/// @brief
	void clearStaticFeaturePoint() noexcept;



	/// @brief Delete a feature point.
	///
	/// @param index: the feature point index.
	void deleteFeaturePoint(uint index) noexcept;



	/// @brief
	///
	/// @param id
	/// @param has_match
	/// @param position_2d
	/// @param position_3d
	void featurePointReturnToSave(uint id,
								  bool has_match,
								  cv::Mat& position_2d,
								  double distance_to_camera) noexcept;



	/// @brief Find the index of the first free space in the feature point array.
	///
	/// @return index of the first free space in the feature point array.
	uint findFirstFeaturePointFreeSpace() noexcept;



	/// @brief
	///
	/// @param id:
	///
	/// @return
	bool isFeaturePoint(uint id) const noexcept;



	/// @brief
	///
	/// @param id: id of the feature point.
	///
	/// @return
	bool isFeaturePointOld(uint id) const noexcept;



	/// @brief
	///
	/// @param id:
	///
	/// @return true if the 3d position of the feature point can be estimated.
	bool isFeaturePointReconstructionOld(uint id) const noexcept;



	/// @brief
	///
	/// @param id: the id of the feature point.
	///
	/// @return
	bool isFeaturePointOpticalFlowOld(uint id) const noexcept;



	/// @brief
	///
	/// @param id
	///
	/// @return
	bool isFeaturePointScaleOld(uint id) const noexcept;



	/// @brief
	///
	/// @param id
	///
	/// @return
	bool isFeaturePointTwoReconstructionsOld(uint id) const noexcept;



	/// @brief
	///
	/// @return
	uint getEndInitialisationTime() const noexcept;



	/// @brief
	///
	/// @return
	uint getEndProcessTime() const noexcept;



	/// @brief
	///
	/// @param index: the index of the feature point.
	///
	/// @return the age of the feature point.
	uint getFeaturePointAge(uint index) const noexcept;



	/// @brief
	///
	/// @param index : the index of the feature point.
	///
	/// @return the confidence value of the feature point.
	double getFeaturePointConfidenceValue(uint index) const noexcept;



	/// @brief
	///
	/// @param index: the index of the feature point.
	///
	/// @return the distance between the camera and the feature point in space.
	double getFeaturePointDistanceToCameraAtT(uint index) const noexcept;



	/// @brief
	///
	/// @param
	///
	/// @return
	double getFeaturePointDistanceToCameraAtTMDelta(uint index) const noexcept;



	/// @brief
	///
	/// @param index: the index of the feature point.
	///
	/// @return the label of the feature point.
	double getFeaturePointLabel(uint index) const noexcept;



	/// @brief
	///
	/// @param index: the index of the feature point.
	///
	/// @return the 2D optical flow of the feature point between T and T-1
	cv::Mat getFeaturePointOpticalFlowTM1(uint index) const noexcept;



	/// @brief
	///
	/// @param index: the index of the feature point.
	///
	/// @return the 2D optical flow of the feature point between T and T-Delta T
	cv::Mat getFeaturePointOpticalFlowTMDelta(uint index) const noexcept;



	/// @brief
	///
	/// @param id:
	///
	/// @return
    std::vector<cv::Mat>::const_iterator getFeaturePointPosition2DAtT(uint id) const noexcept;



	/// @brief
	///
	/// @param id:
	///
	/// @return
	const cv::Mat& getFeaturePointPosition2DAtTM1(uint id) const noexcept;



	/// @brief
	///
	/// @param id:
	///
	/// @return
	const cv::Mat& getFeaturePointPosition2DAtTMDelta(uint id) const noexcept;



	/// @brief
	///
	/// @param time
	/// @param id
//	const cv::Mat getFeaturePointPosition3DAt(uint time
//											 ,uint id) const noexcept;



	/// @brief
	///
	/// @param t
	/// @param id
	///
	/// @return
	std::vector<cv::Mat>::const_iterator getFeaturePointPosition3DAt(uint t
												   	  	  	  	  	,uint id) const noexcept;



	/// @brief
	///
	/// @param id:
	///
	/// @return
	const cv::Mat getFeaturePointPosition3DAtT_Old(uint id) const noexcept;



	/// @brief
	///
	/// @param id
	///
	/// @return
	std::vector<cv::Mat>::const_iterator getFeaturePointPosition3DAtT(uint id) const noexcept;



	/// @brief
	///
	/// @param id:
	///
	/// @return
	std::vector<cv::Mat>::const_iterator getFeaturePointPosition3DAtTM1(uint id) const noexcept;



	/// @brief
	///
	/// @param id:
	///
	/// @return
	std::vector<cv::Mat>::const_iterator getFeaturePointPosition3DAtTMDelta(uint id) const noexcept;



	/// @brief
	///
	/// @param id:
	///
	/// @return
	const FeaturePoint& getFeaturePoint(uint id) const noexcept;



	/// @brief
	///
	/// @return
	const cv::Mat& getImageAtT() const noexcept;



	/// @brief
	///
	/// @return
	const cv::Mat& getImageAtTM1() const noexcept;



    /// @brief
    ///
    /// @return the number of columns in the image
    const uint getImageCol() const noexcept;



    /// @brief
    ///
    /// return the number of rows in the image
    const uint getImageRow() const noexcept;



	/// @brief getIntrinsicParameters
	///
	/// @return
	const cv::Mat& getIntrinsicParameters() const noexcept;



	/// @brief
	///
	/// @return
	uint getLastGoodReconstruction() const noexcept;



	/// @brief getNbStaticPoint
	///
	/// @return
	uint getNbStaticPoint() const noexcept;



	/// @brief
	///
	/// @return the rotation matrix
	const cv::Mat& getRotationAtT() const noexcept;



	/// @brief
	///
	/// @return
	double getReconstructionQualityAtT() const noexcept;



	//---------------------------------------------------------------------------------------
	std::vector<uint> getStaticPointIndex() const noexcept;



	/// @brief
	///
	/// @return
	std::vector<uint>::iterator getStaticPointIndexBegin() noexcept;



	/// @brief
	///
	/// @return
	std::vector<uint>::iterator getStaticPointIndexEnd() noexcept;



	/// @brief
	///
	/// @return
	uint getTime() const noexcept;



	/// @brief
	///
	/// @return
	uint getTimeElapsed() const noexcept;



	/// @brief
	///
	/// @return the translation matrix
	const cv::Mat& getTranslationAtT() const noexcept;



	/// @brief
	///
	/// @return
	const std::vector<uint>& getSaveTime() const noexcept;



	/// @brief
	///
	/// @return
    uint getStartTime() const noexcept;



	/// @brief
	///
	/// @param feature_point_id: the id of the feature point
	///
	/// @return
	bool hasPosition3DComputedAtTM1(uint feature_point_id) const noexcept;



	/// @brief
	///
	/// @param feature_point_id: the id of the feature point
	///
	/// @return
	bool hasPosition3DComputedAtTMDelta(uint feature_point_id) const noexcept;



	/// @brief
	///
	/// @param id:
	///
	/// @return
    bool hasTMDeltaMatching(uint id) const noexcept;



	/// @brief
	///
	/// @return
	bool isFeaturePointStatic(uint index) const noexcept;



	/// @brief
	void nextFrame() noexcept;



	/// @brief
	void saveCurrentState() noexcept;



	/// @brief
	///
	/// @param frame_num: the frame number.
	void setLastGoodReconstruction(uint frame_num) noexcept;



	/// @brief
	///
	/// @param index:
	/// @param bonus_malus:
	void updateConfidenceValue(uint index,
							   double bonus_malus) noexcept;



	/// @brief
	///
	/// @param rotation
	/// @param translation
	void updateExtrinsicParameters(cv::Mat& rotation,
								   cv::Mat& translation) noexcept;



	/// @brief
	///
	/// @param index: the index of the feature point to update.
	/// @param position_2d: the new 2D position.
	void updateFeaturePoint2DPosition(uint index,
									  const cv::Mat& position_2d) noexcept;



	/// @brief
	///
	/// @param index: the index of the feature point to update.
	void updateFeaturePointAge(uint index) noexcept;



	/// @brief
	///
	/// @param index: the index of the feature point to update.
	/// @param distance_to_camera: the new distance.
	void updateFeaturePointDistanceToCamera(uint index,
		  								    double distance_to_camera) noexcept;



	/// @brief
	///
	/// @param quality
    void updateReconstructionQuality(double quality) noexcept;



//-----------------------------------------------------------------------------
// PRIVATE FUNCTION DATA
private:



	/// @brief
	///
	/// @param feature_point_id
	void removeStaticPoint(uint feature_point_id) noexcept;



};
