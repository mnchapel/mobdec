/// @file   lifeCycleManager.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// C++
#include <unordered_map>
#include <vector>

// MoBDec
#include <component/clustering/kmeansOpticalFlow.h>
#include <component/featureDetector/deleteFeaturePointWithGroundTruth.h>
#include <component/featureDetector/ldofDetector.h>
#include <component/labeling/combiningGeometricConstraintOpticalFlow.h>
#include <component/labeling/falseNegativeSuppression.h>
#include <component/labeling/labelingFeaturePoint.h>
#include <component/labeling/labelingStaticInitialisation.h>
#include <component/motionEstimation/essentialMatrix.h>
#include <component/motionEstimation/essentialMatrixKMeans.h>
#include <component/motionEstimation/slowdownStopMotionDetection.h>
#include <component/reconstruction/reconstruction3dTriangulation.h>
#include <component/reconstruction/reconstructionQuality.h>
#include <component/reconstruction/scale.h>
#include <component/reconstruction/scaleInBox.h>
#include <component/segmentation/superPixelSegmentation.h>
#include <core/data.h>
#include <core/phase.h>



/// @brief
class LifeCycleManager{

//-----------------------------------------------------------------------------
// PROTECTED MEMBER DATA
protected:

	///
	int end_time_cycle = 0;

    //!
    std::vector<std::unique_ptr<Phase>> phases;



//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

    /// @brief Default constructor.
    LifeCycleManager() = default;



    /// @brief Constructor.
    ///
    /// @param end_tile_cycle:
    /// @param list_phase:
    LifeCycleManager(int end_time_cycle,
    				 const std::vector<std::string>& list_phase) noexcept;



	/// @brief Copy constructor
    LifeCycleManager(const LifeCycleManager&) = delete;



	/// @brief Move constructor
    LifeCycleManager(LifeCycleManager&&) = default;



	/// @brief Copy assignment operator
    LifeCycleManager& operator=(const LifeCycleManager&) = delete;



	/// @brief Move assignment operator
    LifeCycleManager& operator=(LifeCycleManager&&) = default;



    /// @brief Destructor.
    ~LifeCycleManager() noexcept = default;



    /// @brief launch
    ///
    /// @param data
    void launch(Data* data) noexcept;



//-----------------------------------------------------------------------------
// PRIVATE MEMBER FUNCTION
private:

    /// @brief createPhaseResultDirectory
    ///
    /// @param data
    void createPhaseResultDirectory(const Data& data) const noexcept;



};
