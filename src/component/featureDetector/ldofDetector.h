/// @file   ldofDetector.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// C++
#include <iostream>
#include <stdlib.h>
#include <vector>

// OpenCV
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>

// MoBDec
#include <component/featureDetector/ldof/CFilter.h>
#include <component/featureDetector/ldof/CTensor.h>
#include <component/featureDetector/ldof/CVector.h>
#include <component/featureDetector/ldof/ldof.h>
#include <core/dataWriter.h>
#include <core/phase.h>
#include <core/settings.h>



/// @brief The Track class
class LdofOptF : public Phase{

//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

    /// @brief Default constructor.
    LdofOptF() = default;



	/// @brief Copy constructor
    LdofOptF(const LdofOptF&) = delete;



	/// @brief Move constructor
    LdofOptF(LdofOptF&&) = default;



	/// @brief Copy assignment operator
    LdofOptF& operator=(const LdofOptF&) = delete;



	/// @brief Move assignment operator
	LdofOptF& operator=(LdofOptF&&) = default;



    /// @brief Destructor.
    ~LdofOptF() = default;



    /// @brief getComponentName
    ///
    /// @return
    std::string getComponentName() const noexcept override;



//-----------------------------------------------------------------------------
// PROTECTED MEMBER FUNCTION
protected:

    /// @brief Compute
    void compute() noexcept override;



    /// @brief Load trajectories from files.
    void loadTrajectories() noexcept;



    /// @brief updateTracks
    ///
    /// @param size_x
    /// @param size_y
    /// @param unreliable
    /// @param forward
    void updateTracks(uint size_x,
                      uint size_y,
                      CMatrix<float> &unreliable,
                      CTensor<float> &forward) noexcept;



    /// @brief Track::computeOpticalFlow
    ///
    /// @param current_trajectories
    void computeOpticalFlow(const unsigned int subsampling_factor) noexcept;



    /// @brief computeCorners
    ///
    /// @param aImage
    /// @param aCorners
    /// @param aRho
    void computeCorners(CTensor<float>& aImage, CMatrix<float>& aCorners, float aRho) noexcept;



    /// @brief convertImageToPPM
    void convertImageToPPM() noexcept;



    /// @brief dt
    ///
    /// @param f
    /// @param d
    /// @param n
    void dt(CVector<float>& f, CVector<float>& d, int n) noexcept;



    /// @brief euclideanDistanceTransform
    ///
    /// @param aMatrix
    void euclideanDistanceTransform(CMatrix<float>& aMatrix) noexcept;
    
    
    
    /// @brief .
    ///
    /// @return true if enough time elapsed to run the phase, false else.
    bool isEnoughTimeElapsed() const noexcept override;



    /// @brief
    void readFileData() noexcept override;



    /// @brief readMiddlebury
    ///
    /// @param aFilename
    /// @param aFlow
    ///
    /// @return
    bool readMiddlebury(const char* aFilename, CTensor<float>& aFlow) noexcept;



    /// @brief writeMiddlebury
    ///
    /// @param aFilename
    /// @param aFlow
    ///
    /// @return
    bool writeMiddlebury(const char* aFilename, CTensor<float>& aFlow) noexcept;

};
