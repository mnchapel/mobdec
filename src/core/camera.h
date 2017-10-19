/// @file   camera.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// C++

// MoBDec
#include <core/settings.h>

// OpenCV
#include <opencv2/opencv.hpp>



/// @brief Camera class
class Camera{

//-----------------------------------------------------------------------------
// PRIVATE MEMBER DATA
private:

	/// Intrinsic parameters of the camera
	cv::Mat intrinsic_parameters;

	/// Extrinsic parameters of the camera (0: rotation, 1: translation)
	std::vector<std::vector<cv::Mat>> extrinsic_parameters;



//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

	/// @brief Default constructor
    Camera()  = default;



	/// @brief
    Camera(const cv::Mat& intrinsic_parameters) noexcept;



	/// @brief Copy constructor
    Camera(const Camera&) = default;



	/// @brief Move constructor
    Camera(Camera&&) = default;



	/// @brief Copy assignment operator
    Camera& operator=(const Camera&) = default;



	/// @brief Move assignment operator
    Camera& operator=(Camera&&) = default;



    /// @brief Destructor.
    ~Camera() = default;



    /// @brief Destructor.
    const cv::Mat& getIntrinsicParameters() const noexcept;



    /// @brief
    ///
    /// @param index
    ///
    /// @return the rotation matrix from extrinsic parameters
    const cv::Mat& getRotation(uint index) const noexcept;



    /// @brief
    ///
    /// @param index
    ///
    /// @return the translation matrix from extrinsic parameters
    const cv::Mat& getTranslation(uint index) const noexcept;



    /// @brief
    ///
    /// @param indexAtT
    /// @param rotation
    /// @param translation
    void updateExtrinsicParameters(uint indexAtT,
    							   cv::Mat& rotation,
                                   cv::Mat& translation) noexcept;

};
