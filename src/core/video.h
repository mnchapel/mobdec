/// @file   video.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// C++
#include <core/settings.h>
#include <array>
#include <cassert>
#include <string>

// MoBDec
#include <opencv2/opencv.hpp>



/// @brief
class Video{

//-----------------------------------------------------------------------------
// PRIVATE MEMBER DATA
private:

	/// The number of the current image.
	uint current_index = 0;

    /// The images of the video sequence
    std::vector<cv::Mat> image;



//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

	/// @brief Default constructor
	Video() noexcept;



	/// @brief Copy constructor
	Video(const Video&) = default;



	/// @brief Move constructor
	Video(Video&&) = default;



	/// @brief Copy assignment operator
	Video& operator=(const Video&) = default;



	/// @brief Move assignment operator
	Video& operator=(Video&&) = default;



	/// @brief Destructor
	~Video() = default;



    /// @brief
    ///
    /// @param num_frame: the number of frame to read
    void readImage(uint num_frame) noexcept;



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
    /// @return
    const cv::Mat& getImageAtTMDelta() const noexcept;



    /// @brief
    ///
    /// @return the number of columns in the image
    const uint getImageCol() const noexcept;



    /// @brief
    ///
    /// return the number of rows in the image
    const uint getImageRow() const noexcept;



    /// @brief
    ///
    /// return
    uint getCurrentIndex() const {return current_index;}

};
