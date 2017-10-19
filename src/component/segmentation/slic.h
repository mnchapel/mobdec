/// @file slic.h.
///
/// @brief This file contains the class elements of the class Slic. This class is an
/// implementation of the SLIC Superpixel algorithm by Achanta et al. [PAMI'12,
/// vol. 34, num. 11, pp. 2274-2282].
/// This implementation is created for the specific purpose of creating
/// over-segmentations in an OpenCV-based environment.
///
/// @author Pascal Mettes.



#pragma once



// C++
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <float.h>



/* 2d matrices are handled by 2d vectors. */
#define vec2dd std::vector<std::vector<double>>
#define vec2di std::vector<std::vector<int>>
#define vec2db std::vector<std::vector<bool>>
/* The number of iterations run by the clustering algorithm. */
#define NR_ITERATIONS 10



/// @class Slic.
///
/// In this class, an over-segmentation is created of an image, provided by the
/// step-size (distance between initial cluster locations) and the colour
/// distance parameter.
class Slic {

//-----------------------------------------------------------------------------
// PRIVATE MEMBER DATA
private:

	/// The cluster assignments for each pixel.
	vec2di clusters;

	/// The distance values for each pixel.
	vec2dd distances;

	/// The LAB and xy values of the centers.
	vec2dd centers;

	/// The number of occurences of each center.
	std::vector<int> center_counts;

	/// The step size per cluster parameter and the colour (nc) and distance (ns)
	int step = 0;

	/// The colour parameter and distance (ns)
	int nc = 0;

	/// The distance parameter
	int ns = 0;



//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

	/// @brief Default constructor.
	Slic() = default;



    /// @brief Copy constructor.
    Slic(const Slic&) noexcept = delete;



    /// @brief Move constructor
    Slic(Slic&&) noexcept = default;



    /// @brief Copy assignment operator
    Slic& operator=(const Slic&) noexcept = delete;



    /// @brief Move assignment operator
    Slic& operator=(Slic&&) noexcept = default;



	/// @brief Destructor.
	~Slic();



	/// @brief Generate an over-segmentation for an image.
	///
	/// @param image:
	/// @param step:
	/// @param nc:
	void generate_superpixels(cv::Mat& image, int step, int nc);



	/// @brief Enforce connectivity for an image.
	///
	/// @param image:
	void create_connectivity(cv::Mat& image);



	/// @brief Draw functions. Resp. displayal of the centers and the contours.
	///
	/// @param image:
	/// @param colour:
	void display_center_grid(IplImage *image, CvScalar colour);



	/// @brief
	///
	/// @param image:
	/// @param colour:
	void display_contours(cv::Mat& image,
						  cv::Scalar colour);



	/// @brief
	///
	/// @param image:
	void colour_with_cluster_means(IplImage *image);



//-----------------------------------------------------------------------------
// PRIVATE MEMBER FUNCTION
private:

	/// @brief Compute the distance between a center and an individual pixel.
	///
	/// @param ci:
	/// @param pixel:
	/// @param colour:
	///
	/// @return
	double compute_dist(int ci,
						cv::Point pixel,
						cv::Vec3b colour);



	/// @brief Find the pixel with the lowest gradient in a 3x3 surrounding.
	///
	/// @param image:
	/// @param center:
	///
	/// @return
	cv::Point find_local_minimum(cv::Mat& image,
								 cv::Point center);



	/// @brief Remove the 2d vectors.
	void clear_data();



	/// @brief Initialize the 2D vectors.
	///
	/// @param image
	void init_data(cv::Mat& image);



	/// @brief
	void readFileData() noexcept;

};
