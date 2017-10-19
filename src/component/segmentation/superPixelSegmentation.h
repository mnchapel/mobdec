/// @file   superPixelSegmentation.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// C++
#include <chrono>
#include <ctime>

// MoBDec
#include <core/dataWriter.h>
#include <core/phase.h>



/// @brief The SuperPixelSegmentation class
class SuperPixelSegmentation : public Phase{

//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

    /// @brief Default constructor
	SuperPixelSegmentation() noexcept = default;



	/// @brief Copy constructor
    SuperPixelSegmentation(const SuperPixelSegmentation&) noexcept = delete;



	/// @brief Move constructor
	SuperPixelSegmentation(SuperPixelSegmentation&&) noexcept = default;



	/// @brief Copy assignment operator
    SuperPixelSegmentation& operator=(const SuperPixelSegmentation&) noexcept = delete;



	/// @brief Move assignment operator
	SuperPixelSegmentation& operator=(SuperPixelSegmentation&&) noexcept = default;



    /// @brief Destructor.
    ~SuperPixelSegmentation() noexcept = default;



    /// @brief Get the component name.
    std::string getComponentName() const noexcept override;



//-----------------------------------------------------------------------------
// PROTECTED MEMBER FUNCTION
protected:

    /// @brief compute
    void compute() noexcept override;



    /// @brief
    void computeSegmentation() noexcept;



	/// @brief Enforce connectivity for an image.
	///
	/// @param image:
	/// @param centers_size:
	/// @param clusters
	void create_connectivity(cv::Mat& image,
			 	 	 	 	 uint centers_size,
							 const std::vector<std::vector<int>>& clusters) noexcept;

	/// @brief Compute the distance between a center and an individual pixel.
	///
	/// @param ci:
	/// @param nc:
	/// @param ns:
	/// @param pixel:
	/// @param colour:
	/// @param centers:
	///
	/// @return
	double distance(int ci,
					int nc,
					int ns,
					cv::Point pixel,
					cv::Vec3b colour,
					const std::vector<std::vector<double>>& centers) noexcept;



	/// @brief Find the pixel with the lowest gradient in a 3x3 surrounding.
	///
	/// @param image: the image to segment
	/// @param center: the pixel center
	///
	/// @return the local gradient minimum (CvPoint).
	cv::Point find_local_minimum(cv::Mat& image,
								 cv::Point center);



	/// @brief Generate an over-segmentation for an image.
	///
	/// @param image:
	/// @param step:
	/// @param nc:
	void generate_superpixels(cv::Mat& image,
							  int step,
							  int nc,
							  std::vector<std::vector<int>>& clusters,
							  std::vector<std::vector<double>>& distances,
							  std::vector<std::vector<double>>& centers,
							  std::vector<int>& center_counts);



	/// @brief Initialize the 2D vectors.
	///
	/// @param image
	void init_data(uint step,
				   std::vector<std::vector<int>>& clusters,
				   std::vector<std::vector<double>>& distances,
				   std::vector<std::vector<double>>& centers,
				   std::vector<int>& center_counts,
				   cv::Mat& image);



    /// @brief .
    ///
    /// @return true if enough time elapsed to run the phase, false else.
    bool isEnoughTimeElapsed() const noexcept override;



	/// @brief
	void readFileData() noexcept override;



	/// @brief
	///
	/// @param clusters:
	void splitClusterOnColor(std::vector<std::vector<int>>& clusters) noexcept;



	//TODO a supprimer
	void paintSlicSuperPixelContourWithConfidenceValue(uint num,
													   cv::Mat current_image) noexcept;

};
