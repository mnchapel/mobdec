/// @file   reconstruction3dTriangulation.h
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



/// @brief Reconstruction3dTriangulation
class Reconstruction3dTriangulation : public Phase{

//-----------------------------------------------------------------------------
// PROTECTED MEMBER DATA
protected:



//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

    /// @brief Default constructor
	Reconstruction3dTriangulation() = default;



    /// @brief Copy constructor
    Reconstruction3dTriangulation(const Reconstruction3dTriangulation&) = delete;



	/// @brief Move constructor
	Reconstruction3dTriangulation(Reconstruction3dTriangulation&&) = default;



	/// @brief Copy assignment operator
    Reconstruction3dTriangulation& operator=(const Reconstruction3dTriangulation&) = delete;



	/// @brief Move assignment operator
	Reconstruction3dTriangulation& operator=(Reconstruction3dTriangulation&&) = default;



    /// @brief Destructor.
    ~Reconstruction3dTriangulation() = default;



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



    /// @brief
    ///
    /// @param projection_matrix:
    void computeProjectionMatrix(cv::Mat& projection_matrix) noexcept;



    /// @brief
    ///
    /// @param projection_matrix_identity:
    void computeProjectionMatrixIdentity(cv::Mat& projection_matrix_identity) noexcept;



    /// @brief
    ///
    /// @param feature_id:
    /// @param feature_point_t:
    /// @param feature_point_tmdelta:
    void getFeaturePointReconstructionOld(std::vector<uint>& feature_id,
    									  std::vector<cv::Point2d>& feature_point_t,
										  std::vector<cv::Point2d>& feature_point_tmdelta) const noexcept;



    /// @brief .
    ///
    /// @return true if enough time elapsed to run the phase, false else.
    bool isEnoughTimeElapsed() const noexcept override;



    /// @brief
    ///
    /// @param feature_point_t:
    /// @param feature_point_tmdelta:
    /// @param triangulate_point:
    void triangulate(const std::vector<cv::Point2d>& feature_point_t,
					 const std::vector<cv::Point2d>& feature_point_tmdelta,
					 cv::Mat& triangulate_point) noexcept;



    /// @brief
    ///
    /// @param feature_id:
    /// @param triangulate_point:
    void updateData(const std::vector<uint>& feature_id,
    				const cv::Mat& triangulate_point) noexcept;

};
