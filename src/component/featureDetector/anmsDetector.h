/// @file   anms.h
/// @author Marie-Neige Chapel
/// @date   2018/05/03



#pragma once



// C++
#include <numeric>

// MoBDec
#include <component/featureDetector/anms/nanoflann.h>
#include <component/featureDetector/anms/PointCloud.h>
#include <component/featureDetector/anms/ranget.h>
#include <core/dataWriter.h>
#include <core/phase.h>



class AnmsDetector : public Phase {

//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

    /// @brief Default contructor
    AnmsDetector() = default;



    /// @brief Copy constructor
    AnmsDetector(const AnmsDetector&) = delete;



    /// @brief Move constructor
    AnmsDetector(AnmsDetector&&) = default;



    /// @brief Copy assignment operator
    AnmsDetector& operator=(const AnmsDetector&) = delete;



    /// @brief Move assignment operator
    AnmsDetector& operator=(AnmsDetector&&) = default;



    /// @brief Destructor.
    ~AnmsDetector() = default;



    /// @brief getComponentName
    ///
    /// @return
    std::string getComponentName() const noexcept override;



//-----------------------------------------------------------------------------
// PROTECTED MEMBER FUNCTION
protected:

    /// @brief
    ///
    /// @param key_points: the key points.
    /// @param nb_points: the exact number of points to return.
    ///
    /// @return
    std::vector<cv::KeyPoint> brownANMS(std::vector<cv::KeyPoint> key_points, int nb_points);



    /// @brief Compute
    void compute() noexcept override;



    /// @brief .
    ///
    /// @return true if enough time elapsed to run the phase, false else.
    bool isEnoughTimeElapsed() const noexcept override;



    /// @brief
    ///
    /// @param key_points:
    /// @param nb_points:
    /// @param tolerance:
    /// @param cols:
    /// @param rows:
    ///
    /// @return
    std::vector<cv::KeyPoint> kdTree(std::vector<cv::KeyPoint> key_points, int nb_points, float tolerance, int cols, int rows);



    /// @brief
    void readFileData() noexcept override;



    /// @brief
    ///
    /// @param key_points: the key points detected.
    /// @param nb_points: the exact number of points to return.
    /// @param tolerance:
    /// @param cols:
    /// @param rows:
    ///
    /// @return
    std::vector<cv::KeyPoint> RangeTree(std::vector<cv::KeyPoint> key_points, int nb_points,float tolerance, int cols, int rows) noexcept;



    /// @brief
    ///
    /// @param key_points: the key points.
    /// @param nb_points: the exact number of points to return.
    /// @param tolerance:
    /// @param cols:
    /// @param rows:
    ///
    /// @return
    std::vector<cv::KeyPoint> sdc(std::vector<cv::KeyPoint> key_points, int nb_points, float tolerance, int cols, int rows) noexcept;



    /// @brief Sort key points by deacreasing order of strength.
    ///
    /// @param key_points: the key points to sort.
    void sortKeyPointsByResponse(std::vector<cv::KeyPoint>& key_points) noexcept;



    /// @brief
    ///
    /// @param key_points: the key points.
    /// @param nb_points: the exact number of points to return.
    /// @param tolerance:
    /// @param cols:
    /// @param rows:
    ///
    /// @return
    std::vector<cv::KeyPoint> ssc(std::vector<cv::KeyPoint> key_points, int nb_points,float tolerance, int cols, int rows) noexcept;



    /// @brief
    ///
    /// @param key_points: the key points.
    /// @param nb_points: the exact number of points to return.
    ///
    /// @return the first nb_points in key_points.
    std::vector<cv::KeyPoint> topN(std::vector<cv::KeyPoint> key_points, int nb_points) noexcept;
};
