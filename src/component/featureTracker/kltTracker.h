/// @file   kltTracker.h
/// @author Marie-Neige Chapel
/// @date   2018/05/03



#pragma once



// MoBDec
#include <core/phase.h>



class KltTracker : public Phase {

//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

    /// @brief Default contructor
    KltTracker() = default;



    /// @brief Copy constructor
    KltTracker(const KltTracker&) = delete;



    /// @brief Move constructor
    KltTracker(KltTracker&&) = default;



    /// @brief Copy assignment operator
    KltTracker& operator=(const KltTracker&) = delete;



    /// @brief Move assignment operator
    KltTracker& operator=(KltTracker&&) = default;



    /// @brief Destructor.
    ~KltTracker() = default;



    /// @brief getComponentName
    ///
    /// @return
    std::string getComponentName() const noexcept override;



//-----------------------------------------------------------------------------
// PROTECTED MEMBER FUNCTION
protected:

    /// @brief
    ///
    /// @param dp_idx:
    /// @param fp_t:
    /// @param status:
    void updateFeaturePointPositions(const std::vector<uint>& fp_idx,
                      const std::vector<cv::Point2f> &fp_t,
                      const std::vector<uchar>& status) noexcept;



    /// @brief Compute
    void compute() noexcept override;



    /// @brief
    ///
    /// @param fp_idx:
    /// @param fp_tm1: feature points in the previous frame.
    void getFeaturePointAtTM1(std::vector<uint> &fp_idx,
                              std::vector<cv::Point2f>& fp_tm1) const noexcept;



    /// @brief .
    ///
    /// @return true if enough time elapsed to run the phase, false else.
    bool isEnoughTimeElapsed() const noexcept override;



    /// @brief
    void readFileData() noexcept override;
};
