/// @file   kmeansOpticalFlow.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// C++
#include <ctime>

// MoBDec
#include <core/dataWriter.h>
#include <core/phase.h>



/// @brief The LabelingProcess class
class KMeansOpticalFlow : public Phase{

//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

    /// @brief Default constructor
	KMeansOpticalFlow() noexcept = default;



    /// @brief Default constructor
	KMeansOpticalFlow(bool read_phase) noexcept
	{
		this->read_phase = true;
	}



	/// @brief Copy constructor
    KMeansOpticalFlow(const KMeansOpticalFlow&) noexcept = delete;



	/// @brief Move constructor
	KMeansOpticalFlow(KMeansOpticalFlow&&) noexcept = default;



	/// @brief Copy assignment operator
    KMeansOpticalFlow& operator=(const KMeansOpticalFlow&) noexcept = delete;



	/// @brief Move assignment operator
	KMeansOpticalFlow& operator=(KMeansOpticalFlow&&) noexcept = default;



    /// @brief Destructor.
    ~KMeansOpticalFlow() noexcept = default;



    /// @brief Get the component name.
    std::string getComponentName() const noexcept override;



    /// @brief
    void readFileData() noexcept override;



//-----------------------------------------------------------------------------
// PROTECTED MEMBER FUNCTION
protected:

    /// @brief compute
    void compute() noexcept override;



    /// @brief
    ///
    /// @param
    void getStaticFeaturePointOld(cv::Mat& feature_point) const noexcept;
    
    
    
    /// @brief .
    ///
    /// @return true if enough time elapsed to run the phase, false else.
    bool isEnoughTimeElapsed() const noexcept override;



    /// @brief
    void selectPointForEssentialMatrixNister() noexcept;


};





















