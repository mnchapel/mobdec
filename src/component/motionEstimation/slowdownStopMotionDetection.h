/// @file slowdownStopMotionDetection.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// C++
#include <ctime>

// MoBDec
#include <core/dataReader.h>
#include <core/dataWriter.h>
#include <core/phase.h>



/// @brief The LabelingProcess class
class SlowdownStopMotionDetection : public Phase{

//-----------------------------------------------------------------------------
// PROTECTED MEMBER DATA
protected:

	//!
	uint n=0;

	//!
	uint t=0;


//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

    /// @brief Default constructor
	SlowdownStopMotionDetection() noexcept = default;



    /// @brief Default constructor
	SlowdownStopMotionDetection(bool read_phase) noexcept
	{
		this->read_phase = true;
	}



	/// @brief Copy constructor
    SlowdownStopMotionDetection(const SlowdownStopMotionDetection&) noexcept = delete;



	/// @brief Move constructor
	SlowdownStopMotionDetection(SlowdownStopMotionDetection&&) noexcept = default;



	/// @brief Copy assignment operator
    SlowdownStopMotionDetection& operator=(const SlowdownStopMotionDetection&) noexcept = delete;



	/// @brief Move assignment operator
	SlowdownStopMotionDetection& operator=(SlowdownStopMotionDetection&&) noexcept = default;



    /// @brief Destructor.
    ~SlowdownStopMotionDetection() noexcept = default;



    /// @brief Get the component name.
    ///
    /// @return The component name (the class name).
    std::string getComponentName() const noexcept override;



    /// @brief
    ///
    /// @return
    std::string getPhaseReturn() const noexcept override;



//-----------------------------------------------------------------------------
// PROTECTED MEMBER FUNCTION
protected:

    /// @brief compute
    void compute() noexcept override;



    /// @brief
    void correctSlowDownStopMotion() noexcept;



    /// @brief .
    ///
    /// @return true if enough time elapsed to run the phase, false else.
    bool isEnoughTimeElapsed() const noexcept override;



    /// @brief
    ///
    /// @return
    bool isOpticalFlowGood() noexcept;



    /// @brief
    ///
    /// @param position_2d
    ///
    /// @return
    bool isOpticalFlowGood(const std::vector<cv::Mat>& position_2d) noexcept;



    /// @brief
    void readFileData() noexcept override;



    /// @brief
    void saveFeaturePoint() noexcept;



    /// @brief
    void selectFeaturePointHistoric() noexcept;

};





















