/// @file   cameraCompensation.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// C++
#include <ctime>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdlib.h>
#include <vector>

// MoBDec
#include <core/dataWriter.h>
#include <core/phase.h>



/// @brief The LabelingProcess class
class CameraCompensation : public Phase {

//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

    /// @brief Default constructor
	CameraCompensation() noexcept = default;



	/// @brief Copy constructor
    CameraCompensation(const CameraCompensation&) noexcept = delete;



	/// @brief Move constructor
	CameraCompensation(CameraCompensation&&) noexcept = default;



	/// @brief Copy assignment operator
    CameraCompensation& operator=(const CameraCompensation&) noexcept = delete;



	/// @brief Move assignment operator
	CameraCompensation& operator=(CameraCompensation&&) noexcept = default;



    /// @brief Destructor.
    ~CameraCompensation() noexcept = default;



    /// @brief Get the component name.
    std::string getComponentName() const noexcept override;



//-----------------------------------------------------------------------------
// PROTECTED MEMBER FUNCTION
protected:

    /// @brief compute
    void compute() noexcept override;


};
