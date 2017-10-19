/// @file   scaleInBox.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// C++
#include <chrono>
#include <ctime>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdlib.h>
#include <vector>

// MoBDec
#include <core/dataWriter.h>
#include <core/phase.h>



/// @brief ScaleInBox
class ScaleInBox : public Phase{

//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

    /// @brief Default constructor
    ScaleInBox() noexcept = default;



    /// @brief Copy constructor
    ScaleInBox(const ScaleInBox&) noexcept = delete;



    /// @brief Move constructor
    ScaleInBox(ScaleInBox&&) noexcept = default;



    /// @brief Copy assignment operator
    ScaleInBox& operator=(const ScaleInBox&) noexcept = delete;



    /// @brief Move assignment operator
    ScaleInBox& operator=(ScaleInBox&&) noexcept = default;



    /// @brief Destructor.
    ~ScaleInBox() noexcept = default;



    /// @brief Get the component name.
    std::string getComponentName() const noexcept override;



//-----------------------------------------------------------------------------
// PROTECTED MEMBER FUNCTION
protected:

    /// @brief compute
    void compute() noexcept override;



    /// @brief
    void computeDistanceInUnitBox() noexcept;



    /// @brief .
    ///
    /// @return true if enough time elapsed to run the phase, false else.
    bool isEnoughTimeElapsed() const noexcept override;



    /// @brief
    void readFileData() noexcept override;

};
