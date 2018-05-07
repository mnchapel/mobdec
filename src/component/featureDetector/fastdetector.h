/// @file   fastDetector.h
/// @author Marie-Neige Chapel
/// @date   2018/05/03



#pragma once



// MoBDec
#include <core/dataWriter.h>
#include <core/phase.h>



class FastDetector : public Phase {

//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

    /// @brief Default contructor
    FastDetector() = default;



    /// @brief Copy constructor
    FastDetector(const FastDetector&) = delete;



    /// @brief Move constructor
    FastDetector(FastDetector&&) = default;



    /// @brief Copy assignment operator
    FastDetector& operator=(const FastDetector&) = delete;



    /// @brief Move assignment operator
    FastDetector& operator=(FastDetector&&) = default;



    /// @brief Destructor.
    ~FastDetector() = default;



    /// @brief getComponentName
    ///
    /// @return
    std::string getComponentName() const noexcept override;



//-----------------------------------------------------------------------------
// PROTECTED MEMBER FUNCTION
protected:

    /// @brief Compute
    void compute() noexcept override;



    /// @brief .
    ///
    /// @return true if enough time elapsed to run the phase, false else.
    bool isEnoughTimeElapsed() const noexcept override;



    /// @brief
    void readFileData() noexcept override;
};
