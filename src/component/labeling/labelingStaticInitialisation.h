/// @file   labelingStaticInitialisation.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// MoBDec
#include <core/dataWriter.h>
#include <core/phase.h>



/// @brief LabelingStaticInitialisation
class LabelingStaticInitialisation : public Phase{

//-----------------------------------------------------------------------------
// PROTECTED MEMBER DATA
protected:



//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

    /// @brief Default constructor
	LabelingStaticInitialisation() = default;



    /// @brief Constructor
    LabelingStaticInitialisation(const LabelingStaticInitialisation&) = delete;



	/// @brief Move constructor
	LabelingStaticInitialisation(LabelingStaticInitialisation&&) = default;



	/// @brief Copy assignment operator
    LabelingStaticInitialisation& operator=(const LabelingStaticInitialisation&) = delete;



	/// @brief Move assignment operator
	LabelingStaticInitialisation& operator=(LabelingStaticInitialisation&&) = default;



    /// @brief Destructor.
    ~LabelingStaticInitialisation() = default;



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



    /// @brief .
    ///
    /// @return true if enough time elapsed to run the phase, false else.
    bool isEnoughTimeElapsed() const noexcept override;

};
