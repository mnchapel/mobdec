/// @file   phase.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// MoBDec
#include <core/data.h>
#include <core/debug.h>



/// @brief
class Phase{

//---------------------------------------------------------------------------------------
// PROTECTED MEMBER DATA
protected:

    //!
    Data* data = NULL;

    //!
    bool read_phase = false;

    //!
    bool break_life_cycle = false;



//---------------------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

    /// @brief Default constructor.
    Phase() = default;



//    /// @brief Constructor.
//    ///
//    /// @param read_phase:
//    Phase(bool read_phase)
//    	: read_phase(read_phase)
//    {}



    /// @brief Copy constructor.
    Phase(const Phase&) = delete;



    /// @brief Move constructor.
    Phase(Phase&&) = default;



	/// @brief Copy assignment operator
    Phase& operator=(const Phase&) = delete;



	/// @brief Move assignment operator
    Phase& operator=(Phase&&) = default;



    /// @brief Destructor.
    virtual ~Phase() = default;



    /// @brief Get the component name.
    ///
    /// @return
    virtual std::string getComponentName() const noexcept = 0;



    /// @brief Destructor.
    virtual std::string getPhaseReturn() const noexcept
    {
    	return "";
    }



    /// @brief Launch the phase.
    ///
    /// @param data
    bool launch(Data* data)
    {
#ifndef NDEBUG
    	std::clock_t start;
    	start = std::clock();
#endif

        this->data = data;

        if(isEnoughTimeElapsed())
            DEBUG_MSG("Launch " << getComponentName() << " ---------------------");
        else
        {
            DEBUG_MSG("Not enough time elapsed to compute " << getComponentName());
            return break_life_cycle;
        }

        if(read_phase)
        	readFileData();
        else
        	compute();

#ifndef NDEBUG
        DEBUG_MSG("Time " << getComponentName() << ": " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl);
#endif

        return break_life_cycle;
    }



    /// @brief Get the component name.
    ///
    /// @return
    virtual void readFileData() noexcept = 0;
    
    
    
//---------------------------------------------------------------------------------------
// PROTECTED MEMBER FUNCTION
protected:

    /// @brief Compute the phase.
    virtual void compute() noexcept = 0;
    
    
    
    /// @brief .
    ///
    /// @return true if enough time elapsed to run the phase, false else.
    virtual bool isEnoughTimeElapsed() const noexcept = 0;
};
