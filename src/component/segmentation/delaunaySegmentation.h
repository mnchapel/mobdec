/// @file   delaunaySegmentation.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// MoBDec
#include <core/phase.h>




/// @brief The Delaunay Segmentation class
class DelaunaySegmentation : public Phase{

//---------------------------------------------------------------------------------------
// PRIVATE MEMBER DATA
private:



//---------------------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

	/// @brief Default constructor
	DelaunaySegmentation() = default;



	/// @brief Copy constructor
    DelaunaySegmentation(const DelaunaySegmentation&) = delete;



	/// @brief Move constructor
	DelaunaySegmentation(DelaunaySegmentation&&) = default;



	/// @brief Copy assignment operator
    DelaunaySegmentation& operator=(const DelaunaySegmentation&) = delete;



	/// @brief Move assignment operator
	DelaunaySegmentation& operator=(DelaunaySegmentation&&) = default;



	/// @brief Destructor
	~DelaunaySegmentation() = default;



    /// @brief Get the component name.
    ///
    /// @return
    virtual std::string getComponentName() const noexcept override;



    /// @brief
	void readFileData() noexcept;
	



//---------------------------------------------------------------------------------------
// PRIVATE MEMBER FUNCTION
private:

    /// @brief Compute the phase.
    virtual void compute() noexcept;

};
