/// @file   initialiseProject.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// C++
#include <vector>

// MoBDec
#include <core/data.h>
#include <core/lifeCycleManager.h>



/// @brief
class InitialiseProject{

//-----------------------------------------------------------------------------
// PRIVATE MEMBER DATA
private:



//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

	/// @brief Destructor
	~InitialiseProject() = default;



	/// @brief
	///
	/// @param configuration_file_path: absolute path to the configuration file.
	static void initialise(std::string configuration_file_path,
			   	   	   	   Data& data,
						   std::vector<LifeCycleManager>& cycle) noexcept;


//---------------------------------------------------------------------------------------
// PRIVATE MEMBER FUNCTION
private:

    /// @brief Default constructor
	InitialiseProject() = default;



	/// @brief Copy constructor
	InitialiseProject(const InitialiseProject&) = default;



	/// @brief Move constructor
	InitialiseProject(InitialiseProject&&) = default;



	/// @brief Copy assignment operator
	InitialiseProject& operator=(const InitialiseProject&) = default;



	/// @brief Move assignment operator
	InitialiseProject& operator=(InitialiseProject&&) = default;



	/// @brief
	///
	/// @param configuration_file_path
	static void copyConfigurationFileInResultFolder(std::string configuration_file_path) noexcept;



	/// @brief
	static void createResultFolder() noexcept;



	/// @brief
	///
	/// @param cycle
	void initialiseACycle(std::vector<LifeCycleManager>& cycle) noexcept;



	/// @brief
	///
	/// @param
	static void initialiseCycle(std::vector<LifeCycleManager>& cycle) noexcept;



	/// @brief
	///
	/// @param
	static void initialiseData(Data& data) noexcept;

};

