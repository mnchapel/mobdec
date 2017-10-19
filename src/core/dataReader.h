/// @file   dataReader.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// C++
#include <fstream>
#include <vector>

// MoBDec
#include <core/data.h>



/// @brief
class DataReader{

//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

	/// @brief Destructor
	~DataReader() = default;



	/// @brief
	///
	/// @param data:
	/// @param component_name:
	static void readConfidenceValue(Data* data,
		 	 	 	 	  	  	  	std::string component_name) noexcept;



	/// @Brief
	///
	/// @param time
	/// @param distance_to_camera
	/// @param data
	/// @param component_name
	static void readDistanceToCameraAfterScale(uint time,
											   std::vector<double>& distance_to_camera,
											   const Data* data,
											   std::string component_name) noexcept;



	/// @brief
	///
	/// @param data:
	/// @param component_name:
	static void readFeaturePoint(Data* data,
            					 std::string component_name) noexcept;



	/// @brief
	///
	/// @param time
	/// @param position_2d
	/// @param age
	/// @param data
	/// @param component_name
	static void readFeaturePoint(uint time,
								 std::vector<cv::Mat>& position_2d,
								 std::vector<uint>& age,
								 const Data* data,
								 std::string component_name) noexcept;



//---------------------------------------------------------------------------------------
// PRIVATE MEMBER FUNCTION
private:

    /// @brief Default constructor
    DataReader() = default;



	/// @brief Copy constructor
    DataReader(const DataReader&) = default;



	/// @brief Move constructor
    DataReader(DataReader&&) = default;



	/// @brief Copy assignment operator
    DataReader& operator=(const DataReader&) = default;



	/// @brief Move assignment operator
    DataReader& operator=(DataReader&&) = default;

};
