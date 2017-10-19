/// @file   settings.h
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#pragma once



// C++
#include <array>
#include <type_traits>

// Mobdec
#include <core/mVariant.h>
#include <core/debug.h>

// OpenCV
#include <opencv2/opencv.hpp>



//namespace mobdec{




/// Setting keys
enum class cst : int{

	// INPUT DATA
	PROJECT_NAME,					// Name of the project
	MOBDEC_DATA_PATH,				// Path to the result file
	TEMPLATE_IMAGE_PATH,			// Path to the images of the video sequence
	TEMPLATE_GT_IMAGE_PATH,			// Path to the ground truth images of the video sequence

	// GENERAL
	DELTA,							// Size between two frames for computations
	NB_ROWS,						// Size of feature point history (DELTA+1)
	NB_MAX_POINT,					// Max number of feature points
	START_TIME,						// First frame
	END_TIME,						// Last frame

	// LDOF
	LDOF_SUBSAMPLING_FACTOR,		// Subsampling factor

	// LABELING
	LABEL_ON_CV_THRESHOLD,			// Label on confidence value (cv) threshold: cv>LABEL_ON_CV_THRESHOLD: STATIC | -LABEL_ON_CV_THRESHOLD<cv<LABEL_ON_CV_THRESHOLD: UNCERTAIN | -LABEL_ON_CV_THRESHOLD<cv: MOVING
	NB_FRAME_ACCUMULATION,			// (3)
	PERCENT_PC_TO_BE_STATIC,		// (0.4)
	MAX_PTP_ERROR_STATIC,			// (0.10)
	MAX_PTP_ERROR_MOVING,			// (0.5-MAX_PTP_ERROR_STATIC)

	// SCALE
	SCALE_MAX_DISTANCE,				// (10)
	SCALE_NB_SAMPLE_POINT,			// (500)

	// SEGMENTATION
	NB_ITERATIONS,					// Number of iterations run by the clustering algorithm (10)
	SLIC_WEIGHT,					// (20)
	NB_SUPER_PIXEL,					// (500)

	NB_SETTINGS // leave at the end of enumeration
};


class Settings {

//---------------------------------------------------------------------------------------
// PUBLIC MEMBER DATA
public:

//---------------------------------------------------------------------------------------
// PRIVATE MEMBER DATA
private:

	///
//	std::array<cv::FileNode, static_cast<int>(cst::NB_SETTINGS)> value;

	///
	std::array<MVariant, static_cast<int>(cst::NB_SETTINGS)> value;

	///
	std::vector<std::vector<std::string>> cycle;

	///
	cv::Mat intrinsic_parameters;


//-----------------------------------------------------------------------------
// PUBLIC MEMBER FUNCTION
public:

	/// @brief Destructor
	~Settings() = default;



	/// @brief
	///
	/// @return
	static Settings& get()
	{
		static Settings instance;
		return instance;
	}



	/// @brief Access to cycles
	///
	/// @return cycles.
	std::vector<std::vector<std::string>> getCycles() noexcept;



	/// @brief Access to intrinsic parameters
	///
	/// @return intrinsic parameters of the camera.
	cv::Mat getIntrinsicParameters() noexcept;



	/// @brief operator()
	///
	/// @param key
	///
	/// @return
	template<typename _T>
	_T operator()(cst key) const
	{
		// Specific key
		if(key == cst::NB_ROWS)
			return static_cast<int>(value[cst2int(cst::DELTA)])+1; // DELTA+1

		MVariant mv = value[cst2int(key)];

		if(mv.isInt())
			return static_cast<int>(mv);
		else if(mv.isDouble())
			return static_cast<double>(mv);

		// TODO: add exception

		return 0;
	}



	/// @brief
	///
	/// @param settings_file_path: absolute path to the settings file.
	void parse(std::string settings_file_path) noexcept;



	/// @brief
	///
	/// @param file_path: absolute path to the intrinsic parameters file YAML.
	void parseIntrinsicParameters(std::string file_path) noexcept;



//---------------------------------------------------------------------------------------
// PRIVATE MEMBER FUNCTION
private:

    /// @brief Default constructor
	Settings() = default;



	/// @brief Copy constructor
	Settings(const Settings&) = default;



	/// @brief Move constructor
	Settings(Settings&&) = default;



	/// @brief Copy assignment operator
	Settings& operator=(const Settings&) = default;



	/// @brief Move assignment operator
	Settings& operator=(Settings&&) = default;



	// @brief Check that settings for the components are supplied in the configuration file
	void checkSettings() const noexcept;



	/// @brief
	///
	/// @param
	///
	/// @return
	MVariant convFileNodeToMVariant(const cv::FileNode& file_node) const noexcept;



	/// @brief
	///
	/// @param it
	void readACycle(cv::FileNodeIterator it) noexcept;



	/// @brief
	///
	/// @param it
	/// @param a_cycle
	void readACycleModuleList(cv::FileNodeIterator it,
							  std::vector<std::string>& a_cycle) noexcept;


	/// @brief
	///
	/// @param
	template<typename E>
	constexpr auto cst2int(E e) const -> typename std::underlying_type<E>::type
	{
	   return static_cast<typename std::underlying_type<E>::type>(e);
	}

};




template<>
inline std::string Settings::operator()<std::string>(cst key) const
{
	MVariant mv = value[cst2int(key)];
    return (std::string)mv;
}



// Variable name to string
#define GET_VARIABLE_NAME(Variable) (#Variable)

//
#define CST(type, key) Settings::get().operator()<type>(key)



//}
