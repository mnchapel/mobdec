/// @file   settings.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <core/settings.h>



//---------------------------------------------------------------------------------------
void Settings::checkSettings() const noexcept
{
	uint nb_cycle = cycle.size();
	for(int i=0; i<nb_cycle; i++)
	{
		uint nb_component = cycle[i].size();
		for(int j=0; j<nb_component; j++)
		{

		}
	}
}



//---------------------------------------------------------------------------------------
MVariant Settings::convFileNodeToMVariant(const cv::FileNode& file_node) const noexcept
{
	MVariant mv;

	if(file_node.isInt())
		mv = MVariant((int)file_node);
	else if(file_node.isReal())
		mv = MVariant((double)file_node);
	else if(file_node.isString())
		mv = MVariant((std::string)file_node);

	return mv;
}



//---------------------------------------------------------------------------------------
std::vector<std::vector<std::string>> Settings::getCycles() noexcept
{
	return cycle;
}



//---------------------------------------------------------------------------------------
cv::Mat Settings::getIntrinsicParameters() noexcept
{
	return intrinsic_parameters;
}



//---------------------------------------------------------------------------------------
void Settings::parse(std::string settings_file_path) noexcept
{
	cv::FileStorage fs(settings_file_path, cv::FileStorage::READ);

	if(!fs.isOpened())
	{
		std::cerr << "Error : cannot open the configuration file " << settings_file_path << std::endl;
        exit(EXIT_FAILURE);
	}

//	std::cout << GET_VARIABLE_NAME(a) << std::endl;

//	for(int i=0; i<cst::NB_SETTINGS; i++)
//		value[i] = fs[];



	// GENERAL
	value[cst2int(cst::PROJECT_NAME)]			= convFileNodeToMVariant(fs["project"]);
	value[cst2int(cst::MOBDEC_DATA_PATH)]		= convFileNodeToMVariant(fs["mobdec_data_path"]);
	value[cst2int(cst::TEMPLATE_IMAGE_PATH)]	= convFileNodeToMVariant(fs["template_image_path"]);
	value[cst2int(cst::TEMPLATE_GT_IMAGE_PATH)]	= convFileNodeToMVariant(fs["template_gt_image_path"]);

	value[cst2int(cst::DELTA)]			= convFileNodeToMVariant(fs["delta"]);
	value[cst2int(cst::NB_MAX_POINT)]	= convFileNodeToMVariant(fs["nb_max_points"]);
	value[cst2int(cst::START_TIME)]		= convFileNodeToMVariant(fs["start_time"]);
	value[cst2int(cst::END_TIME)]		= convFileNodeToMVariant(fs["end_time"]);

	// LDOF
	value[cst2int(cst::LDOF_SUBSAMPLING_FACTOR)] = convFileNodeToMVariant(fs["ldof_subsampling_factor"]);

	// LABELING
	value[cst2int(cst::LABEL_ON_CV_THRESHOLD)]	= convFileNodeToMVariant(fs["label_cv_on_threshold"]);
	value[cst2int(cst::NB_FRAME_ACCUMULATION)]	= convFileNodeToMVariant(fs["nb_frame_accumulation"]);
	value[cst2int(cst::PERCENT_PC_TO_BE_STATIC)]= convFileNodeToMVariant(fs["percent_pc_to_be_static"]);
	value[cst2int(cst::MAX_PTP_ERROR_STATIC)]	= convFileNodeToMVariant(fs["max_ptp_error_static"]);
	value[cst2int(cst::MAX_PTP_ERROR_MOVING)]	= convFileNodeToMVariant(fs["max_ptp_error_moving"]);

	// SCALE
	value[cst2int(cst::SCALE_MAX_DISTANCE)]		= convFileNodeToMVariant(fs["scale_max_distance"]);
	value[cst2int(cst::SCALE_NB_SAMPLE_POINT)]	= convFileNodeToMVariant(fs["scale_nb_sample_point"]);

	// SEGMENTATION
	value[cst2int(cst::NB_ITERATIONS)]	= convFileNodeToMVariant(fs["nb_iterations"]);
	value[cst2int(cst::SLIC_WEIGHT)]	= convFileNodeToMVariant(fs["slic_weight"]);
	value[cst2int(cst::NB_SUPER_PIXEL)]	= convFileNodeToMVariant(fs["nb_super_pixel"]);

	// INTRINSIC_PARAMETERS
	parseIntrinsicParameters((std::string)fs["intrinsic_parameters"]);

	// CYCLES
	cv::FileNode cycle = fs["cycle"];
	cv::FileNodeIterator it		= cycle.begin();
	cv::FileNodeIterator it_end	= cycle.end();

	for(; it != it_end; ++it)
		readACycle(it);

	fs.release();
}



//---------------------------------------------------------------------------------------
void Settings::parseIntrinsicParameters(std::string file_path) noexcept
{
	cv::FileStorage fs(file_path, cv::FileStorage::READ);

	if(!fs.isOpened())
	{
		std::cerr << "Error : cannot open the configuration file " << file_path << std::endl;
		exit(EXIT_FAILURE);
	}

	fs["intrinsicParameters"] >> intrinsic_parameters;

	fs.release();
}



//---------------------------------------------------------------------------------------
void Settings::readACycle(cv::FileNodeIterator it) noexcept
{
	std::vector<std::string> a_cycle;

	a_cycle.push_back(std::to_string((int)(*it)["end"]));

	readACycleModuleList(it, a_cycle);

	cycle.push_back(a_cycle);
}



//---------------------------------------------------------------------------------------
void Settings::readACycleModuleList(cv::FileNodeIterator it,
									std::vector<std::string>& a_cycle) noexcept
{
	cv::FileNode cycle_child = (*it)["module"];

	cv::FileNodeIterator it_begin	= cycle_child.begin();
	cv::FileNodeIterator it_end		= cycle_child.end();

	std::vector<std::string> module_list;

	for(; it_begin != it_end; ++it_begin)
		a_cycle.push_back((std::string)*it_begin);
}
