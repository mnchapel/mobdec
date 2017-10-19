/// @file   initialiseProject.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <core/initialiseProject.h>



//---------------------------------------------------------------------------------------
void InitialiseProject::copyConfigurationFileInResultFolder(std::string configuration_file_path) noexcept
{
	std::string mobdec_data_path = CST(std::string,cst::MOBDEC_DATA_PATH);
	std::string project_name = CST(std::string,cst::PROJECT_NAME);

	int dummy;
	std::string command_copy;
	std::string result_folder_path = mobdec_data_path + "/" + project_name + "/";

	command_copy = "cp " + configuration_file_path + " " + result_folder_path;
	dummy = system(command_copy.c_str());
}



//---------------------------------------------------------------------------------------
void InitialiseProject::createResultFolder() noexcept
{
	std::vector<std::vector<std::string>> cycles = Settings::get().getCycles();

    // Create result directory for each component
    const uint nb_cycle = cycles.size();
    for(int i=0; i<nb_cycle; i++)
    {
    	const uint nb_module = cycles[i].size();
    	for(int j=1; j<nb_module; j++) // The first element is end_time_cycle
    	{
			// Create result directory
			std::string command_create_directory;
			int dummy;

			std::string module_name = cycles[i][j];
			std::string component_image_directory = CST(std::string,cst::MOBDEC_DATA_PATH)
												  + "/" + CST(std::string,cst::PROJECT_NAME)
												  + "/" + module_name + "/";

			command_create_directory = "mkdir -p " + component_image_directory + " > nul 2>&1";
			dummy = system(command_create_directory.c_str());
    	}
    }
}



//---------------------------------------------------------------------------------------
void InitialiseProject::initialise(std::string configuration_file_path,
		   	   	   	   	   	   	   Data& data,
								   std::vector<LifeCycleManager>& cycle) noexcept
{
	Settings::get().parse(configuration_file_path);

    createResultFolder();
    copyConfigurationFileInResultFolder(configuration_file_path);

    initialiseData(data);
    initialiseCycle(cycle);
}



//---------------------------------------------------------------------------------------
void InitialiseProject::initialiseCycle(std::vector<LifeCycleManager>& cycle) noexcept
{
	std::vector<std::vector<std::string>> cycles = Settings::get().getCycles();

    uint nb_cycle = cycles.size();
    cycle.resize(nb_cycle);
    for(int i=0; i<nb_cycle; i++)
    {
    	int end_time_cycle = std::stoi(cycles[i][0]);
    	std::vector<std::string> cycle_step_names(cycles[i].begin()+1, cycles[i].end());
    	cycle[i] = LifeCycleManager(end_time_cycle, cycles[i]);
    }
}



//---------------------------------------------------------------------------------------
void InitialiseProject::initialiseData(Data& data) noexcept
{
    Video video;

    cv::Mat intrinsic_parameters = Settings::get().getIntrinsicParameters();
    Camera camera(intrinsic_parameters);

    data = Data(video,
				camera);
}
