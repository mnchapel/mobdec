/// @file   mobdec.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



// C++
#include <iostream>

// MoBDec
#include <core/initialiseProject.h>
#include <core/data.h>



//---------------------------------------------------------------------------------------
int main(int argc, char** argv)
{

	if(argc<2)
	{
        std::cout << argv[0] << " <path_to_configuration_file>" << std::endl;
		exit(0);
	}

	Data data;
	std::vector<LifeCycleManager> cycle;

	InitialiseProject::initialise(argv[1], data, cycle);

	for(int i=0; i<cycle.size(); i++)
        cycle[i].launch(&data);

	std::cout << "finished" << std::endl;

	return 0;
}
