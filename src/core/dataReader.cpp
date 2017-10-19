/// @file   dataReader.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <core/dataReader.h>



//---------------------------------------------------------------------------------------
void DataReader::readConfidenceValue(Data* data,
		 	 	 	   	  	  	  	 std::string component_name) noexcept
{
    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_DATA_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/confidence_value_%03d.txt";
    sprintf(buffer, path.c_str(), data->getTime());

    std::ifstream file;
    file.open(buffer);

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        double confidence_value;

        file >> confidence_value;

        if(!data->isFeaturePoint(i))
			continue;

        double bonus_malus = confidence_value - data->getFeaturePointConfidenceValue(i);
        data->updateConfidenceValue(i, bonus_malus);
    }

    file.close();
}



//---------------------------------------------------------------------------------------
void DataReader::readDistanceToCameraAfterScale(uint time,
												std::vector<double>& distance_to_camera,
												const Data* data,
												std::string component_name) noexcept
{
    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_DATA_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/distance_to_camera_after_scale_%03d.txt";
    sprintf(buffer, path.c_str(), time);

    std::ifstream file;
    file.open(buffer);

    double distance;

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        file >> distance;
        distance_to_camera.push_back(distance);
    }

    file.close();
}



//---------------------------------------------------------------------------------------
void DataReader::readFeaturePoint(Data* data,
		 	 	 	 	 	 	  std::string component_name) noexcept
{
    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_DATA_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/feature_point_%03d.txt";
    sprintf(buffer, path.c_str(), data->getTime());

    std::ifstream file;
    file.open(buffer);

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        FeaturePoint feature_point;
        cv::Mat position_2d(2,1,CV_64F);
        uint age;

        file >> position_2d.at<double>(0)
             >> position_2d.at<double>(1)
             >> age;

        if(age == 0
        || age == 1)
        	data->deleteFeaturePoint(i);

        feature_point.update2DPosition(position_2d);
        feature_point.updateAge();
    }

    file.close();
}



//---------------------------------------------------------------------------------------
void DataReader::readFeaturePoint(uint time,
								  std::vector<cv::Mat>& position_2d_list,
								  std::vector<uint>& age_list,
								  const Data* data,
		 	 	 	 	 	 	  std::string component_name) noexcept
{
    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_DATA_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/feature_point_%03d.txt";
    sprintf(buffer, path.c_str(), time);

    std::ifstream file;
    file.open(buffer);

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        FeaturePoint feature_point;
        cv::Mat position_2d(2,1,CV_64F);
        uint age;

        file >> position_2d.at<double>(0)
             >> position_2d.at<double>(1)
             >> age;

        position_2d_list.push_back(position_2d);
        age_list.push_back(age);
    }

    file.close();
}







