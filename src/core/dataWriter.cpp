/// @file   dataWriter.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <core/dataWriter.h>



//---------------------------------------------------------------------------------------
// buildColorCode code from : eccv
cv::Scalar DataWriter::ageColor(uint age) noexcept
{
	cv::Scalar color;

	uint t = age * 10;

	if(t < 256)
		color = cv::Scalar(255,t,0);

	else if(t<256*2)
		color = cv::Scalar(255-t,255,0);

	else if(t<256*3)
		color = cv::Scalar(0,255,t);

	else if(t<256*4)
		color = cv::Scalar(0,255-t,255);

	else if(t<256*5)
		color = cv::Scalar(t,0,255);

	else
		color = cv::Scalar(255,0,255);

	return color;
}



//---------------------------------------------------------------------------------------
void DataWriter::paintCluster(const Data* data,
                              std::string component_name,
							  const std::vector<std::vector<uint>>& cluster_feature_point_id_list) noexcept
{
    cv::Mat current_image = data->getImageAtT().clone();
    cv::RNG rng(0xFFFFFFFF);

    uint nb_cluster = cluster_feature_point_id_list.size();
    for(int i=0; i<nb_cluster; i++)
	{
    	cv::Scalar color = DataWriter::randomColor(rng);

    	std::vector<std::vector<cv::Point>> hull(1);
    	std::vector<cv::Point> point_set(cluster_feature_point_id_list[i].size());
    	for(int j=0; j<cluster_feature_point_id_list[i].size(); j++)
    	{
    		uint feature_point_id = cluster_feature_point_id_list[i][j];
    		cv::Point2d point_2d(*(data->getFeaturePointPosition2DAtT(feature_point_id)));
    		point_set[j] = cv::Point(point_2d.x, point_2d.y);

            cv::circle(current_image, point_2d, 1, color);
    	}

    	cv::convexHull(cv::Mat(point_set), hull[0], false);
    	cv::drawContours(current_image, hull, 0, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
	}

    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/cluster_convex_hull_%03d.png";
    sprintf(buffer, path.c_str(), data->getTime());
    cv::imwrite(buffer, current_image);
}



//---------------------------------------------------------------------------------------
void DataWriter::paintConfidenceValue(const Data* data,
                                 	  std::string component_name) noexcept
{
    cv::Mat current_image = data->getImageAtT().clone();

    cv::Scalar blue (255,0,0);
    cv::Scalar green(0,255,0);
    cv::Scalar red  (0,0,255);
    cv::Scalar white(255,255,255);

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        if(!data->isFeaturePoint(i))
            continue;

        cv::Point2d point(*(data->getFeaturePointPosition2DAtT(i)));

        if(data->getFeaturePointLabel(i) == UNLABELED)
            cv::circle(current_image, point, 1, white);
        else if(data->getFeaturePointLabel(i) == UNCERTAIN)
            cv::circle(current_image, point, 1, blue);
        else if(data->getFeaturePointLabel(i) == MOVING)
            cv::circle(current_image, point, 1, red);
        else if(data->getFeaturePointLabel(i) == STATIC)
            cv::circle(current_image, point, 1, green);
    }

    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/confidence_value_%03d.png";
    sprintf(buffer, path.c_str(), data->getTime());
    cv::imwrite(buffer, current_image);
}



//---------------------------------------------------------------------------------------
void DataWriter::paintFeaturePoint(const Data* data,
                                   std::string component_name) noexcept
{
    cv::Scalar green(0,255,0);
    cv::Scalar red(0,0,255);
    cv::Scalar color;

    cv::Mat current_image = data->getImageAtT().clone();

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        if(!data->isFeaturePoint(i))
            continue;

        color = ageColor(data->getFeaturePointAge(i));
        cv::Point2d current_point(*(data->getFeaturePointPosition2DAtT(i)));
        cv::circle(current_image, current_point, 1, color);
    }

    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/feature_point_%03d.png";
    sprintf(buffer, path.c_str(), data->getTime());
    cv::imwrite(buffer, current_image);
}



//---------------------------------------------------------------------------------------
void DataWriter::paintKMeansCluster(const Data* data,
									std::string component_name,
									uint nb_cluster,
									const std::vector<cv::Point2f>& points,
									const cv::Mat& labels) noexcept
{
	std::vector<cv::Scalar> cluster_color;
    cv::RNG rng(0xFFFFFFFF);
	for(int i=0; i<nb_cluster; i++)
		cluster_color.push_back(randomColor(rng));

    cv::Mat current_image = data->getImageAtT().clone();

    for(int i=0; i<points.size(); i++)
    {
    	uint label = labels.at<int>(i);
    	cv::circle(current_image, points[i], 1, cluster_color[label]);
    }

    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/kmeans_cluster_%03d.png";
    sprintf(buffer, path.c_str(), data->getTime());
    cv::imwrite(buffer, current_image);
}



//---------------------------------------------------------------------------------------
void DataWriter::paintKMeansClusterCenters(const Data* data,
										   std::string component_name,
										   const std::vector<cv::Point2f>& centers) noexcept
{
	cv::Scalar green(0,255,0);

    cv::Mat current_image = data->getImageAtT().clone();

    for(int i=0; i<centers.size(); i++)
    {
    	cv::circle(current_image, centers[i], 1, green);
    	cv::circle(current_image, centers[i], 5, green);
    }

    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/kmeans_centers_%03d.png";
    sprintf(buffer, path.c_str(), data->getTime());
    cv::imwrite(buffer, current_image);
}



//---------------------------------------------------------------------------------------
void DataWriter::paintOpticalFlow(const Data* data,
                                  std::string component_name)
{
    cv::Scalar green(0,255,0);
    cv::Scalar blue (255,0,0);

    cv::Mat current_image = data->getImageAtT().clone();

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        if(!data->isFeaturePointOpticalFlowOld(i))
            continue;

        cv::Point2d current_point (*(data->getFeaturePointPosition2DAtT(i)));
        cv::Point2d previous_point(data->getFeaturePointPosition2DAtTMDelta(i));

        cv::circle(current_image, current_point,  1, green);
        cv::circle(current_image, previous_point, 1, blue);

        if(previous_point.x == 0
        && previous_point.y == 0)
        	continue;

		cv::line(current_image, current_point, previous_point, green);

		if(i==771)
			cv::line(current_image, current_point, previous_point, cv::Scalar(0,0,0));
    }

    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/optical_flow_%03d.png";
    sprintf(buffer, path.c_str(), data->getTime());
    cv::imwrite(buffer, current_image);
}



//---------------------------------------------------------------------------------------
cv::Scalar DataWriter::randomColor(cv::RNG& rng) noexcept
{
    int icolor = (unsigned) rng;
    return cv::Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255, 255*0.7);
}



//---------------------------------------------------------------------------------------
void DataWriter::paintSlicSuperPixelContour(const Data* data,
        									std::string component_name) noexcept
{
    const int dx8[8] = {-1, -1,  0,  1, 1, 1, 0, -1};
	const int dy8[8] = { 0, -1, -1, -1, 0, 1, 1,  1};

    cv::Mat current_image = data->getImageAtT().clone();

    std::vector<std::vector<int>> clusters;// = data->getSuperPixelCluster();

	/* Initialize the contour vector and the matrix detailing whether a pixel
	 * is already taken to be a contour. */
	std::vector<cv::Point> contours;
	std::vector<std::vector<bool>> istaken;

	for(int i=0; i<current_image.cols; i++)
	{
		std::vector<bool> nb;
        for(int j=0; j<current_image.rows; j++)
        {
            nb.push_back(false);
        }
        istaken.push_back(nb);
    }

    /* Go through all the pixels. */
    for(int i=0; i<current_image.cols; i++)
    {
        for(int j=0; j<current_image.rows; j++)
        {
            int nr_p = 0;

            /* Compare the pixel to its 8 neighbours. */
            for(int k=0; k<8; k++)
            {
                int x = i + dx8[k], y = j + dy8[k];

                if(x >= 0 && y >= 0
                && x < current_image.cols && y < current_image.rows)
                {
                    if(istaken[x][y] == false
                    && clusters[i][j] != clusters[x][y])
                    {
                        nr_p += 1;
                    }
                }
            }

            /* Add the pixel to the contour list if desired. */
            if(nr_p >= 2)
            {
                contours.push_back(cvPoint(i,j));
                istaken[i][j] = true;
            }
        }
    }

    cv::Vec3b color(255,0,0);

    /* Draw the contour pixels. */
    for(int i=0; i<(int)contours.size(); i++)
    {
    	current_image.at<cv::Vec3b>(contours[i].y, contours[i].x) = color;
    }

    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/super_pixel_%03d.png";
    sprintf(buffer, path.c_str(), data->getTime());
    cv::imwrite(buffer, current_image);
}



//---------------------------------------------------------------------------------------
void DataWriter::paintSlicSuperPixelContourWithConfidenceValue(const Data* data,
        													   std::string component_name) noexcept
{
    cv::Scalar blue (255,0,0);
    cv::Scalar green(0,255,0);
    cv::Scalar red  (0,0,255);
    cv::Scalar white(255,255,255);

    const int dx8[8] = {-1, -1,  0,  1, 1, 1, 0, -1};
	const int dy8[8] = { 0, -1, -1, -1, 0, 1, 1,  1};

    cv::Mat current_image = data->getImageAtT().clone();

    std::vector<std::vector<int>> clusters;// = data->getSuperPixelCluster();

	/* Initialize the contour vector and the matrix detailing whether a pixel
	 * is already taken to be a contour. */
	std::vector<cv::Point> contours;
	std::vector<std::vector<bool>> istaken;

	for(int i=0; i<current_image.cols; i++)
	{
		std::vector<bool> nb;
        for(int j=0; j<current_image.rows; j++)
        {
            nb.push_back(false);
        }
        istaken.push_back(nb);
    }

    /* Go through all the pixels. */
    for(int i=0; i<current_image.cols; i++)
    {
        for(int j=0; j<current_image.rows; j++)
        {
            int nr_p = 0;

            /* Compare the pixel to its 8 neighbours. */
            for(int k=0; k<8; k++)
            {
                int x = i + dx8[k], y = j + dy8[k];

                if(x >= 0 && y >= 0
                && x < current_image.cols && y < current_image.rows)
                {
                    if(istaken[x][y] == false
                    && clusters[i][j] != clusters[x][y])
                    {
                        nr_p += 1;
                    }
                }
            }

            /* Add the pixel to the contour list if desired. */
            if(nr_p >= 2)
            {
                contours.push_back(cvPoint(i,j));
                istaken[i][j] = true;
            }
        }
    }

    cv::Vec3b color(255,0,0);

    /* Draw the contour pixels. */
    for(int i=0; i<(int)contours.size(); i++)
    {
    	current_image.at<cv::Vec3b>(contours[i].y, contours[i].x) = color;
    }

    /* Paint confidence value */

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        if(!data->isFeaturePoint(i))
            continue;

        cv::Point2d point(*(data->getFeaturePointPosition2DAtT(i)));

        if(data->getFeaturePointLabel(i) == UNLABELED)
            cv::circle(current_image, point, 1, white);
        else if(data->getFeaturePointLabel(i) == UNCERTAIN)
            cv::circle(current_image, point, 1, blue);
        else if(data->getFeaturePointLabel(i) == MOVING)
            cv::circle(current_image, point, 1, red);
        else if(data->getFeaturePointLabel(i) == STATIC)
            cv::circle(current_image, point, 1, green);
    }

    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/super_pixel_confidence_value_%03d.png";
    sprintf(buffer, path.c_str(), data->getTime());
    cv::imwrite(buffer, current_image);
}



//---------------------------------------------------------------------------------------
void DataWriter::paintSlicSuperPixelRegionWithConfidenceValue(const Data* data,
        													  std::string component_name) noexcept
{
    cv::Vec3b blue (255,0,0);
    cv::Vec3b green(0,255,0);
    cv::Vec3b red  (0,0,255);
    cv::Vec3b white(255,255,255);

    cv::Mat current_image = data->getImageAtT().clone();

    std::vector<std::vector<int>> clusters;// = data->getSuperPixelCluster();

	std::vector<cv::Vec3b> cluster_color(CST(int,cst::NB_SUPER_PIXEL), white);

	std::vector<int> confidence_feature_point(3,0);
	std::vector<std::vector<int>> nb_confidence_feature_point(CST(int,cst::NB_SUPER_PIXEL), confidence_feature_point);

    /* Paint confidence value */
    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        if(!data->isFeaturePoint(i))
            continue;

        cv::Point2d point(*(data->getFeaturePointPosition2DAtT(i)));

        int cluster_num = clusters[point.x][point.y];

        if(cluster_num < 0 || cluster_num > CST(int,cst::NB_SUPER_PIXEL))
        	continue;

        if(data->getFeaturePointLabel(i) == UNCERTAIN)
        	nb_confidence_feature_point[cluster_num][0]++;
		else if(data->getFeaturePointLabel(i) == MOVING)
        	nb_confidence_feature_point[cluster_num][1]++;
		else if(data->getFeaturePointLabel(i) == STATIC)
        	nb_confidence_feature_point[cluster_num][2]++;

        if(nb_confidence_feature_point[cluster_num][0] > nb_confidence_feature_point[cluster_num][1]
        && nb_confidence_feature_point[cluster_num][0] > nb_confidence_feature_point[cluster_num][2])
        	cluster_color[cluster_num] = blue;
        else if(nb_confidence_feature_point[cluster_num][1] > nb_confidence_feature_point[cluster_num][0]
		     && nb_confidence_feature_point[cluster_num][1] > nb_confidence_feature_point[cluster_num][2])
			cluster_color[cluster_num] = red;
        else if(nb_confidence_feature_point[cluster_num][2] > nb_confidence_feature_point[cluster_num][0]
		     && nb_confidence_feature_point[cluster_num][2] > nb_confidence_feature_point[cluster_num][1])
			cluster_color[cluster_num] = green;
    }

    for(int i=0; i<current_image.rows; i++)
    {
    	for(int j=0; j<current_image.cols; j++)
    	{
    		int cluster_num = clusters[j][i];

            if(cluster_num < 0 || cluster_num > CST(int,cst::NB_SUPER_PIXEL))
            	continue;

    		current_image.at<cv::Vec3b>(i,j) = cluster_color[cluster_num];
    	}
    }

    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH)
    				 + "/" + CST(std::string,cst::PROJECT_NAME)
					 + "/" + component_name
					 + "/super_pixel_region_confidence_value_%03d.png";
    sprintf(buffer, path.c_str(), data->getTime());
    cv::imwrite(buffer, current_image);
}



//---------------------------------------------------------------------------------------
void DataWriter::paintSlicSuperPixelRegionWithConfidenceValueBis(const Data* data,
        													  	 std::string component_name) noexcept
{
    cv::Vec3b blue (255,0,0);
    cv::Vec3b green(0,255,0);
    cv::Vec3b red  (0,0,255);
    cv::Vec3b white(255,255,255);

    cv::Mat current_image = data->getImageAtT().clone();

    std::vector<std::vector<int>> clusters;// = data->getSuperPixelCluster();

	std::vector<cv::Vec3b> cluster_color(CST(int,cst::NB_SUPER_PIXEL), white);

	std::vector<int> confidence_feature_point(2,0);
	std::vector<std::vector<int>> nb_confidence_feature_point(CST(int,cst::NB_SUPER_PIXEL), confidence_feature_point);

    /* Paint confidence value */
    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        if(!data->isFeaturePoint(i))
            continue;

        cv::Point2d point(*(data->getFeaturePointPosition2DAtT(i)));

        int cluster_num = clusters[point.x][point.y];

        if(cluster_num < 0 || cluster_num > CST(int,cst::NB_SUPER_PIXEL))
        	continue;

		if(data->getFeaturePointLabel(i) == MOVING)
        	nb_confidence_feature_point[cluster_num][0]++;
		else if(data->getFeaturePointLabel(i) == STATIC)
        	nb_confidence_feature_point[cluster_num][1]++;


        if(nb_confidence_feature_point[cluster_num][0] > nb_confidence_feature_point[cluster_num][1])
			cluster_color[cluster_num] = red;
        else if(nb_confidence_feature_point[cluster_num][1] > nb_confidence_feature_point[cluster_num][0])
			cluster_color[cluster_num] = green;
        else
        	cluster_color[cluster_num] = white;
    }

    for(int i=0; i<current_image.rows; i++)
    {
    	for(int j=0; j<current_image.cols; j++)
    	{
    		int cluster_num = clusters[j][i];

            if(cluster_num < 0 || cluster_num > CST(int,cst::NB_SUPER_PIXEL))
            	continue;

    		current_image.at<cv::Vec3b>(i,j) = cluster_color[cluster_num];
    	}
    }

    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH)
    				 + "/" + CST(std::string,cst::PROJECT_NAME)
					 + "/" + component_name
					 + "/super_pixel_region_confidence_value_bis_%03d.png";
    sprintf(buffer, path.c_str(), data->getTime());
    cv::imwrite(buffer, current_image);
}



//---------------------------------------------------------------------------------------
void DataWriter::paintTrendBonusMalus(const Data* data,
                              	  	  std::string component_name,
									  const std::vector<std::vector<uint>>& cluster_feature_point_id_list,
									  const std::vector<double>& bonus_malus_value) noexcept
{
    cv::Mat current_image = data->getImageAtT().clone();
    cv::Scalar color;
    cv::Scalar blue(255,0,0);
    cv::Scalar green(0,255,0);
    cv::Scalar red  (0,0,255);

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
	{
    	if(!data->isFeaturePointOld(i))
    		continue;

		cv::Point2d point_2d(*(data->getFeaturePointPosition2DAtT(i)));

		if(bonus_malus_value[i] > 0)
			color = green;
		else if(bonus_malus_value[i] < 0)
			color = red;
		else if(bonus_malus_value[i] == 0)
			color = blue;

		cv::circle(current_image, point_2d, 1, color);
	}

    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/trend_bonus_malus_value_%03d.png";
    sprintf(buffer, path.c_str(), data->getTime());
    cv::imwrite(buffer, current_image);
}



//---------------------------------------------------------------------------------------
void DataWriter::paintTrendClusterBonusMalus(const Data* data,
                              	  	  	  	 std::string component_name,
											 const std::vector<std::vector<uint>>& cluster_feature_point_id_list,
											 const std::vector<double>& cluster_bonus_malus_value) noexcept
{
    cv::Mat current_image = data->getImageAtT().clone();
    cv::Scalar color;
    cv::Scalar blue(255,0,0);
    cv::Scalar green(0,255,0);
    cv::Scalar red  (0,0,255);

    uint nb_cluster = cluster_feature_point_id_list.size();
    for(int i=0; i<nb_cluster; i++)
	{
    	std::vector<std::vector<cv::Point>> hull(1);
    	std::vector<cv::Point> point_set(cluster_feature_point_id_list[i].size());
    	for(int j=0; j<cluster_feature_point_id_list[i].size(); j++)
    	{
    		uint feature_point_id = cluster_feature_point_id_list[i][j];
    		cv::Point2d point_2d(*(data->getFeaturePointPosition2DAtT(feature_point_id)));
    		point_set[j] = cv::Point(point_2d.x, point_2d.y);

    		if(cluster_bonus_malus_value[i] > 0)
    			color = green;
    		else if(cluster_bonus_malus_value[i] < 0)
    			color = red;
    		else if(cluster_bonus_malus_value[i] == 0)
    			color = blue;

			cv::circle(current_image, point_2d, 1, color);
    	}

    	cv::convexHull(cv::Mat(point_set), hull[0], false);
    	cv::drawContours(current_image, hull, 0, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
	}

    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/trend_cluster_bonus_malus_value_%03d.png";
    sprintf(buffer, path.c_str(), data->getTime());
    cv::imwrite(buffer, current_image);
}



//---------------------------------------------------------------------------------------
void DataWriter::writeBlenderFile(const Data* data,
								  std::string component_name) noexcept
{
    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/blender_%03d.txt";
    sprintf(buffer, path.c_str(), data->getTime());

    std::ofstream file;
    file.open(buffer);

    uint count = 0;
    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        cv::Mat point_3d = cv::Mat::zeros(3,1,CV_64F);

        if(data->isFeaturePointOld(i))
        {
            point_3d = *(data->getFeaturePointPosition3DAtT(i));
            count++;
        }

        file << point_3d.at<double>(0)    << " "
             << point_3d.at<double>(2)    << " "
             << point_3d.at<double>(1)*-1 << std::endl;
    }
    file.close();
}




//---------------------------------------------------------------------------------------
void DataWriter::writeCluster(const Data* data,
                              std::string component_name,
							  const std::vector<std::vector<uint>>& cluster_feature_point_id_list) noexcept
{
    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/cluster_%03d.txt";
    sprintf(buffer, path.c_str(), data->getTime());

    std::ofstream file;
    file.open(buffer);

    uint nb_cluster = cluster_feature_point_id_list.size();
    file << nb_cluster << std::endl;

    for(int i=0; i<nb_cluster; i++)
    {
    	uint nb_point_in_cluster = cluster_feature_point_id_list[i].size();
    	for(int j=0; j<nb_point_in_cluster; j++)
    	{
    		uint feature_point_id = cluster_feature_point_id_list[i][j];
    		file << i << " " << feature_point_id << std::endl;
    	}
    }

    file.close();
}




//----------------------------------------------------------------------------------------
void DataWriter::writeConfidenceValue(const Data* data,
                                      std::string component_name) noexcept
{
    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/confidence_value_%03d.txt";
    sprintf(buffer, path.c_str(), data->getTime());

    std::ofstream file;
    file.open(buffer);

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
        file << data->getFeaturePointConfidenceValue(i) << std::endl;

    file.close();
}



//---------------------------------------------------------------------------------------
void DataWriter::writeDistanceToCamera(const Data* data,
                                       std::string component_name) noexcept
{
    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/distance_to_camera_%03d.txt";
    sprintf(buffer, path.c_str(), data->getTime());

    std::ofstream file;
    file.open(buffer);

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        if(!data->isFeaturePointOld(i))
            file << 0 << std::endl;
        else
            file << data->getFeaturePointDistanceToCameraAtT(i) << std::endl;
    }

    file.close();
}



//---------------------------------------------------------------------------------------
void DataWriter::writeExtrinsicParameters(const Data* data,
									  	  std::string component_name,
										  const cv::Mat& rotation,
										  const cv::Mat& translation) noexcept
{
    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/extrinsic_parameters_%03d.txt";
    sprintf(buffer, path.c_str(), data->getTime());

    std::ofstream file;
    file.open(buffer);

    cv::Mat rotation_vec(3,1,CV_64F);
    cv::Rodrigues(rotation, rotation_vec);

    file << "Computed with all point" << std::endl;

    file << "Rotation matrix" << std::endl;
    file << rotation.at<double>(0,0) << " " << rotation.at<double>(0,1) << " " << rotation.at<double>(0,2) << std::endl;
    file << rotation.at<double>(1,0) << " " << rotation.at<double>(1,1) << " " << rotation.at<double>(1,2) << std::endl;
    file << rotation.at<double>(2,0) << " " << rotation.at<double>(2,1) << " " << rotation.at<double>(2,2) << std::endl;
    file << std::endl;

    file << "Rotation vector (radian)" << std::endl;
    file << rotation_vec.at<double>(0) << " " << rotation_vec.at<double>(1) << " " << rotation_vec.at<double>(2) << std::endl;
    file << std::endl;

    file << "Rotation vector (degree)" << std::endl;
    file << rotation_vec.at<double>(0)*180./M_PI << " " << rotation_vec.at<double>(1)*180./M_PI << " " << rotation_vec.at<double>(2)*180./M_PI << std::endl;
    file << std::endl;

    file << "Translation vector" << std::endl;
    file << translation.at<double>(0) << " " << translation.at<double>(1) << " " << translation.at<double>(2) << std::endl;

    file.close();
}



//---------------------------------------------------------------------------------------
void DataWriter::writeFeaturePointPosition(const Data* data,
                                   	   	   std::string component_name) noexcept
{
    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/feature_point_position_%03d.txt";
    sprintf(buffer, path.c_str(), data->getTime());

    std::ofstream file;
    file.open(buffer);

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        FeaturePoint feature_point = data->getFeaturePoint(i);
        cv::Mat fp = *(feature_point.get2DPositionAtT());
        file << fp.at<double>(0) << " "
             << fp.at<double>(1) << std::endl;
    }

    file.close();
}



//---------------------------------------------------------------------------------------
void DataWriter::writeFeaturePointPositionAge(const Data* data,
                                   	   	   	  std::string component_name) noexcept
{
    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/feature_point_position_age_%03d.txt";
    sprintf(buffer, path.c_str(), data->getTime());

    std::ofstream file;
    file.open(buffer);

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        FeaturePoint feature_point = data->getFeaturePoint(i);
        cv::Mat fp = *(feature_point.get2DPositionAtT());
        file << fp.at<double>(0) << " "
             << fp.at<double>(1) << " "
             << feature_point.getAge() << std::endl;
    }

    file.close();
}



//---------------------------------------------------------------------------------------
void DataWriter::writeLabel(const Data* data,
                            std::string component_name) noexcept
{
    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/label_%03d.txt";
    sprintf(buffer, path.c_str(), data->getTime());

    std::ofstream file;
    file.open(buffer);

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
        file << data->getFeaturePointLabel(i) << std::endl;

    file.close();
}



//---------------------------------------------------------------------------------------
void DataWriter::writeMedianScale(const Data* data,
                                  std::string component_name,
                                  double median_scale) noexcept
{
    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/median_scale_%03d.txt";
    sprintf(buffer, path.c_str(), data->getTime());

    std::ofstream file;
    file.open(buffer);

    file << median_scale << std::endl;

    file.close();
}



//---------------------------------------------------------------------------------------
void DataWriter::writeState(const Data* data,
                            std::string component_name,
                            std::string state) noexcept
{
    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/state_%03d.txt";
    sprintf(buffer, path.c_str(), data->getTime());

    std::ofstream file;
    file.open(buffer);

    file << state << std::endl;

    file.close();
}



//---------------------------------------------------------------------------------------
void DataWriter::writeStaticFeaturePointPositionAtT(const Data* data,
                                   	   	   	        std::string component_name) noexcept
{
    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/"
    				 + CST(std::string,cst::PROJECT_NAME) + "/"
					 + component_name
					 + "/static_feature_point_position_t_%03d.txt";
    sprintf(buffer, path.c_str(), data->getTime());

    std::ofstream file;
    file.open(buffer);

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
    	if(!data->isFeaturePointStatic(i))
    		continue;

        FeaturePoint feature_point = data->getFeaturePoint(i);
        cv::Mat fp = *(feature_point.get2DPositionAtT());
        file << fp.at<double>(0) << " "
             << fp.at<double>(1) << std::endl;
    }

    file.close();
}



//---------------------------------------------------------------------------------------
void DataWriter::writeStaticFeaturePointPositionAtTMDelta(const Data* data,
                                   	   	   	        	  std::string component_name) noexcept
{
    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/"
    				 + CST(std::string,cst::PROJECT_NAME) + "/"
					 + component_name
					 + "/static_feature_point_position_tmdelta_%03d.txt";
    sprintf(buffer, path.c_str(), data->getTime());

    std::ofstream file;
    file.open(buffer);

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
    	if(!data->isFeaturePointStatic(i))
    		continue;

        FeaturePoint feature_point = data->getFeaturePoint(i);
        cv::Mat fp = feature_point.get2DPositionAtTMDelta();
        file << fp.at<double>(0) << " "
             << fp.at<double>(1) << std::endl;
    }

    file.close();
}



//---------------------------------------------------------------------------------------
void DataWriter::writeTrendBonusMalus(const Data* data,
                              	  	  std::string component_name,
									  const std::vector<std::vector<uint>>& cluster_feature_point_id_list,
									  const std::vector<double>& bonus_malus_value) noexcept
{
    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/trend_bonus_malus_value_%03d.txt";
    sprintf(buffer, path.c_str(), data->getTime());

    std::ofstream file;
    file.open(buffer);

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
	{
    	if(!data->isFeaturePointOld(i))
    		file << -2 << std::endl;
    	else
    		file << bonus_malus_value[i] << std::endl;
	}

    file.close();
}



//---------------------------------------------------------------------------------------
void DataWriter::writeTrendClusterBonusMalus(const Data* data,
                              	  	  	  	 std::string component_name,
											 const std::vector<std::vector<uint>>& cluster_feature_point_id_list,
											 const std::vector<double>& cluster_bonus_malus_value) noexcept
{
    char buffer[500];
    std::string path = CST(std::string,cst::MOBDEC_RESULT_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + component_name + "/trend_cluster_bonus_malus_value_%03d.txt";
    sprintf(buffer, path.c_str(), data->getTime());

    std::ofstream file;
    file.open(buffer);

    uint nb_cluster = cluster_feature_point_id_list.size();
    for(int i=0; i<nb_cluster; i++)
	{
    	file << nb_cluster << std::endl;

    	for(int j=0; j<cluster_feature_point_id_list[i].size(); j++)
    	{
    		uint feature_point_id = cluster_feature_point_id_list[i][j];
			file << feature_point_id << " " << cluster_bonus_malus_value[i] << std::endl;;
    	}
	}

    file.close();
}














































