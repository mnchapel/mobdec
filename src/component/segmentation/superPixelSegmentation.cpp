/// @file   superPixelSegmentation.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <component/segmentation/superPixelSegmentation.h>



//---------------------------------------------------------------------------------------
void SuperPixelSegmentation::compute() noexcept
{
    computeSegmentation();
}



//---------------------------------------------------------------------------------------
std::string SuperPixelSegmentation::getComponentName() const noexcept
{
    return "SuperPixelSegmentation";
}



//---------------------------------------------------------------------------------------
void SuperPixelSegmentation::computeSegmentation() noexcept
{
	int DELAY_BLUR = 100;
	int MAX_KERNEL_LENGTH_1 = 15;
	int MAX_KERNEL_LENGTH_2 = 21;


	// BLUR
	cv::Mat image = data->getImageAtT();
//	cv::Mat image_blur_1;
//	cv::Mat image_blur_2;
//
//	/* Gaussian Blur */
//	for(int i=1; i<MAX_KERNEL_LENGTH_1; i=i+2)
//		GaussianBlur(image, image_blur_1, cv::Size(i, i), 0, 0);
//
//	for(int i=1; i<MAX_KERNEL_LENGTH_2; i=i+2)
//		GaussianBlur(image, image_blur_2, cv::Size(i, i), 0, 0);
//
//	cv::Mat image_blur_gray_1 = image_blur_1;
//	cv::Mat image_blur_gray_2 = image_blur_2;
//	cv::Mat image_gray;
//  image = image_blur_1;
	// --------

//	cv::cvtColor(image_blur_1, image_blur_gray_1, CV_BGR2GRAY);
//	cv::cvtColor(image_blur_2, image_blur_gray_2, CV_BGR2GRAY);

    /* Load the image and convert to Lab colour space. */

//	char buffer[500];
//	sprintf(buffer, "image_dog_%03d.png", data->getTime());
//	cv::imwrite(buffer, image);

//	cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
    cv::Mat lab_image;
    image.copyTo(lab_image);
    cv::cvtColor(image, lab_image, CV_BGR2Lab);

    /* Yield the number of superpixels and weight-factors from the user. */
    uint w = data->getImageCol();
    uint h = data->getImageRow();
    int nr_superpixels = CST(int,cst::NB_SUPER_PIXEL);
    int nc = CST(int,cst::SLIC_WEIGHT);

    double step = sqrt((w * h) / (double) nr_superpixels);

	std::vector<int> center_counts;
	std::vector<std::vector<int>> clusters;
	std::vector<std::vector<double>> distances;
	std::vector<std::vector<double>> centers;

    generate_superpixels(lab_image, step, nc, clusters, distances, centers, center_counts);
    create_connectivity(lab_image, centers.size(), clusters);

#ifndef NDEBUG
    DataWriter::paintSlicSuperPixelContour(data, getComponentName());
    DataWriter::paintSlicSuperPixelContourWithConfidenceValue(data, getComponentName());
    DataWriter::paintSlicSuperPixelRegionWithConfidenceValue(data, getComponentName());
    DataWriter::paintSlicSuperPixelRegionWithConfidenceValueBis(data, getComponentName());
#endif
}



//---------------------------------------------------------------------------------------
/*
 * Enforce connectivity of the superpixels. This part is not actively discussed
 * in the paper, but forms an active part of the implementation of the authors
 * of the paper.
 *
 * Input : The image (IplImage*).
 * Output: -
 * util
 */
void SuperPixelSegmentation::create_connectivity(cv::Mat& image,
												 uint centers_size,
												 const std::vector<std::vector<int>>& clusters) noexcept
{
    int label = 0, adjlabel = 0;
    const int lims = (image.cols * image.rows) / ((int)centers_size);

    const int dx4[4] = {-1,  0,  1,  0};
	const int dy4[4] = { 0, -1,  0,  1};


    /* Initialize the new cluster matrix. */
	std::vector<std::vector<int>> new_clusters;
    for(int i=0; i<image.cols; i++)
    {
    	std::vector<int> nc;
        for(int j=0; j<image.rows; j++)
        {
            nc.push_back(-1);
        }
        new_clusters.push_back(nc);
    }

    for(int i=0; i<image.cols; i++)
    {
        for(int j=0; j<image.rows; j++)
        {
            if(new_clusters[i][j] == -1)
            {
            	std::vector<CvPoint> elements;
                elements.push_back(cvPoint(i, j));

                /* Find an adjacent label, for possible use later. */
                for(int k=0; k<4; k++) {
                    int x = elements[0].x + dx4[k], y = elements[0].y + dy4[k];

                    if(x >= 0 && x < image.cols && y >= 0 && y < image.rows)
                    {
                        if (new_clusters[x][y] >= 0) {
                            adjlabel = new_clusters[x][y];
                        }
                    }
                }

                int count = 1;
                for(int c=0; c<count; c++)
                {
                    for(int k=0; k<4; k++)
                    {
                        int x = elements[c].x + dx4[k], y = elements[c].y + dy4[k];

                        if(x>=0 && x<image.cols
                        && y>=0 && y<image.rows)
                        {
                            if(new_clusters[x][y] == -1
                            && clusters[i][j] == clusters[x][y])
                            {
                                elements.push_back(cvPoint(x, y));
                                new_clusters[x][y] = label;
                                count += 1;
                            }
                        }
                    }
                }

                /* Use the earlier found adjacent label if a segment size is
                   smaller than a limit. */
                if(count<=lims >> 2)
                {
                    for(int c=0; c<count; c++)
                    {
                        new_clusters[elements[c].x][elements[c].y] = adjlabel;
                    }
                    label -= 1;
                }
                label += 1;
            }
        }
    }
}



//---------------------------------------------------------------------------------------
/*
 * Compute the distance between a cluster center and an individual pixel.
 *
 * Input : The cluster index (int), the pixel (CvPoint), and the Lab values of
 *         the pixel (CvScalar).
 * Output: The distance (double).
 * util
 */
double SuperPixelSegmentation::distance(int ci,
										int nc,
										int ns,
					  	  	  	  	    cv::Point pixel,
										cv::Vec3b colour,
										const std::vector<std::vector<double>>& centers) noexcept
{
    double dc = sqrt(pow(centers[ci][0] - colour.val[0], 2) + pow(centers[ci][1]
              - colour.val[1], 2) + pow(centers[ci][2] - colour.val[2], 2));

    double ds = sqrt(pow(centers[ci][3] - pixel.x, 2) + pow(centers[ci][4] - pixel.y, 2));

    return sqrt(pow(dc/nc, 2) + pow(ds/ns, 2));
}



//---------------------------------------------------------------------------------------
cv::Point SuperPixelSegmentation::find_local_minimum(cv::Mat& image,
								   	   	   	   	     cv::Point center)
{
    double min_grad = FLT_MAX;
    CvPoint loc_min = cvPoint(center.x, center.y);

    for(int i=center.x-1; i<center.x+2; i++)
    {
        for(int j=center.y-1; j<center.y+2; j++)
        {
        	cv::Vec3b c1 = image.at<cv::Vec3b>(j+1, i);
            cv::Vec3b c2 = image.at<cv::Vec3b>(j, i+1);
            cv::Vec3b c3 = image.at<cv::Vec3b>(j, i);

            /* Convert colour values to grayscale values. */
            double i1 = c1.val[0];
            double i2 = c2.val[0];
            double i3 = c3.val[0];

            /* Compute horizontal and vertical gradients and keep track of the
               minimum. */
            if(sqrt(pow(i1 - i3, 2)) + sqrt(pow(i2 - i3,2)) < min_grad)
            {
                min_grad = fabs(i1 - i3) + fabs(i2 - i3);
                loc_min.x = i;
                loc_min.y = j;
            }
        }
    }

    return loc_min;
}



//---------------------------------------------------------------------------------------
/*
 * Compute the over-segmentation based on the step-size and relative weighting
 * of the pixel and colour values.
 *
 * Input : The Lab image (IplImage*), the stepsize (int), and the weight (int).
 * Output: -
 * util
 */
void SuperPixelSegmentation::generate_superpixels(cv::Mat& image,
												  int step,
												  int nc,
												  std::vector<std::vector<int>>& clusters,
												  std::vector<std::vector<double>>& distances,
												  std::vector<std::vector<double>>& centers,
												  std::vector<int>& center_counts)
{
	uint ns = step;

    /* Clear previous data (if any), and re-initialize it. */
    init_data(step, clusters, distances, centers, center_counts,image);

    /* Run EM for 10 iterations (as prescribed by the algorithm). */
    for(int i=0; i<CST(int,cst::NB_ITERATIONS); i++)
    {
        /* Reset distance values. */
        for(int j=0; j<image.cols; j++)
            for(int k=0; k<image.rows; k++)
                distances[j][k] = FLT_MAX;

        for(int j=0; j<(int)centers.size(); j++)
        {
            /* Only compare to pixels in a 2 x step by 2 x step region. */
            for(int k=centers[j][3]-step; k<centers[j][3]+step; k++)
            {
                for(int l=centers[j][4]-step; l<centers[j][4]+step; l++)
                {

                    if(k >= 0 && k < image.cols && l >= 0 && l < image.rows)
                    {
                    	cv::Vec3b colour = image.at<cv::Vec3b>(l, k);
                        double d = distance(j, nc, ns, cv::Point(k,l), colour, centers);

                        /* Update cluster allocation if the cluster minimizes the
                           distance. */
                        if(d < distances[k][l])
                        {
                            distances[k][l] = d;
                            clusters[k][l] = j;
                        }
                    }
                }
            }
        }

        /* Clear the center values. */
        for(int j=0; j<centers.size(); j++)
        {
            centers[j][0] = centers[j][1] = centers[j][2] = centers[j][3] = centers[j][4] = 0;
            center_counts[j] = 0;
        }

        /* Compute the new cluster centers. */
        for(int j=0; j<image.cols; j++)
        {
            for(int k=0; k<image.rows; k++)
            {
                int c_id = clusters[j][k];

                if (c_id != -1) {
                	cv::Vec3b colour = image.at<cv::Vec3b>(k, j);

                    centers[c_id][0] += colour.val[0];
                    centers[c_id][1] += colour.val[1];
                    centers[c_id][2] += colour.val[2];
                    centers[c_id][3] += j;
                    centers[c_id][4] += k;

                    center_counts[c_id] += 1;
                }
            }
        }

        /* Normalize the clusters. */
        for(int j=0; j<centers.size(); j++)
        {
            centers[j][0] /= center_counts[j];
            centers[j][1] /= center_counts[j];
            centers[j][2] /= center_counts[j];
            centers[j][3] /= center_counts[j];
            centers[j][4] /= center_counts[j];
        }
    }
}



//---------------------------------------------------------------------------------------
/*
 * Initialize the cluster centers and initial values of the pixel-wise cluster
 * assignment and distance values.
 *
 * Input : The image (IplImage*).
 * Output: -
 * util
 */
void SuperPixelSegmentation::init_data(uint step,
									   std::vector<std::vector<int>>& clusters,
									   std::vector<std::vector<double>>& distances,
									   std::vector<std::vector<double>>& centers,
									   std::vector<int>& center_counts,
									   cv::Mat& image)
{
    /* Initialize the cluster and distance matrices. */
    for (int i = 0; i < image.cols; i++)
    {
        std::vector<int> cr;
        std::vector<double> dr;
        for (int j = 0; j < image.rows; j++)
        {
            cr.push_back(-1);
            dr.push_back(FLT_MAX);
        }
        clusters.push_back(cr);
        distances.push_back(dr);
    }

    /* Initialize the centers and counters. */
    for (int i=step; i<image.cols-step/2; i+=step)
    {
        for (int j=step; j<image.rows-step/2; j+=step)
        {
        	std::vector<double> center;
            /* Find the local minimum (gradient-wise). */
            cv::Point nc = find_local_minimum(image, cv::Point(i,j));
            cv::Vec3b colour = image.at<cv::Vec3b>(nc.y, nc.x);

            /* Generate the center vector. */
            center.push_back(colour.val[0]);
            center.push_back(colour.val[1]);
            center.push_back(colour.val[2]);
            center.push_back(nc.x);
            center.push_back(nc.y);

            /* Append to vector of centers. */
            centers.push_back(center);
            center_counts.push_back(0);
        }
    }
}



//---------------------------------------------------------------------------------------
bool SuperPixelSegmentation::isEnoughTimeElapsed() const noexcept
{
    if(data->getTimeElapsed() >= (CST(int,cst::DELTA)*2))
        return true;
    return false;
}



//---------------------------------------------------------------------------------------
void SuperPixelSegmentation::readFileData() noexcept
{

}



//---------------------------------------------------------------------------------------
void SuperPixelSegmentation::splitClusterOnColor(std::vector<std::vector<int>>& clusters) noexcept
{
	std::vector<int> nb_feature_point_per_ci(3,0);
	std::vector<std::vector<int>> nb_feature_point_per_ci_cluster(CST(int,cst::NB_SUPER_PIXEL), nb_feature_point_per_ci);

	for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
	{
		if(!data->isFeaturePoint(i))
			continue;

		cv::Point2d feature_point(*(data->getFeaturePointPosition2DAtT(i)));

		int cluster_num = clusters[feature_point.x][feature_point.y];

		if(cluster_num > CST(int,cst::NB_SUPER_PIXEL))
			continue;

        if(data->getFeaturePointLabel(i) == UNCERTAIN)
        	nb_feature_point_per_ci_cluster[cluster_num][0]++;
		else if(data->getFeaturePointLabel(i) == MOVING)
			nb_feature_point_per_ci_cluster[cluster_num][1]++;
		else if(data->getFeaturePointLabel(i) == STATIC)
			nb_feature_point_per_ci_cluster[cluster_num][2]++;
	}

	bool segment;

	uint count = 0;
	for(int i=0; i<CST(int,cst::NB_SUPER_PIXEL); i++)
	{
		segment = false;

		if(nb_feature_point_per_ci_cluster[i][0] != 0
		&& nb_feature_point_per_ci_cluster[i][1] != 0)
			segment = true;
		else if(nb_feature_point_per_ci_cluster[i][1] != 0
			 && nb_feature_point_per_ci_cluster[i][2] != 0)
			segment = true;
//		else if(nb_feature_point_per_ci_cluster[i][0] != 0
//			 && nb_feature_point_per_ci_cluster[i][2] != 0)
//			segment = true;

		if(segment)
			count++;

		if(segment)
		{
			/* Load the image and convert to Lab colour space. */
		    cv::Mat image = data->getImageAtT().clone();
		    cv::Mat lab_image;

		    int min_x = image.cols;
		    int min_y = image.rows;
		    int max_x = 0;
		    int max_y = 0;

		    for(int k=0; k<image.rows; k++)
		    {
		    	for(int l=0; l<image.cols; l++)
		    	{
		    		if(clusters[l][k] != i)
		    			image.at<cv::Vec3b>(k,l) = cv::Vec3b(0,0,0);
		    		else
		    		{
		    			min_x = (l<min_x)?l:min_x;
		    			min_y = (k<min_y)?k:min_y;
		    			max_x = (l>max_x)?l:max_x;
		    			max_y = (k>max_y)?k:max_y;
		    		}
		    	}
		    }

		    cv::Rect rect = cv::Rect(min_x, min_y, max_x-min_x, max_y-min_y);
		    cv::Mat sub_image = image(rect);

		    std::cout << min_x << " " << min_y << " " << max_x << " " << max_y << std::endl;
		    std::cout << sub_image.rows << " " << sub_image.cols << std::endl;

		    sub_image.copyTo(lab_image);
		    cv::cvtColor(sub_image, lab_image, CV_BGR2Lab);

//		    cv::imshow("image" , image);
//		    cv::imshow("sub_image" , sub_image);
//		    cv::waitKey();

		    /* Yield the number of superpixels and weight-factors from the user. */
		    uint w = sub_image.cols;
		    uint h = sub_image.rows;
		    int nr_superpixels = 5;
		    int nc = 80;

		    double step = sqrt((w * h) / (double) nr_superpixels);

			std::vector<int> center_counts;
			std::vector<std::vector<int>> clusters_bis;
			std::vector<std::vector<double>> distances;
			std::vector<std::vector<double>> centers;

		    generate_superpixels(lab_image, step, nc, clusters_bis, distances, centers, center_counts);
		    create_connectivity(lab_image, centers.size(), clusters_bis);

		    paintSlicSuperPixelContourWithConfidenceValue(i, sub_image.clone());
		}
	}

	std::cout << "nb cluster to split " << count << std::endl;
}



//---------------------------------------------------------------------------------------
void SuperPixelSegmentation::paintSlicSuperPixelContourWithConfidenceValue(uint num,
																		   cv::Mat current_image) noexcept
{

    const int dx8[8] = {-1, -1,  0,  1, 1, 1, 0, -1};
    const int dy8[8] = { 0, -1, -1, -1, 0, 1, 1,  1};

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
    std::string path = CST(std::string,cst::MOBDEC_DATA_PATH)
    				 + "/" + CST(std::string,cst::PROJECT_NAME)
					 + "/" + getComponentName()
					 + "/sub_image_%03d_cluster_%03d.png";

    sprintf(buffer, path.c_str(), data->getTime(), num);
    cv::imwrite(buffer, current_image);
}














