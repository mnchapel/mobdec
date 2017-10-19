/// @file   video.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18


#include <core/video.h>



//---------------------------------------------------------------------------------------
Video::Video() noexcept
{
	image.resize(CST(int,cst::NB_ROWS));
}



//---------------------------------------------------------------------------------------
void Video::readImage(uint num_frame) noexcept
{
	char image_path[500];
	sprintf(image_path, CST(std::string,cst::TEMPLATE_IMAGE_PATH).c_str(), num_frame);
	cv::Mat current_image = cv::imread(image_path);

    current_index++;
    current_index = current_index%CST(int,cst::NB_ROWS);
    image[current_index] = current_image;

    assert(("The image is not loaded" , !image[current_index].empty()));
}



//---------------------------------------------------------------------------------------
const cv::Mat& Video::getImageAtT() const noexcept
{
	assert(image[current_index].rows != 0 && image[current_index].cols != 0);
    return image[current_index];
}



//---------------------------------------------------------------------------------------
const cv::Mat& Video::getImageAtTM1() const noexcept
{
    assert(image[(current_index+CST(int,cst::NB_ROWS)-1)%CST(int,cst::NB_ROWS)].rows != 0
    	&& image[(current_index+CST(int,cst::NB_ROWS)-1)%CST(int,cst::NB_ROWS)].cols != 0);

    return image[(current_index+CST(int,cst::NB_ROWS)-1)%CST(int,cst::NB_ROWS)];
}



//---------------------------------------------------------------------------------------
const cv::Mat& Video::getImageAtTMDelta() const noexcept
{
    assert(image[(current_index+1)%CST(int,cst::NB_ROWS)].rows != 0
    	&& image[(current_index+1)%CST(int,cst::NB_ROWS)].cols != 0);

    return image[(current_index+1)%CST(int,cst::NB_ROWS)];
}



//---------------------------------------------------------------------------------------
const uint Video::getImageCol() const noexcept
{
	return image[current_index].cols;
}



//---------------------------------------------------------------------------------------
const uint Video::getImageRow() const noexcept
{
	return image[current_index].rows;
}











