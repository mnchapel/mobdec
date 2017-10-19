/// @file   camera.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <core/camera.h>



//---------------------------------------------------------------------------------------
Camera::Camera(const cv::Mat& intrinsic_parameters) noexcept
	: intrinsic_parameters(intrinsic_parameters)
{
	extrinsic_parameters.resize(CST(int,cst::NB_ROWS));
	for(int i=0; i<CST(int,cst::NB_ROWS); i++)
		extrinsic_parameters[i].resize(2);
}



//---------------------------------------------------------------------------------------
const cv::Mat& Camera::getIntrinsicParameters() const noexcept
{
	return intrinsic_parameters;
}



//---------------------------------------------------------------------------------------
const cv::Mat& Camera::getRotation(uint index) const noexcept
{
	return extrinsic_parameters[index][0];
}



//---------------------------------------------------------------------------------------
const cv::Mat& Camera::getTranslation(uint index) const noexcept
{
	return extrinsic_parameters[index][1];
}



//---------------------------------------------------------------------------------------
void Camera::updateExtrinsicParameters(uint indexAtT,
									   cv::Mat& rotation,
									   cv::Mat& translation) noexcept
{
	extrinsic_parameters[indexAtT][0] = rotation;
	extrinsic_parameters[indexAtT][1] = translation;
}
