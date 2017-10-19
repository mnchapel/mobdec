/// @file   scaleInBox.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <component/reconstruction/scaleInBox.h>



//---------------------------------------------------------------------------------------
void ScaleInBox::compute() noexcept
{
    computeDistanceInUnitBox();

#ifndef NDEBUG
    DataWriter::writeDistanceToCamera(data, getComponentName());
    DataWriter::writeBlenderFile(data, getComponentName());
#endif
}



//---------------------------------------------------------------------------------------
std::string ScaleInBox::getComponentName() const noexcept
{
    return "ScaleInBox";
}



//---------------------------------------------------------------------------------------
void ScaleInBox::computeDistanceInUnitBox() noexcept
{
    double max_distance = 0.;
    std::vector<double> distance_test;

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        if(!data->isFeaturePointOld(i))
            continue;

        double distance = (*data->getFeaturePointPosition3DAtT(i)).at<double>(2);
        distance_test.push_back(distance);

        max_distance  = (max_distance < distance)?distance:max_distance;
    }

    std::sort(distance_test.begin(), distance_test.end());

    uint max_dist = 0;
    uint count = 0;
    for(int i=distance_test.size()-1; i>0; i--)
    {
        max_dist = distance_test[i];

        while(fabs(max_dist - distance_test[i]) < 1)
        {
            count++;
            i--;
        }

        if(count >= 10)
            break;
    }

    DEBUG_COMPONENT_MSG("max distance ");

    for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        if(!data->isFeaturePointOld(i))
            continue;

        double distance = data->getFeaturePointDistanceToCameraAtT(i);
        data->updateFeaturePointDistanceToCamera(i, distance/max_dist * CST(int,cst::SCALE_MAX_DISTANCE));
    }
}



//---------------------------------------------------------------------------------------
bool ScaleInBox::isEnoughTimeElapsed() const noexcept
{
    if(data->getTimeElapsed() >= CST(int,cst::DELTA))
        return true;
    return false;
}



//---------------------------------------------------------------------------------------
void ScaleInBox::readFileData() noexcept
{

}


















