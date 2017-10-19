/// @file   labelingStaticInitialisation.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <component/labeling/labelingStaticInitialisation.h>



//---------------------------------------------------------------------------------------
void LabelingStaticInitialisation::compute() noexcept
{
	for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
	{
		if(!data->isFeaturePointReconstructionOld(i))
			continue;
		data->updateConfidenceValue(i,1.);
    }
}



//---------------------------------------------------------------------------------------
std::string LabelingStaticInitialisation::getComponentName() const noexcept
{
    return "LabelingStaticInitialisation";
}



//---------------------------------------------------------------------------------------
std::string LabelingStaticInitialisation::getPhaseReturn() const noexcept
{
	return "NextPhase";
}



//---------------------------------------------------------------------------------------
bool LabelingStaticInitialisation::isEnoughTimeElapsed() const noexcept
{
    return true;
}



//---------------------------------------------------------------------------------------
void LabelingStaticInitialisation::readFileData() noexcept
{

}



















