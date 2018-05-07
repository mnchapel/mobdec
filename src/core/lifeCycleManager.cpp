/// @file   lifeCycleManager.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <core/lifeCycleManager.h>



//---------------------------------------------------------------------------------------
LifeCycleManager::LifeCycleManager(int end_time_cycle,
								   const std::vector<std::string>& list_phase) noexcept
{
	this->end_time_cycle = end_time_cycle;

    for(std::string phase_name: list_phase)
    {
        if(phase_name == "AnmsDetector")
            phases.emplace_back(new AnmsDetector);
        else if(phase_name == "CombiningGeometricConstraintOpticalFlow")
            phases.emplace_back(new CombiningGeometricConstraintOpticalFlow);
		else if(phase_name == "R_CombiningGeometricConstraintOpticalFlow")
        // 	phases.emplace_back(new CombiningGeometricConstraintOpticalFlow{true});
		// else if(phase_name == "DeleteFeaturePointWithGroundTruth")
            phases.emplace_back(new DeleteFeaturePointWithGroundTruth);
        else if(phase_name == "EssentialMatrix")
            phases.emplace_back(new EssentialMatrix);
        else if(phase_name == "EssentialMatrixKMeans")
            phases.emplace_back(new EssentialMatrixKMeans);
        else if(phase_name == "FalseNegativeSuppression")
            phases.emplace_back(new FalseNegativeSuppression);
        else if(phase_name == "FastDetector")
            phases.emplace_back(new FastDetector);
        else if(phase_name == "KltTracker")
            phases.emplace_back(new KltTracker);
        else if(phase_name == "KMeansOpticalFlow")
            phases.emplace_back(new KMeansOpticalFlow);
        else if(phase_name == "LabelingFeaturePoint")
            phases.emplace_back(new LabelingFeaturePoint);
        else if(phase_name == "LabelingStaticInitialisation")
            phases.emplace_back(new LabelingStaticInitialisation);
        else if(phase_name == "LdofOptF")
            phases.emplace_back(new LdofOptF);
        else if(phase_name == "Reconstruction3dTriangulation")
            phases.emplace_back(new Reconstruction3dTriangulation);
        else if(phase_name == "ReconstructionQuality")
            phases.emplace_back(new ReconstructionQuality);
        else if(phase_name == "Scale")
            phases.emplace_back(new Scale);
        else if(phase_name == "ScaleInBox")
            phases.emplace_back(new ScaleInBox);
        else if(phase_name == "SlowdownStopMotionDetection")
            phases.emplace_back(new SlowdownStopMotionDetection);
        else if(phase_name == "SuperPixelSegmentation")
            phases.emplace_back(new SuperPixelSegmentation);
	}
}



//---------------------------------------------------------------------------------------
void LifeCycleManager::launch(Data* data) noexcept
{
    while(data->getTime() < end_time_cycle)
    {
        for(std::unique_ptr<Phase>& phase: phases)
    	{
    		std::cout << "time " << data->getTime() << std::endl;
            phase->launch(data);
    	}

        data->nextFrame();
    }
}

















