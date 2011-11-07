/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <sonar_detectors/SonarBeamProcessing.hpp>
#include <machine_learning/DBScan.hpp>
#include <base/samples/pointcloud.h>
#include <dsp_acoustics/FIRFilter.h>
#include <map>

using namespace sonar_feature_estimator;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TaskBase(name, engine, initial_state)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

// bool Task::configureHook()
// {
//     if (! TaskBase::configureHook())
//         return false;
//     return true;
// }

bool Task::startHook()
{
    if (! TaskBase::startHook())
         return false;
        
    // check if input ports are connected
    if (!_sonar_input.connected())
    {
        std::cerr << TaskContext::getName() << ": " 
                    << "Input port 'sonar_input' is not connected." << std::endl;
        return false;
    }
    
    featureList = featureMap.getFeatureListPtr();
    featureMap.setFeatureTimeout(15000);
    current_orientation.invalidate();
    
    return true;
}

void Task::updateHook()
{
    // read current orientation
    _orientation_sample.readNewest(current_orientation);
    
    base::samples::SonarBeam sonarBeam;
    while (_sonar_input.read(sonarBeam) == RTT::NewData) 
    {
        featureExtraction.setBoundingBox(1.5, sonarBeam.sampling_interval);
        
        std::vector<float> beam(sonarBeam.beam.size());
        dsp::movingAverageFilterSymD<std::vector<uint8_t>::const_iterator,std::vector<float>::iterator>(sonarBeam.beam.begin(), sonarBeam.beam.end(), beam.begin(), sonarBeam.beam.size() / 30);
        int index = featureExtraction.getFeatureMaximalLevelDifference(beam);
        
        if (index >= 0)
        {
            sonar_detectors::obstaclePoint feature = sonar_detectors::SonarBeamProcessing::computeObstaclePoint(index, sonarBeam, current_orientation.orientation);
            _new_feature.write(feature.position);
            featureMap.addFeature(feature, feature.angle.rad, feature.time);
        }
    }
    
    base::samples::Pointcloud pointCloud;
    pointCloud.time = base::Time::now();
    for(std::list<sonar_detectors::obstaclePoint>::const_iterator it = featureList->begin(); it != featureList->end(); it++)
    {
        pointCloud.points.push_back(it->position);
    }
    
    _features.write(pointCloud);
    
    // ------ Clustering Approach
    machine_learning::DBScan dbscan(featureList,3,2.0);
    std::map<sonar_detectors::obstaclePoint*, int> clustering = dbscan.scan();
    
    // Debug Printout
    std::cout << "Clustering:" << std::endl;
    for(std::map<sonar_detectors::obstaclePoint*, int>::iterator it = clustering.begin(); it != clustering.end(); it++) {
        std::cout << machine_learning::pointToString(it->first) << " clustered as " << it->second << std::endl;
    }
}

// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }
// void Task::stopHook()
// {
//     TaskBase::stopHook();
// }
// void Task::cleanupHook()
// {
//     TaskBase::cleanupHook();
// }

