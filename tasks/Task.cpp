/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <sonar_detectors/SonarBeamProcessing.hpp>
#include <sonar_detectors/PointClustering.hpp>
#include <machine_learning/DBScan.hpp>
#include <machine_learning/ClusteringUtils.hpp>
#include <base/samples/pointcloud.h>
#include <dsp_acoustics/FIRFilter.h>
#include <map>
#include <cassert>
#include <stdio.h>
#include <boost/foreach.hpp>
#include <vector>
#include <set>

using namespace sonar_feature_estimator;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
{
    colors.push_back(base::Vector3d(0,0,255)); // blue
    colors.push_back(base::Vector3d(255,0,0)); // red
    colors.push_back(base::Vector3d(0,255,0)); // green
    colors.push_back(base::Vector3d(255,255,0)); // yellow
    colors.push_back(base::Vector3d(255,0,255)); // magenta
    //colors.push_back(base::Vector3d(255,128,128)); // pink
    //colors.push_back(base::Vector3d(128,128,128)); // gray
    //colors.push_back(base::Vector3d(128,0,0)); // brown
    //colors.push_back(base::Vector3d(255,128,0)); // orange
    colors.push_back(base::Vector3d(0,255,255)); // cyan
    colors.push_back(base::Vector3d(0,0,0)); // black
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
        try 
        {
            dsp::movingAverageFilterSymD<std::vector<uint8_t>::const_iterator,std::vector<float>::iterator>(sonarBeam.beam.begin(), sonarBeam.beam.end(), beam.begin(), sonarBeam.beam.size() / 30);
        } 
        catch (std::runtime_error e)
        {
            RTT::log(RTT::Warning) << "Error while filtering the sonarbeam: " << e.what() << RTT::endlog();
            return;
        }
        int index = featureExtraction.getFeatureMaximalLevelDifference(beam);
        
        if (index >= 0)
        {
            sonar_detectors::obstaclePoint feature = sonar_detectors::SonarBeamProcessing::computeObstaclePoint(index, sonarBeam, current_orientation.orientation);
            _new_feature.write(feature.position);
            featureMap.addFeature(feature, feature.angle.rad, feature.time);
        }
    }
    

    
    sonar_detectors::PointClustering pc;
    
    std::list<base::Vector3d*> cl_featureList;
    for(std::list<sonar_detectors::obstaclePoint>::iterator it = featureList->begin(); it != featureList->end(); it++) {
        cl_featureList.push_back(&(*it).position);
        // printf("pushed back address %p\n",&(*it).position);
    }

    std::cout << "cl_featureList.size(): " << cl_featureList.size() << std::endl;
    
    std::vector< std::set<base::Vector3d*> > clusterVec = pc.clusterPointCloud(&cl_featureList,_DBScan_min_pts.get(), _DBScan_epsilon.get());
    
    std::cout << "clusterVec.size(): " << clusterVec.size() << std::endl;
    
    //std::cout << "set sizes in task" << std::endl;
    BOOST_FOREACH(std::set<base::Vector3d*> set, clusterVec) {
        std::cout << set.size() << std::endl;
    }

    // machine_learning::DBScan dbscan(&cl_featureList,_DBScan_min_pts.get(), _DBScan_epsilon.get());
    // std::map<base::Vector3d*, int> clustering = dbscan.scan();

    //std::cout << "Cluster count in this point cloud: " << dbscan.getClusterCount() << std::endl;
    //std::cout << "NOISE count in this point cloud: " << dbscan.getNoiseCount() << std::endl;

    // Generating color information
    std::vector<base::Vector3d> point_colors;

    base::samples::Pointcloud pointCloud;
    pointCloud.time = base::Time::now();


    if(!cl_featureList.empty()) {
        int cl_id = 0;
        BOOST_FOREACH(std::set<base::Vector3d*> set, clusterVec) {
            BOOST_FOREACH(base::Vector3d* point, set) {
                base::Vector3d distinct_color = getDistinctColor(cl_id);
                point_colors.push_back(distinct_color);
                pointCloud.points.push_back(*point);
            }
            cl_id++;
        }
        //std::cout << "colors being written out" << std::endl;
        BOOST_FOREACH(base::Vector3d color, point_colors) {
            std::cout << machine_learning::pointToString(color);
        }
        std::cout << endl;
        
        //std::cout << "cl_featureList.size() = " << cl_featureList.size() << ", point color count: " << point_colors.size() << std::endl;
        
        _features.write(pointCloud);
        _point_colors.write(point_colors);
        
    }
    //if(!cl_featureList.empty()) {
    //    BOOST_FOREACH(base::Vector3d* p, cl_featureList) {
    //        base::Vector3d distinct_color = getDistinctColor(clustering[p]);
    //       point_colors.push_back(distinct_color);
    //    }
    //   _point_colors.write(point_colors);
    //}

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

base::Vector3d Task::getDistinctColor(int cluster_id)
{
    //std::cout << "getDistinctColor(): cluster_id = " << cluster_id;
    if(cluster_id == machine_learning::DBScan::NOISE) {
        //std::cout << " with color white" << std::endl;
        return base::Vector3d(255,255,255); // white
    } else {
        assert(cluster_id >= 0 && cluster_id < colors.size());
        //std::cout << " with color " << machine_learning::pointToString(colors[cluster_id]) << std::endl;
        return colors[cluster_id];
    }
}
