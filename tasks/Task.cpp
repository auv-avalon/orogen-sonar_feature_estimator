/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base/samples/pointcloud.h>
#include <base/samples/laser_scan.h>
#include <sonar_feature_estimator/FeatureEstimationDebugTypes.hpp>
#include <dsp_acoustics/FIRFilter.h>

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
        RTT::log(RTT::Error) << TaskContext::getName() << ": " 
                    << "Input port 'sonar_input' is not connected." << RTT::endlog();
        return false;
    }
    
    featureList = featureMap.getFeatureListPtr();
    featureMap.setFeatureTimeout(15000);
    secondFeatureList = secondFeatureMap.getFeatureListPtr();
    secondFeatureMap.setFeatureTimeout(15000);
    current_orientation.initUnknown();
    last_sample = base::Time::now();
    return true;
}

void Task::updateHook()
{
    // read current orientation
    _orientation_sample.readNewest(current_orientation);
    
    base::samples::SonarBeam sonarBeam;
    while (_sonar_input.read(sonarBeam) == RTT::NewData) 
    {
        // avoid sample copies
        if (sonarBeam.time == last_sample)
            return;
        last_sample = sonarBeam.time;
        
        // update sonar environment model
        model.updateSonarBeamProperties(sonarBeam.sampling_interval);
        model.updateAUVOrientation(current_orientation.orientation);
        model.updateDistanceToSurface(current_orientation.position.z() * -1);
        
        std::vector<float> filtered_beam(sonarBeam.beam.size());
        std::vector<sonar_detectors::FeatureCandidate> feature_candidates;
        int feature_index = -1;
        int feature_index_first = -1;
        int feature_index_second = -1;
        try 
        {
            // threshold filter
            float average_value = 0.0f;
            std::vector<float> temp_beam(sonarBeam.beam.size());
            dsp::proportionallyThresholdFilter<std::vector<uint8_t>::const_iterator, std::vector<float>::iterator, float>(sonarBeam.beam.begin(), sonarBeam.beam.end(), temp_beam.begin(), average_value, (float)_proportional_value_threshold, true);
            
            if(average_value < (float)_signal_threshold)
            {
                RTT::log(RTT::Info) << "This sonar beam is allmost empty, skip this one." << RTT::endlog();
            }
            else
            {
                // filter beam
                dsp::movingAverageFilterSymD<std::vector<float>::const_iterator,std::vector<float>::iterator>(temp_beam.begin(), temp_beam.end(), filtered_beam.begin(), temp_beam.size() / 30);

                // subtract noise distributions
                model.updateNoiseDistributionValues(sonarBeam.bearing.rad, filtered_beam);
                model.setSimpleAUVModelBoundary(_avalon_boundary_box_size);
                dsp::subtractFunctionFromSignal<std::vector<float>::const_iterator,std::vector<float>::iterator, float>(filtered_beam.begin(), filtered_beam.end(), filtered_beam.begin(), &model.device_noise_distribution, 255.0f, 0.0f);
                //dsp::subtractFunctionFromSignal<std::vector<float>::const_iterator,std::vector<float>::iterator, float>(filtered_beam.begin(), filtered_beam.end(), filtered_beam.begin(), &model.gaussian_distribution_surface, 255.0f, 0.0f);

                
                // compute feature index
                featureExtraction.setDerivativeFeatureConfiguration((unsigned int)_derivative_history_length, (float)_feature_threshold, (unsigned int)_best_values_size, 
                                                                        (float)_signal_balancing, (float)_plain_length, (float)_plain_threshold);
                
                feature_candidates = featureExtraction.computeDerivativeFeatureCandidates(filtered_beam);
                
                // select feature with highest possibility
                if(feature_candidates.size() > 0 && feature_candidates.front().probability > 0.0)
                {
                    feature_index = feature_candidates.front().beam_index;
                    feature_index_first = feature_candidates.front().beam_index;
                }
                
                // reweight candidates by line enforcement
                if(_enforce_line_rate > 0.0 || _minimum_enforce_line_value > 0.0)
                {
                    featureExtraction.setEnforceLinesConfiguration(5, 4, _enforce_line_rate, _minimum_enforce_line_value, _enforce_line_beam_covariance);
                    featureExtraction.enforceLines(feature_candidates, sonarBeam.bearing, sonarBeam.getSpatialResolution(), sonarBeam.beam.size());
                    
                    if(feature_candidates.size() > 0 && feature_candidates.front().probability > 0.0)
                    {
                        feature_index = feature_candidates.front().beam_index;
                        feature_index_second = feature_candidates.front().beam_index;
                    }
                }
                
                // compute feature index, obsolete version
                //featureExtraction.setMinResponseValue(20.0);
                //featureExtraction.setBoundingBox(1.5, sonarBeam.sampling_interval); 
                //feature_index = featureExtraction.getFeatureMaximalLevelDifference(filtered_beam, filtered_beam.size() / 30); 
            }
        } 
        catch (std::runtime_error e)
        {
            RTT::log(RTT::Warning) << "Error while filtering the sonarbeam: " << e.what() << RTT::endlog();
        }
        
        // create debug output
        if(_enable_debug_output)
        {
            try
            {
                // display noise distributions
                std::vector<float> device_noise(sonarBeam.beam.size());
                std::vector<float> gauss(sonarBeam.beam.size());
                
                int pos_auv, pos_ground, pos_surface;
                model.getExpectedObstaclePositions(sonarBeam.bearing.rad, pos_auv, pos_ground, pos_surface);
                
                dsp::addFunctionToSignal<std::vector<float>::const_iterator, std::vector<float>::iterator>(device_noise.begin(), device_noise.end(), device_noise.begin(), &model.device_noise_distribution, 255.0f, 0.0f);
                dsp::addFunctionToSignal<std::vector<float>::const_iterator, std::vector<float>::iterator>(gauss.begin(), gauss.end(), gauss.begin(), &model.gaussian_distribution_surface, 255.0f, 0.0f);
                
                // create 1d debug data debug data
                std::vector<float> derivative;
                float value_threshold, plain_window_threshold;
                featureExtraction.getDerivativeFeatureDebugData(derivative, value_threshold, plain_window_threshold);
                
                sonar_detectors::FeatureEstimation1DDebug debug_data;
                debug_data.filteredBeam = filtered_beam;
                debug_data.derivative = derivative;
                debug_data.device_noise_distribution = device_noise;
                debug_data.gaussian_distribution = gauss;
                debug_data.bestPos = feature_index;
                debug_data.pos_auv = pos_auv;
                debug_data.pos_ground = pos_ground;
                debug_data.pos_surface = pos_surface;
                _debug_output.write(debug_data);
                
                
                // create 2d debug data
                sonar_detectors::FeatureEstimation2DDebug debug_data_2d;
                
                std::list<sonar_detectors::HoughEntry> hough_entries;
                featureExtraction.getEnforceLinesDebugData(hough_entries, debug_data_2d.force_line_pos);
                
                // save feature as obstaclePoint if it has found one
                if (feature_index_first >= 0)
                {
                    sonar_detectors::obstaclePoint feature = sonar_detectors::FeatureExtraction::computeObstaclePoint(feature_index_first, sonarBeam, current_orientation.orientation);
                    if(feature_index_first == feature_index_second)
                        feature.position.z() = 1.0;
                    else
                        feature.position.z() = 2.0;
                    featureMap.addFeature(feature, feature.angle.rad, feature.time);
                }
                debug_data_2d.point_cloud.time = base::Time::now();
                for(std::list<sonar_detectors::obstaclePoint>::const_iterator it = featureList->begin(); it != featureList->end(); it++)
                {
                    debug_data_2d.point_cloud.points.push_back(base::Vector3d(it->position.x(), it->position.y(), it->position.z()));
                }
                
                // save diff features 
                if(feature_index_second >= 0)
                {
                    sonar_detectors::obstaclePoint feature = sonar_detectors::FeatureExtraction::computeObstaclePoint(feature_index_second, sonarBeam, current_orientation.orientation);
                    if(feature_index_first == feature_index_second)
                        feature.position.z() = -1.0;
                    else
                        feature.position.z() = -2.0;
                    secondFeatureMap.addFeature(feature, feature.angle.rad, feature.time);
                }
                debug_data_2d.point_cloud_force_line.time = base::Time::now();
                for(std::list<sonar_detectors::obstaclePoint>::const_iterator it = secondFeatureList->begin(); it != secondFeatureList->end(); it++)
                {
                    debug_data_2d.point_cloud_force_line.points.push_back(base::Vector3d(it->position.x(), it->position.y(), it->position.z()));
                }
                
                // save feature candidates
                debug_data_2d.feature_candidates.time = base::Time::now();
                for(std::vector<sonar_detectors::FeatureCandidate>::const_iterator it = feature_candidates.begin(); it != feature_candidates.end(); it++)
                {
                    sonar_detectors::obstaclePoint feature = sonar_detectors::FeatureExtraction::computeObstaclePoint(it->beam_index, sonarBeam, current_orientation.orientation);
                    debug_data_2d.feature_candidates.points.push_back(base::Vector3d(feature.position.x(), feature.position.y(), it->probability));
                }
                
                _2d_debug_output.write(debug_data_2d);
            }
            catch (std::runtime_error e)
            {
                RTT::log(RTT::Warning) << "Error while creating the debug output: " << e.what() << RTT::endlog();
            }
        }
        
        
        sonar_detectors::ObstacleFeatures features;
        features.time = sonarBeam.time;
        features.angle = sonarBeam.bearing.rad;
        
        for(std::vector<sonar_detectors::FeatureCandidate>::const_iterator it = feature_candidates.begin(); it != feature_candidates.end(); it++)
        {        
          sonar_detectors::ObstacleFeature of;
          of.confidence = it->probability;
          of.range = (uint32_t) ((it->beam_index * sonarBeam.getSpatialResolution()) * 1000);
          
          features.features.push_back(of);
          
        }
        _features_out.write(features);
        
        // write newest feature as laser scan without heading correction
        _new_feature.write(sonar_detectors::FeatureExtraction::computeLaserScan(feature_index, sonarBeam));
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

