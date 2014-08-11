#ifndef FeatureEstimationDebug_TASK_TYPES_HPP_
#define FeatureEstimationDebug_TASK_TYPES_HPP_

#include <vector>
#include <base/time.h>
#include <base/eigen.h>
#include <base/samples/pointcloud.h>

namespace sonar_detectors
{

struct FeatureEstimation1DDebug
{
    base::Time time;
    std::vector<float> filteredBeam;
    std::vector<float> derivative;
    std::vector<float> device_noise_distribution;
    std::vector<float> gaussian_distribution;
    int bestPos;
    int pos_auv;
    int pos_ground;
    int pos_surface;
    FeatureEstimation1DDebug()
    : time(base::Time::now()), bestPos(0), pos_auv(0), pos_ground(0), pos_surface(0){}
};

struct FeatureEstimation2DDebug
{
    base::Time time;
    base::samples::Pointcloud point_cloud;
    base::samples::Pointcloud point_cloud_force_line;
    base::samples::Pointcloud feature_candidates;
    std::vector<base::Vector3d> force_line_pos;
    FeatureEstimation2DDebug()
    : time(base::Time::now()){}
};


}

#endif
