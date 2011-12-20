#ifndef FeatureEstimationDebug_TASK_TYPES_HPP_
#define FeatureEstimationDebug_TASK_TYPES_HPP_

#include <vector>
#include <base/time.h>

namespace sonar_detectors
{

struct FeatureEstimationDebug
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
    FeatureEstimationDebug()
    : time(base::Time::now()), bestPos(0), pos_auv(0), pos_ground(0), pos_surface(0){}
};

}

#endif
