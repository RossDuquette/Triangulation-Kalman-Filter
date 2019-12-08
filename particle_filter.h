#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <random>
#include <vector>

#include "beacon.h"
#include "position.h"
#include "vehicle.h"

// Custom particle filter in which the number of particles per vehicle
// is dependant on the number of beacons.
class ParticleFilter
{
    public:
        ParticleFilter(float noise_std_dev);
        void estimate(const Vehicle vehicle, const std::vector<Beacon>& beacons);
        Pose2D get_estimate();
        std::vector<Position2D>& get_particles();

    private:
        Position2D triangulation(Vehicle veh, Beacon b1, Beacon b2, Beacon b3);

        std::vector<Position2D> particles_;
        Pose2D prev_estimate_;
        std::default_random_engine gen_;
        std::normal_distribution<float> norm_dist_;
};

#endif
