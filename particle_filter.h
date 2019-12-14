#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <vector>

#include "beacon.h"
#include "position.h"
#include "vehicle.h"

// Custom particle filter in which the number of particles per vehicle
// is dependant on the number of beacons.
class ParticleFilter
{
    public:
        ParticleFilter();
        void estimate(const Vehicle vehicle, std::vector<Beacon>& beacons);
        Position2D get_estimate();
        std::vector<Position2D>& get_particles();

    private:
        Position2D triangulation(Vehicle veh, Beacon b1, Beacon b2, Beacon b3);

        std::vector<Position2D> particles_;
        Position2D prev_estimate_;
};

#endif
