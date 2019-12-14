#ifndef BEACON_H_
#define BEACON_H_

#include <random>

#include "position.h"

class Beacon
{
    public:
        Beacon(Position2D position, float noise_std_dev);
        Position2D get_position();
        void measure(Position2D veh_pos);
        float get_distance();

    private:
        float distance_;
        Position2D position_;
        std::default_random_engine gen_;
        std::normal_distribution<float> norm_dist_;
};

#endif
