#include "beacon.h"

Beacon::Beacon(Position2D position, float noise_std_dev) :
    norm_dist_(0, noise_std_dev),
    position_(position)
{
}

Position2D Beacon::get_position()
{
    return position_;
}

void Beacon::measure(Position2D veh_pos)
{
    float noise = norm_dist_(gen_);
    distance_ = distance(position_, veh_pos) + noise;
}

float Beacon::get_distance()
{
    return distance_;
}
