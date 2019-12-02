#include "beacon.h"

Beacon::Beacon(Position2D& position) :
    position_(position)
{
}

Position2D& Beacon::get_position()
{
    return position_;
}
