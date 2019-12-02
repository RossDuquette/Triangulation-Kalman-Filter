#ifndef BEACON_H_
#define BEACON_H_

#include "position.h"

class Beacon
{
    public:
        Beacon(Position2D& position);
        Position2D& get_position();

    private:
        Position2D& position_;
};

#endif
