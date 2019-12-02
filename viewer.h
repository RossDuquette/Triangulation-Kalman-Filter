#ifndef VIEWER_H_
#define VIEWER_H_

#include <vector>

#include "beacon.h"
#include "vehicle.h"

class Viewer
{
    public:
        Viewer(std::vector<Vehicle>& vehicles, std::vector<Beacon>& beacons);
        void update();

    private:
        void draw_obj(float x, float y);
        void draw_circle(float x, float y, float r);
        void draw_vehicles();
        void draw_beacons();
        void draw_distances();

        std::vector<Vehicle>& vehicles_;
        std::vector<Beacon>& beacons_;
        const int width_;
        const int height_;
};

#endif
