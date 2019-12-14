#ifndef VIEWER_H_
#define VIEWER_H_

#include <vector>

#include "beacon.h"
#include "particle_filter.h"
#include "kalman_filter.h"
#include "vehicle.h"

class Viewer
{
    public:
        Viewer(Vehicle& vehicle, std::vector<Beacon>& beacons,
               ParticleFilter& particle, KalmanFilter& kalman, float size_m);
        void update();

    private:
        float m_to_pix(float m);
        void draw_square(float x, float y, float size);
        void draw_circle(float x, float y, float r);
        void draw_vehicle();
        void draw_beacons();
        void draw_distances();
        void draw_particles();
        void draw_estimate();

        Vehicle& vehicle_;
        std::vector<Beacon>& beacons_;
        ParticleFilter& particle_;
        KalmanFilter& kalman_;
        const float size_m_;
        const int size_pix_;

        void set_colour(float r, float g, float b);
#define WHITE 1.0,1.0,1.0
#define BLACK 0.0,0.0,0.0
#define RED   1.0,0.0,0.0
#define GREEN 0.0,0.7,0.0
#define BLUE  0.0,0.0,0.1
};

#endif
