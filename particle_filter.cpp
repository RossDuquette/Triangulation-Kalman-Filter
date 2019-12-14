#include <math.h>
#include <stdio.h>

#include "beacon.h"
#include "particle_filter.h"
#include "vehicle.h"

ParticleFilter::ParticleFilter() :
    prev_estimate_(0, 0)
{
    particles_.clear();
}

void ParticleFilter::estimate(const Vehicle vehicle, std::vector<Beacon>& beacons)
{
    float x = 0;
    float y = 0;

    // Make all measurements
    for (int b = 0; b < beacons.size(); b++) {
        beacons[b].measure(vehicle.get_pose());
    }
    particles_.clear();
    for (int b1 = 0; b1 < beacons.size(); b1++) {
        for (int b2 = b1 + 1; b2 < beacons.size(); b2++) {
            for (int b3 = b2 + 1; b3 < beacons.size(); b3++) {
                Position2D pos = triangulation(vehicle, beacons[b1],
                                               beacons[b2], beacons[b3]);
                particles_.push_back(pos);
                x += pos.x;
                y += pos.y;
            }
        }
    }
    x *= 1 / float(particles_.size());
    y *= 1 / float(particles_.size());

    // Save estimate
    prev_estimate_ = Position2D(x, y);
}

Position2D ParticleFilter::get_estimate()
{
    return prev_estimate_;
}

std::vector<Position2D>& ParticleFilter::get_particles()
{
    return particles_;
}

Position2D ParticleFilter::triangulation(Vehicle veh, Beacon b1, Beacon b2, Beacon b3)
{
    Position2D pos(0, 0);

    // Calculate 2 intersection points from beacons 1 and 2
    Position2D intersections[2] = { Position2D(0, 0), Position2D(0, 0) };
    get_circle_intersections(b1.get_position(), b1.get_distance(),
                             b2.get_position(), b2.get_distance(),
                             intersections);

    // Average using third beacon
    float diff1 = distance(b3.get_position(), intersections[0]) - b3.get_distance();
    float diff2 = distance(b3.get_position(), intersections[1]) - b3.get_distance();
    if (abs(diff1) < abs(diff2)) {
        float dx = b3.get_position().x - intersections[0].x;
        float dy = b3.get_position().y - intersections[0].y;
        float R = sqrt(dx * dx + dy * dy);
        pos.x = intersections[0].x + diff1 * dx / (2 * R);
        pos.y = intersections[0].y + diff1 * dy / (2 * R);
    } else {
        float dx = b3.get_position().x - intersections[1].x;
        float dy = b3.get_position().y - intersections[1].y;
        float R = sqrt(dx * dx + dy * dy);
        pos.x = intersections[1].x + diff2 * dx / (2 * R);
        pos.y = intersections[1].y + diff2 * dy / (2 * R);
    }

    return pos;
}
