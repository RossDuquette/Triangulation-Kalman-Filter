#include <math.h>

#include "beacon.h"
#include "particle_filter.h"
#include "vehicle.h"

ParticleFilter::ParticleFilter(float noise_std_dev) :
    norm_dist_(0, noise_std_dev),
    prev_estimate_(0, 0, 0)
{
    particles_.clear();
}

void ParticleFilter::estimate(const Vehicle vehicle, const std::vector<Beacon>& beacons)
{
    float x = 0;
    float y = 0;
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

    float angle_drove = atan2(y - prev_estimate_.y, x - prev_estimate_.x);
    float angle_diff = angle_drove - prev_estimate_.theta;
    float theta = angle_drove + angle_diff;

    // Save estimate
    prev_estimate_ = Pose2D(x, y, theta);
}

Pose2D ParticleFilter::get_estimate()
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
    float noise[3];
    float dist[3];

    // Calculate noise
    noise[0] = norm_dist_(gen_);
    noise[1] = norm_dist_(gen_);
    noise[2] = norm_dist_(gen_);

    // Measure distances, including noise
    dist[0] = distance(veh.get_pose(), b1.get_position()) + noise[0];
    dist[1] = distance(veh.get_pose(), b2.get_position()) + noise[1];
    dist[2] = distance(veh.get_pose(), b3.get_position()) + noise[2];

    // Calculate 2 intersection points from beacons 1 and 2
    Position2D intersections[2] = { Position2D(0, 0), Position2D(0, 0) };
    get_circle_intersections(b1.get_position(), dist[0],
                             b2.get_position(), dist[1],
                             intersections);

    // Average using third beacon
    float diff1 = distance(b3.get_position(), intersections[0]) - dist[2];
    float diff2 = distance(b3.get_position(), intersections[1]) - dist[2];
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
