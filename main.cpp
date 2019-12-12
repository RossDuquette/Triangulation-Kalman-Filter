#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>

#include "beacon.h"
#include "particle_filter.h"
#include "position.h"
#include "vehicle.h"
#include "viewer.h"

int main(int argc, char* argv[])
{
    const float MAX_DIST_M = 40; // -40 to 40

    std::vector<Beacon> beacons;
    beacons.push_back(Beacon(Position2D(10, 10)));
    beacons.push_back(Beacon(Position2D(10, -10)));
    beacons.push_back(Beacon(Position2D(-10, 10)));
    beacons.push_back(Beacon(Position2D(-10, -10)));
    beacons.push_back(Beacon(Position2D(0, 20)));
    beacons.push_back(Beacon(Position2D(0, -20)));

    Vehicle vehicle(Pose2D(0, -20, 0));

    float noise = 1; // Std dev of gaussian noise on distance measurements
    ParticleFilter particle(noise);

    Viewer viewer(vehicle, beacons, particle, MAX_DIST_M);
    float speed = 1;
    float theta_dot = 0.04;
    float dt = 1;
    while (1) {
        vehicle.drive(speed, theta_dot, dt);
        particle.estimate(vehicle, beacons);
        viewer.update();
        usleep(500000);
    }

    return 0;
}
