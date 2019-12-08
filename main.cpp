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

    Beacon beacon1(Position2D(10, 10));
    Beacon beacon2(Position2D(10, -10));
    Beacon beacon3(Position2D(-10, 10));
    Beacon beacon4(Position2D(-10, -10));
    Vehicle vehicle(Pose2D(0, -30, 0));

    beacons.push_back(beacon1);
    beacons.push_back(beacon2);
    beacons.push_back(beacon3);
    beacons.push_back(beacon4);

    float noise = 0; // Std dev of gaussian noise on distance measurements
    ParticleFilter particle(0);

    Viewer viewer(vehicle, beacons, particle, MAX_DIST_M);
    float speed = 0.2;
    float theta_dot = 0.01;
    float dt = 1;
    while (1) {
        vehicle.drive(speed, theta_dot, dt);
        particle.estimate(vehicle, beacons);
        viewer.update();
    }

    return 0;
}
