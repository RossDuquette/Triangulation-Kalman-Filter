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
    const float noise = 2; // Std dev of gaussian noise on distance measurements

    std::vector<Beacon> beacons;
    beacons.push_back(Beacon(Position2D(-20, 20), noise));
    beacons.push_back(Beacon(Position2D(20, 20), noise));
    beacons.push_back(Beacon(Position2D(20, -20), noise));
    beacons.push_back(Beacon(Position2D(-20, -20), noise));

    Vehicle vehicle(Pose2D(10, 20, 2.5));

    ParticleFilter particle;

    Viewer viewer(vehicle, beacons, particle, MAX_DIST_M);
    float speed = 1;
    float theta_dot = 0.04;
    float dt = 1;
    while (1) {
        vehicle.drive(speed, theta_dot, dt);
        printf("Vehicle at - x:%.3f, y:%.3f\n", vehicle.get_pose().x, vehicle.get_pose().y);
        particle.estimate(vehicle, beacons);
        viewer.update();
        usleep(100000);
    }

    return 0;
}
