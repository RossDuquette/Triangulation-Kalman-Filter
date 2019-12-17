#include <math.h>
#include <stdio.h>
#include <vector>

#include "beacon.h"
#include "particle_filter.h"
#include "position.h"
#include "vehicle.h"
#include "viewer.h"

int main(int argc, char* argv[])
{
    const float MAX_DIST_M = 40; // -40 to 40
    const float noise = 3; // Std dev of gaussian noise on distance measurements
    const float measurement_variance = noise * noise;
    const float init_state_variance = 1000;
    const float motion_variance = 0.01;

    std::vector<Beacon> beacons;
    beacons.push_back(Beacon(Position2D(-20, 20), noise));
    beacons.push_back(Beacon(Position2D(20, 20), noise));
    beacons.push_back(Beacon(Position2D(20, -20), noise));
    beacons.push_back(Beacon(Position2D(-20, -20), noise));

    Vehicle vehicle(Pose2D(0, -20, 0));

    ParticleFilter particle;
    KalmanFilter kalman(init_state_variance, motion_variance, measurement_variance);

    Viewer viewer(vehicle, beacons, particle, kalman, MAX_DIST_M);
    float speed = 5;
    float theta_dot = 0.25;
    float dt = 0.1;
    while (1) {
        vehicle.drive(speed, theta_dot, dt);
        printf("Vehicle at - x:%.3f, y:%.3f, theta:%.2f\n",
               vehicle.get_pose().x, vehicle.get_pose().y,
               vehicle.get_pose().theta * 180 / M_PI);
        particle.estimate(vehicle, beacons);
        kalman.estimate(particle.get_estimate(), speed, theta_dot, dt);
        viewer.update();
        // Continue loop with enter key
        scanf("%*1[\n]");
    }

    return 0;
}
