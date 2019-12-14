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
    beacons.push_back(Beacon(Position2D(0, 30), noise));

    Vehicle vehicle(Pose2D(10, 20, 2.5));

    ParticleFilter particle;
    float init_state_variance = 1000;
    float motion_variance = 0.01;
    float measurement_variance = noise * noise;
    KalmanFilter kalman(init_state_variance, motion_variance, measurement_variance);

    Viewer viewer(vehicle, beacons, particle, kalman, MAX_DIST_M);
    float speed = 2;
    float theta_dot = 0.09;
    float dt = 0.1;
    while (1) {
        vehicle.drive(speed, theta_dot, dt);
        printf("Vehicle at - x:%.3f, y:%.3f, theta:%.2f\n",
               vehicle.get_pose().x, vehicle.get_pose().y,
               vehicle.get_pose().theta * 180 / M_PI);
        particle.estimate(vehicle, beacons);
        kalman.estimate(particle.get_estimate(), speed, theta_dot, dt);
        viewer.update();
        usleep(50000);
    }

    return 0;
}
