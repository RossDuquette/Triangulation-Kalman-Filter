#include <math.h>
#include <ncurses.h>
#include <stdio.h>
#include <vector>

#include "beacon.h"
#include "particle_filter.h"
#include "position.h"
#include "vehicle.h"
#include "viewer.h"

static void keyboard_drive(int key, float* speed, float* theta_dot)
{
    const float speed_val = 3;
    const float theta_dot_val = 0.2;
    switch (key) {
    case 'w':
        *speed = speed_val;
        break;
    case 's':
        *speed = -speed_val;
        break;
    case 'a':
        *theta_dot = theta_dot_val;
        break;
    case 'd':
        *theta_dot = -theta_dot_val;
        break;
    case 'q':
        *speed = speed_val;
        *theta_dot = theta_dot_val;
        break;
    case 'e':
        *speed = speed_val;
        *theta_dot = -theta_dot_val;
        break;
    default:
        *speed = 0;
        *theta_dot = 0;
        break;
    }
}

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
    initscr();
    mvprintw(0, 0, "Use \"qweasd\" keys to drive\n");
    timeout(0);
    float speed;
    float theta_dot;
    float dt = 0.1;
    while (1) {
        keyboard_drive(getch(), &speed, &theta_dot);
        vehicle.drive(speed, theta_dot, dt);
        mvprintw(1, 0, "X: %.2f Y: %.2f Theta: %.2f\n",
                 vehicle.get_pose().x, vehicle.get_pose().y,
                 vehicle.get_pose().theta);
        particle.estimate(vehicle, beacons);
        kalman.estimate(particle.get_estimate(), speed, theta_dot, dt);
        viewer.update();
    }

    return 0;
}
