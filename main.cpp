#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>

#include "beacon.h"
#include "position.h"
#include "vehicle.h"
#include "viewer.h"

int main(int argc, char* argv[])
{
    const float MAX_DIST_M = 40; // -40 to 40
    std::vector<Beacon> beacons;
    std::vector<Vehicle> vehicles;

    Position2D pos1(10, 10);
    Position2D pos2(10, -10);
    Position2D pos3(-10, 10);
    Position2D pos4(-10, -10);
    Position2D pos5(0, 0);
    Pose2D pose_v(0, -30, 0);
    Beacon beacon1(pos1);
    Beacon beacon2(pos2);
    Beacon beacon3(pos3);
    Beacon beacon4(pos4);
    Beacon beacon5(pos5);
    Vehicle vehicle(pose_v);

    beacons.push_back(beacon1);
    beacons.push_back(beacon2);
    beacons.push_back(beacon3);
    beacons.push_back(beacon4);
    beacons.push_back(beacon5);
    vehicles.push_back(vehicle);

    Viewer viewer(vehicles, beacons, MAX_DIST_M);
    float speed = 0.2;
    float theta_dot = 0.01;
    float dt = 1;
    while (1) {
        vehicle.drive(speed, theta_dot, dt);
        viewer.update();
    }

    return 0;
}
