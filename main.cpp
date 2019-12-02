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
    std::vector<Beacon> beacons;
    std::vector<Vehicle> vehicles;

    Position2D pos1(10, 10);
    Position2D pos2(10, -10);
    Position2D pos3(-10, 10);
    Position2D pos4(-10, -10);
    Pose2D pose_v(-40, -40, M_PI / 4.0);
    Beacon beacon1(pos1);
    Beacon beacon2(pos2);
    Beacon beacon3(pos3);
    Beacon beacon4(pos4);
    Vehicle vehicle(pose_v);

    beacons.push_back(beacon1);
    beacons.push_back(beacon2);
    beacons.push_back(beacon3);
    beacons.push_back(beacon4);
    vehicles.push_back(vehicle);

    Viewer viewer(vehicles, beacons);
    while (1) {
        vehicle.drive(1, 0, 1);
        viewer.update();
        printf("Distance from vehicle to beacon1: %f\n",
               distance(beacon1.get_position(), vehicle.get_pose()));
        sleep(1);
    }

    return 0;
}
