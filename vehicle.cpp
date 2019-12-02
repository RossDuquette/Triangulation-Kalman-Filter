#include <math.h>

#include "vehicle.h"

Vehicle::Vehicle(Pose2D& pose) :
    pose_(pose)
{
}

Pose2D& Vehicle::get_pose()
{
    return pose_;
}

void Vehicle::drive(float speed, float theta_dot, float dt)
{
    float avg_theta = pose_.theta + theta_dot * dt;
    pose_.theta += theta_dot * dt;

    pose_.x += speed * dt * cos(avg_theta);
    pose_.y += speed * dt * sin(avg_theta);
}
