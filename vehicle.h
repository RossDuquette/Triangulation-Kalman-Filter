#ifndef VEHICLE_H_
#define VEHICLE_H_

#include "position.h"

class Vehicle
{
    public:
        Vehicle(Pose2D& pose);
        Pose2D& get_pose();
        void drive(float speed, float theta_dot, float dt);

    private:
        Pose2D& pose_;
};

#endif
