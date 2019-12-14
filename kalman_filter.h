#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "position.h"

class KalmanFilter
{
    public:
        KalmanFilter(float p0, float q, float r);
        void estimate(Position2D measurement, float speed, float theta_dot, float dt);
        Pose2D get_estimate();

    private:
        Pose2D prev_estimate_;
        float p0;
        float q;
        float r;
};

#endif
