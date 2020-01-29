#include <math.h>
#include <stdio.h>

#include "kalman_filter.h"
#include "vehicle.h"

KalmanFilter::KalmanFilter(float p0, float q, float r) :
    prev_estimate_(Pose2D(0, 0, 0)),
    p0(p0),
    q(q),
    r(r)
{
}

void KalmanFilter::estimate(Position2D measurement, float speed, float theta_dot, float dt)
{
    // Prediction
    Vehicle vehicle(prev_estimate_);
    vehicle.drive(speed, theta_dot, dt);
    Pose2D state_estimate = vehicle.get_pose();

    // Update
    float K = p0 / (p0 + r);
    float x_hat = state_estimate.x + K * (measurement.x - state_estimate.x);
    float y_hat = state_estimate.y + K * (measurement.y - state_estimate.y);
    float theta_meas = atan2(y_hat - prev_estimate_.y, x_hat - prev_estimate_.x);
    float theta_hat = state_estimate.theta + K * theta_diff(state_estimate.theta, theta_meas);

    p0 = (1 - K) * p0 + q;
    prev_estimate_ = Pose2D(x_hat, y_hat, theta_hat);
}

Pose2D KalmanFilter::get_estimate()
{
    return prev_estimate_;
}
