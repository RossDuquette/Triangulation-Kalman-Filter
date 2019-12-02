#include <math.h>

#include "position.h"

Position2D::Position2D(float x_, float y_) :
    x(x_),
    y(y_)
{
}

Pose2D::Pose2D(float x_, float y_, float theta_) :
    Position2D(x_, y_),
    theta(theta_)
{
}

float distance(const Position2D& pos1, const Position2D& pos2)
{
    float del_x = pos1.x - pos2.x;
    float del_y = pos1.y - pos2.y;
    return sqrt(del_x * del_x + del_y * del_y);
}
