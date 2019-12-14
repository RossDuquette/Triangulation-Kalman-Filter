#include <math.h>
#include <stdio.h>

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

void get_circle_intersections(const Position2D& center1, const float rad1,
                              const Position2D& center2, const float rad2,
                              Position2D* intersections)
{
    float centerdx = center1.x - center2.x;
    float centerdy = center1.y - center2.y;
    // Distance between circle centers
    float R = sqrt(centerdx * centerdx + centerdy * centerdy);
    if (rad1 > (R + rad2)) { // Circle 2 enclosed in circle 1
        intersections[0].x = center2.x - centerdx * (rad1 - R - rad2) / R;
        intersections[0].y = center2.y - centerdy * (rad1 - R - rad2) / R;
        intersections[1] = intersections[0];
    } else if (rad2 > (R + rad1)) { // Circle 1 enclosed in circle 2
        intersections[0].x = center1.x + centerdx * (rad2 - R - rad1) / R;
        intersections[0].y = center1.y + centerdy * (rad2 - R - rad1) / R;
        intersections[1] = intersections[0];
    } else if (R > (rad1 + rad2)) { // Imaginary intersection between small circles
        intersections[0].x = (center1.x * rad2 + center2.x * rad1) / (rad1 + rad2);
        intersections[0].y = (center1.y * rad2 + center2.y * rad1) / (rad1 + rad2);
        intersections[1] = intersections[0];
    } else { // Two intersections
        // From https://gist.github.com/jupdike/bfe5eb23d1c395d8a0a1a4ddd94882ac
        float R2 = R * R;
        float R4 = R2 * R2;
        float r_diff2 = rad1 * rad1 - rad2 * rad2;
        float a = r_diff2 / (2 * R2);
        float c = sqrt(2 * (rad1 * rad1 + rad2 * rad2) / R2 - (r_diff2 * r_diff2) / R4 - 1);

        float fx = (center1.x + center2.x) / 2.0 + a * (center2.x - center1.x);
        float gx = c * (center2.y - center1.y) / 2.0;
        intersections[0].x = fx + gx;
        intersections[1].x = fx - gx;

        float fy = (center1.y + center2.y) / 2.0 + a * (center2.y - center1.y);
        float gy = c * (center1.x - center2.x) / 2.0;
        intersections[0].y = fy + gy;
        intersections[1].y = fy - gy;
    }
}
